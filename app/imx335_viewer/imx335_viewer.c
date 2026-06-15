/*
 * imx335_viewer.c - minimal IMX335 V4L2 camera display test
 *
 * Captures NV12 frames from the RKISP V4L2 node and displays them on the
 * local LCD through DRM/KMS, falling back to fbdev if DRM is unavailable.
 *
 * Default target path on AtomPi-CA1:
 *   IMX335 -> rkisp_selfpath /dev/video1 -> 640x480 NV12 -> LCD
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <linux/videodev2.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <drm_fourcc.h>
#include <drm_mode.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#ifndef VIDEO_MAX_PLANES
#define VIDEO_MAX_PLANES 8
#endif

#define DEFAULT_VIDEO_DEV "/dev/video1"
#define DEFAULT_FB_DEV    "/dev/fb0"
#define DEFAULT_DRM_CARD  "/dev/dri/card0"
#define DEFAULT_WIDTH     640
#define DEFAULT_HEIGHT    480
#define DEFAULT_FPS       30
#define BUFFER_COUNT      4

static volatile int g_running = 1;

struct app_config {
	const char *video_dev;
	const char *fb_dev;
	const char *drm_card;
	unsigned int width;
	unsigned int height;
	unsigned int fps;
	unsigned int frame_limit;
	int rotation;
	bool drm_disabled;
	bool keep_aspect;
};

struct camera_buffer {
	void *start;
	size_t length;
};

struct camera {
	int fd;
	enum v4l2_buf_type type;
	bool multiplanar;
	unsigned int width;
	unsigned int height;
	unsigned int stride;
	unsigned int sizeimage;
	uint32_t pixfmt;
	struct camera_buffer buffers[BUFFER_COUNT];
	unsigned int buffer_count;
};

struct display {
	int fd;
	bool is_drm;
	void *map;
	size_t size;
	unsigned int width;
	unsigned int height;
	unsigned int stride_pixels;
	int bpp;
	struct fb_var_screeninfo vinfo;
	struct fb_fix_screeninfo finfo;
	uint32_t drm_conn_id;
	uint32_t drm_crtc_id;
	uint32_t drm_fb_id;
	uint32_t drm_handle;
	drmModeCrtcPtr drm_saved_crtc;
};

static void handle_signal(int sig)
{
	(void)sig;
	g_running = 0;
}

static int xioctl(int fd, unsigned long request, void *arg)
{
	int ret;

	do {
		ret = ioctl(fd, request, arg);
	} while (ret < 0 && errno == EINTR);

	return ret;
}

static uint8_t clip_u8(int value)
{
	if (value < 0)
		return 0;
	if (value > 255)
		return 255;
	return (uint8_t)value;
}

static void nv12_to_rgb(const uint8_t *data, unsigned int stride,
			unsigned int width, unsigned int height,
			unsigned int x, unsigned int y,
			uint8_t *r, uint8_t *g, uint8_t *b)
{
	const uint8_t *y_plane = data;
	const uint8_t *uv_plane = data + stride * height;
	unsigned int uv_x;
	int yy, uu, vv;
	int c, d, e;

	if (x >= width)
		x = width - 1;
	if (y >= height)
		y = height - 1;

	uv_x = x & ~1U;
	yy = y_plane[y * stride + x];
	uu = uv_plane[(y / 2) * stride + uv_x + 0];
	vv = uv_plane[(y / 2) * stride + uv_x + 1];

	c = yy - 16;
	d = uu - 128;
	e = vv - 128;
	if (c < 0)
		c = 0;

	*r = clip_u8((298 * c + 409 * e + 128) >> 8);
	*g = clip_u8((298 * c - 100 * d - 208 * e + 128) >> 8);
	*b = clip_u8((298 * c + 516 * d + 128) >> 8);
}

static uint32_t rgb_to_pixel(const struct display *disp,
			     uint8_t r, uint8_t g, uint8_t b)
{
	if (disp->bpp == 16) {
		return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
	}

	if (disp->is_drm) {
		return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
	}

	{
		uint32_t r_val = r >> (8 - disp->vinfo.red.length);
		uint32_t g_val = g >> (8 - disp->vinfo.green.length);
		uint32_t b_val = b >> (8 - disp->vinfo.blue.length);

		return (r_val << disp->vinfo.red.offset) |
		       (g_val << disp->vinfo.green.offset) |
		       (b_val << disp->vinfo.blue.offset);
	}
}

static void display_clear(const struct display *disp)
{
	memset(disp->map, 0, disp->size);
}

static void display_put_pixel(const struct display *disp,
			      unsigned int x, unsigned int y,
			      uint8_t r, uint8_t g, uint8_t b)
{
	size_t offset;
	uint32_t pixel;

	if (x >= disp->width || y >= disp->height)
		return;

	pixel = rgb_to_pixel(disp, r, g, b);
	offset = (size_t)y * disp->stride_pixels + x;

	if (disp->bpp == 16) {
		((uint16_t *)disp->map)[offset] = (uint16_t)pixel;
	} else {
		((uint32_t *)disp->map)[offset] = pixel;
	}
}

static int drm_find_crtc(int fd, drmModeResPtr res, drmModeConnectorPtr conn,
			 uint32_t *crtc_id)
{
	drmModeEncoderPtr enc;
	int i, j;

	if (conn->encoder_id) {
		enc = drmModeGetEncoder(fd, conn->encoder_id);
		if (enc) {
			if (enc->crtc_id) {
				*crtc_id = enc->crtc_id;
				drmModeFreeEncoder(enc);
				return 0;
			}
			drmModeFreeEncoder(enc);
		}
	}

	for (i = 0; i < conn->count_encoders; i++) {
		enc = drmModeGetEncoder(fd, conn->encoders[i]);
		if (!enc)
			continue;

		for (j = 0; j < res->count_crtcs; j++) {
			if (enc->possible_crtcs & (1 << j)) {
				*crtc_id = res->crtcs[j];
				drmModeFreeEncoder(enc);
				return 0;
			}
		}

		drmModeFreeEncoder(enc);
	}

	return -1;
}

static int display_open_drm(struct display *disp, const char *card)
{
	drmModeResPtr res = NULL;
	drmModeConnectorPtr conn = NULL;
	drmModeModeInfo mode;
	struct drm_mode_create_dumb creq;
	struct drm_mode_map_dumb mreq;
	uint32_t handles[4] = {0};
	uint32_t pitches[4] = {0};
	uint32_t offsets[4] = {0};
	int fd = -1;
	int i;

	fd = open(card, O_RDWR | O_CLOEXEC);
	if (fd < 0)
		return -1;

	drmSetClientCap(fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);

	res = drmModeGetResources(fd);
	if (!res)
		goto err_close;

	for (i = 0; i < res->count_connectors; i++) {
		conn = drmModeGetConnector(fd, res->connectors[i]);
		if (!conn)
			continue;
		if (conn->connection == DRM_MODE_CONNECTED &&
		    conn->count_modes > 0)
			break;
		drmModeFreeConnector(conn);
		conn = NULL;
	}

	if (!conn)
		goto err_res;

	mode = conn->modes[0];
	if (drm_find_crtc(fd, res, conn, &disp->drm_crtc_id) < 0)
		goto err_conn;

	memset(&creq, 0, sizeof(creq));
	creq.width = mode.hdisplay;
	creq.height = mode.vdisplay;
	creq.bpp = 32;

	if (xioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) < 0)
		goto err_conn;

	disp->drm_handle = creq.handle;
	handles[0] = creq.handle;
	pitches[0] = creq.pitch;

	if (drmModeAddFB2(fd, creq.width, creq.height, DRM_FORMAT_XRGB8888,
			  handles, pitches, offsets, &disp->drm_fb_id, 0) < 0) {
		if (drmModeAddFB(fd, creq.width, creq.height, 24, 32,
				 creq.pitch, creq.handle, &disp->drm_fb_id) < 0)
			goto err_destroy;
	}

	memset(&mreq, 0, sizeof(mreq));
	mreq.handle = creq.handle;
	if (xioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) < 0)
		goto err_rmfb;

	disp->map = mmap(NULL, creq.size, PROT_READ | PROT_WRITE,
			 MAP_SHARED, fd, mreq.offset);
	if (disp->map == MAP_FAILED)
		goto err_rmfb;

	disp->drm_conn_id = conn->connector_id;
	disp->drm_saved_crtc = drmModeGetCrtc(fd, disp->drm_crtc_id);

	if (drmModeSetCrtc(fd, disp->drm_crtc_id, disp->drm_fb_id, 0, 0,
			   &disp->drm_conn_id, 1, &mode) < 0)
		goto err_munmap;

	disp->fd = fd;
	disp->is_drm = true;
	disp->size = creq.size;
	disp->width = creq.width;
	disp->height = creq.height;
	disp->stride_pixels = creq.pitch / 4;
	disp->bpp = 32;

	printf("DRM display: %s %ux%u pitch=%u\n",
	       card, disp->width, disp->height, creq.pitch);

	drmModeFreeConnector(conn);
	drmModeFreeResources(res);
	return 0;

err_munmap:
	munmap(disp->map, creq.size);
	disp->map = MAP_FAILED;
err_rmfb:
	if (disp->drm_fb_id)
		drmModeRmFB(fd, disp->drm_fb_id);
err_destroy:
	{
		struct drm_mode_destroy_dumb dreq;

		memset(&dreq, 0, sizeof(dreq));
		dreq.handle = creq.handle;
		xioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
	}
err_conn:
	if (conn)
		drmModeFreeConnector(conn);
err_res:
	if (res)
		drmModeFreeResources(res);
err_close:
	close(fd);
	return -1;
}

static int display_open_fbdev(struct display *disp, const char *fbdev)
{
	int fd;

	fd = open(fbdev, O_RDWR);
	if (fd < 0)
		return -1;

	if (xioctl(fd, FBIOGET_VSCREENINFO, &disp->vinfo) < 0)
		goto err_close;
	if (xioctl(fd, FBIOGET_FSCREENINFO, &disp->finfo) < 0)
		goto err_close;

	disp->map = mmap(NULL, disp->finfo.smem_len,
			 PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (disp->map == MAP_FAILED)
		goto err_close;

	disp->fd = fd;
	disp->is_drm = false;
	disp->size = disp->finfo.smem_len;
	disp->width = disp->vinfo.xres;
	disp->height = disp->vinfo.yres;
	disp->stride_pixels = disp->vinfo.xres_virtual;
	disp->bpp = disp->vinfo.bits_per_pixel;

	printf("fbdev display: %s %ux%u bpp=%d\n",
	       fbdev, disp->width, disp->height, disp->bpp);
	return 0;

err_close:
	close(fd);
	return -1;
}

static int display_open(struct display *disp, const struct app_config *cfg)
{
	memset(disp, 0, sizeof(*disp));
	disp->fd = -1;
	disp->map = MAP_FAILED;

	if (!cfg->drm_disabled &&
	    display_open_drm(disp, cfg->drm_card) == 0)
		return 0;

	if (display_open_fbdev(disp, cfg->fb_dev) == 0)
		return 0;

	return -1;
}

static void display_close(struct display *disp)
{
	if (disp->fd < 0)
		return;

	if (disp->is_drm) {
		if (disp->drm_saved_crtc) {
			drmModeSetCrtc(disp->fd, disp->drm_saved_crtc->crtc_id,
				       disp->drm_saved_crtc->buffer_id,
				       disp->drm_saved_crtc->x,
				       disp->drm_saved_crtc->y,
				       &disp->drm_conn_id, 1,
				       &disp->drm_saved_crtc->mode);
			drmModeFreeCrtc(disp->drm_saved_crtc);
		}
		if (disp->map && disp->map != MAP_FAILED)
			munmap(disp->map, disp->size);
		if (disp->drm_fb_id)
			drmModeRmFB(disp->fd, disp->drm_fb_id);
		if (disp->drm_handle) {
			struct drm_mode_destroy_dumb dreq;

			memset(&dreq, 0, sizeof(dreq));
			dreq.handle = disp->drm_handle;
			xioctl(disp->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
		}
		close(disp->fd);
		return;
	}

	if (disp->map && disp->map != MAP_FAILED)
		munmap(disp->map, disp->size);
	close(disp->fd);
}

static int camera_open(struct camera *cam, const struct app_config *cfg)
{
	struct v4l2_capability cap;
	struct v4l2_format fmt;
	struct v4l2_streamparm parm;
	struct v4l2_requestbuffers req;
	unsigned int i;
	int fd;
	uint32_t caps;

	memset(cam, 0, sizeof(*cam));
	cam->fd = -1;

	fd = open(cfg->video_dev, O_RDWR | O_NONBLOCK);
	if (fd < 0) {
		perror("open video");
		return -1;
	}

	if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
		perror("VIDIOC_QUERYCAP");
		goto err_close;
	}

	caps = cap.device_caps ? cap.device_caps : cap.capabilities;
	if (caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
		cam->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		cam->multiplanar = true;
	} else if (caps & V4L2_CAP_VIDEO_CAPTURE) {
		cam->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cam->multiplanar = false;
	} else {
		fprintf(stderr, "%s is not a capture device\n", cfg->video_dev);
		goto err_close;
	}

	if (!(caps & V4L2_CAP_STREAMING)) {
		fprintf(stderr, "%s does not support streaming I/O\n",
			cfg->video_dev);
		goto err_close;
	}

	memset(&fmt, 0, sizeof(fmt));
	fmt.type = cam->type;
	if (cam->multiplanar) {
		fmt.fmt.pix_mp.width = cfg->width;
		fmt.fmt.pix_mp.height = cfg->height;
		fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
		fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
		fmt.fmt.pix_mp.num_planes = 1;
	} else {
		fmt.fmt.pix.width = cfg->width;
		fmt.fmt.pix.height = cfg->height;
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
		fmt.fmt.pix.field = V4L2_FIELD_NONE;
	}

	if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
		perror("VIDIOC_S_FMT");
		goto err_close;
	}

	if (cam->multiplanar) {
		cam->width = fmt.fmt.pix_mp.width;
		cam->height = fmt.fmt.pix_mp.height;
		cam->pixfmt = fmt.fmt.pix_mp.pixelformat;
		cam->stride = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
		cam->sizeimage = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
	} else {
		cam->width = fmt.fmt.pix.width;
		cam->height = fmt.fmt.pix.height;
		cam->pixfmt = fmt.fmt.pix.pixelformat;
		cam->stride = fmt.fmt.pix.bytesperline;
		cam->sizeimage = fmt.fmt.pix.sizeimage;
	}

	if (cam->pixfmt != V4L2_PIX_FMT_NV12) {
		fprintf(stderr, "Only NV12 is supported in this test program\n");
		goto err_close;
	}
	if (cam->stride == 0)
		cam->stride = cam->width;

	memset(&parm, 0, sizeof(parm));
	parm.type = cam->type;
	parm.parm.capture.timeperframe.numerator = 1;
	parm.parm.capture.timeperframe.denominator = cfg->fps;
	if (xioctl(fd, VIDIOC_S_PARM, &parm) < 0)
		fprintf(stderr, "VIDIOC_S_PARM failed, continuing: %s\n",
			strerror(errno));

	memset(&req, 0, sizeof(req));
	req.count = BUFFER_COUNT;
	req.type = cam->type;
	req.memory = V4L2_MEMORY_MMAP;

	if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
		perror("VIDIOC_REQBUFS");
		goto err_close;
	}
	if (req.count < 2) {
		fprintf(stderr, "Not enough V4L2 buffers\n");
		goto err_close;
	}

	cam->buffer_count = req.count;
	if (cam->buffer_count > BUFFER_COUNT)
		cam->buffer_count = BUFFER_COUNT;

	for (i = 0; i < cam->buffer_count; i++) {
		struct v4l2_buffer buf;
		struct v4l2_plane planes[VIDEO_MAX_PLANES];
		size_t length;
		off_t offset;

		memset(&buf, 0, sizeof(buf));
		memset(planes, 0, sizeof(planes));
		buf.type = cam->type;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;

		if (cam->multiplanar) {
			buf.length = VIDEO_MAX_PLANES;
			buf.m.planes = planes;
		}

		if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
			perror("VIDIOC_QUERYBUF");
			goto err_unmap;
		}

		if (cam->multiplanar) {
			length = planes[0].length;
			offset = planes[0].m.mem_offset;
		} else {
			length = buf.length;
			offset = buf.m.offset;
		}

		cam->buffers[i].length = length;
		cam->buffers[i].start = mmap(NULL, length, PROT_READ | PROT_WRITE,
					     MAP_SHARED, fd, offset);
		if (cam->buffers[i].start == MAP_FAILED) {
			perror("mmap video");
			goto err_unmap;
		}
	}

	for (i = 0; i < cam->buffer_count; i++) {
		struct v4l2_buffer buf;
		struct v4l2_plane planes[VIDEO_MAX_PLANES];

		memset(&buf, 0, sizeof(buf));
		memset(planes, 0, sizeof(planes));
		buf.type = cam->type;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;

		if (cam->multiplanar) {
			buf.length = VIDEO_MAX_PLANES;
			buf.m.planes = planes;
		}

		if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
			perror("VIDIOC_QBUF");
			goto err_unmap;
		}
	}

	if (xioctl(fd, VIDIOC_STREAMON, &cam->type) < 0) {
		perror("VIDIOC_STREAMON");
		goto err_unmap;
	}

	cam->fd = fd;

	printf("Camera: %s %ux%u NV12 stride=%u size=%u buffers=%u %s\n",
	       cfg->video_dev, cam->width, cam->height, cam->stride,
	       cam->sizeimage, cam->buffer_count,
	       cam->multiplanar ? "mplane" : "single-plane");
	return 0;

err_unmap:
	for (i = 0; i < cam->buffer_count; i++) {
		if (cam->buffers[i].start &&
		    cam->buffers[i].start != MAP_FAILED)
			munmap(cam->buffers[i].start, cam->buffers[i].length);
	}
err_close:
	close(fd);
	return -1;
}

static int camera_dequeue(struct camera *cam, unsigned int *index)
{
	struct v4l2_buffer buf;
	struct v4l2_plane planes[VIDEO_MAX_PLANES];

	memset(&buf, 0, sizeof(buf));
	memset(planes, 0, sizeof(planes));
	buf.type = cam->type;
	buf.memory = V4L2_MEMORY_MMAP;

	if (cam->multiplanar) {
		buf.length = VIDEO_MAX_PLANES;
		buf.m.planes = planes;
	}

	if (xioctl(cam->fd, VIDIOC_DQBUF, &buf) < 0) {
		if (errno == EAGAIN)
			return 1;
		perror("VIDIOC_DQBUF");
		return -1;
	}

	if (buf.index >= cam->buffer_count) {
		fprintf(stderr, "Invalid buffer index %u\n", buf.index);
		return -1;
	}

	*index = buf.index;
	return 0;
}

static int camera_queue(struct camera *cam, unsigned int index)
{
	struct v4l2_buffer buf;
	struct v4l2_plane planes[VIDEO_MAX_PLANES];

	memset(&buf, 0, sizeof(buf));
	memset(planes, 0, sizeof(planes));
	buf.type = cam->type;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = index;

	if (cam->multiplanar) {
		buf.length = VIDEO_MAX_PLANES;
		buf.m.planes = planes;
	}

	if (xioctl(cam->fd, VIDIOC_QBUF, &buf) < 0) {
		perror("VIDIOC_QBUF");
		return -1;
	}

	return 0;
}

static void camera_close(struct camera *cam)
{
	unsigned int i;

	if (cam->fd >= 0)
		xioctl(cam->fd, VIDIOC_STREAMOFF, &cam->type);

	for (i = 0; i < cam->buffer_count; i++) {
		if (cam->buffers[i].start &&
		    cam->buffers[i].start != MAP_FAILED)
			munmap(cam->buffers[i].start, cam->buffers[i].length);
	}

	if (cam->fd >= 0)
		close(cam->fd);
}

static void map_rotated_coord(const struct camera *cam, int rotation,
			      unsigned int u, unsigned int v,
			      unsigned int *src_x, unsigned int *src_y)
{
	switch (rotation) {
	case 90:
		*src_x = v;
		*src_y = cam->height - 1 - u;
		break;
	case 180:
		*src_x = cam->width - 1 - u;
		*src_y = cam->height - 1 - v;
		break;
	case 270:
		*src_x = cam->width - 1 - v;
		*src_y = u;
		break;
	case 0:
	default:
		*src_x = u;
		*src_y = v;
		break;
	}
}

static void render_frame(const struct display *disp, const struct camera *cam,
			 const uint8_t *nv12, const struct app_config *cfg)
{
	unsigned int src_view_w;
	unsigned int src_view_h;
	unsigned int dst_w;
	unsigned int dst_h;
	unsigned int dst_x;
	unsigned int dst_y;
	unsigned int y;

	if (cfg->rotation == 90 || cfg->rotation == 270) {
		src_view_w = cam->height;
		src_view_h = cam->width;
	} else {
		src_view_w = cam->width;
		src_view_h = cam->height;
	}

	if (cfg->keep_aspect) {
		if ((uint64_t)disp->width * src_view_h >
		    (uint64_t)disp->height * src_view_w) {
			dst_h = disp->height;
			dst_w = (unsigned int)((uint64_t)dst_h * src_view_w /
					       src_view_h);
		} else {
			dst_w = disp->width;
			dst_h = (unsigned int)((uint64_t)dst_w * src_view_h /
					       src_view_w);
		}
	} else {
		dst_w = disp->width;
		dst_h = disp->height;
	}

	if (dst_w == 0)
		dst_w = 1;
	if (dst_h == 0)
		dst_h = 1;

	dst_x = (disp->width - dst_w) / 2;
	dst_y = (disp->height - dst_h) / 2;

	for (y = 0; y < dst_h; y++) {
		unsigned int x;
		unsigned int v = (unsigned int)((uint64_t)y * src_view_h / dst_h);

		for (x = 0; x < dst_w; x++) {
			unsigned int u;
			unsigned int src_x;
			unsigned int src_y;
			uint8_t r, g, b;

			u = (unsigned int)((uint64_t)x * src_view_w / dst_w);
			if (u >= src_view_w)
				u = src_view_w - 1;
			if (v >= src_view_h)
				v = src_view_h - 1;

			map_rotated_coord(cam, cfg->rotation, u, v,
					  &src_x, &src_y);
			nv12_to_rgb(nv12, cam->stride, cam->width, cam->height,
				    src_x, src_y, &r, &g, &b);
			display_put_pixel(disp, dst_x + x, dst_y + y, r, g, b);
		}
	}
}

static double time_delta_sec(const struct timespec *a,
			     const struct timespec *b)
{
	return (a->tv_sec - b->tv_sec) +
	       (a->tv_nsec - b->tv_nsec) / 1000000000.0;
}

static int run_loop(struct camera *cam, const struct display *disp,
		    const struct app_config *cfg)
{
	struct timespec fps_start;
	unsigned int frame_count = 0;
	unsigned int fps_frames = 0;

	clock_gettime(CLOCK_MONOTONIC, &fps_start);

	while (g_running) {
		fd_set fds;
		struct timeval tv;
		unsigned int index = 0;
		int ret;

		if (cfg->frame_limit && frame_count >= cfg->frame_limit)
			break;

		FD_ZERO(&fds);
		FD_SET(cam->fd, &fds);
		tv.tv_sec = 2;
		tv.tv_usec = 0;

		ret = select(cam->fd + 1, &fds, NULL, NULL, &tv);
		if (ret < 0) {
			if (errno == EINTR)
				continue;
			perror("select");
			return -1;
		}
		if (ret == 0) {
			fprintf(stderr, "Camera timeout\n");
			return -1;
		}

		ret = camera_dequeue(cam, &index);
		if (ret < 0)
			return -1;
		if (ret > 0)
			continue;

		render_frame(disp, cam, cam->buffers[index].start, cfg);

		if (camera_queue(cam, index) < 0)
			return -1;

		frame_count++;
		fps_frames++;

		if (fps_frames >= cfg->fps) {
			struct timespec now;
			double elapsed;

			clock_gettime(CLOCK_MONOTONIC, &now);
			elapsed = time_delta_sec(&now, &fps_start);
			if (elapsed > 0.0) {
				printf("frames=%u fps=%.1f\n",
				       frame_count, fps_frames / elapsed);
				fflush(stdout);
			}
			fps_start = now;
			fps_frames = 0;
		}
	}

	return 0;
}

static void usage(const char *prog)
{
	printf("IMX335 V4L2 camera viewer\n");
	printf("Usage: %s [options]\n", prog);
	printf("Options:\n");
	printf("  -v <dev>     V4L2 device (default: %s)\n", DEFAULT_VIDEO_DEV);
	printf("  -d <dev>     fbdev fallback device (default: %s)\n", DEFAULT_FB_DEV);
	printf("  -k <dev>     DRM card (default: %s)\n", DEFAULT_DRM_CARD);
	printf("  -W <width>   Capture width (default: %u)\n", DEFAULT_WIDTH);
	printf("  -H <height>  Capture height (default: %u)\n", DEFAULT_HEIGHT);
	printf("  -r <fps>     Capture FPS request (default: %u)\n", DEFAULT_FPS);
	printf("  -R <deg>     Rotate display: 0, 90, 180, 270 (default: 0)\n");
	printf("  -n <count>   Stop after count frames (default: 0/infinite)\n");
	printf("  -s           Stretch to full screen instead of keeping aspect\n");
	printf("  -B           Disable DRM and use fbdev directly\n");
	printf("  -h           Show this help\n");
}

int main(int argc, char **argv)
{
	struct app_config cfg = {
		.video_dev = DEFAULT_VIDEO_DEV,
		.fb_dev = DEFAULT_FB_DEV,
		.drm_card = DEFAULT_DRM_CARD,
		.width = DEFAULT_WIDTH,
		.height = DEFAULT_HEIGHT,
		.fps = DEFAULT_FPS,
		.frame_limit = 0,
		.rotation = 0,
		.drm_disabled = false,
		.keep_aspect = true,
	};
	struct camera cam;
	struct display disp;
	int opt;
	int ret = 1;

	while ((opt = getopt(argc, argv, "v:d:k:W:H:r:R:n:sBh")) != -1) {
		switch (opt) {
		case 'v':
			cfg.video_dev = optarg;
			break;
		case 'd':
			cfg.fb_dev = optarg;
			break;
		case 'k':
			cfg.drm_card = optarg;
			break;
		case 'W':
			cfg.width = (unsigned int)atoi(optarg);
			break;
		case 'H':
			cfg.height = (unsigned int)atoi(optarg);
			break;
		case 'r':
			cfg.fps = (unsigned int)atoi(optarg);
			if (cfg.fps == 0)
				cfg.fps = DEFAULT_FPS;
			break;
		case 'R':
			cfg.rotation = atoi(optarg);
			if (cfg.rotation != 0 && cfg.rotation != 90 &&
			    cfg.rotation != 180 && cfg.rotation != 270) {
				fprintf(stderr, "Invalid rotation: %s\n", optarg);
				return 1;
			}
			break;
		case 'n':
			cfg.frame_limit = (unsigned int)atoi(optarg);
			break;
		case 's':
			cfg.keep_aspect = false;
			break;
		case 'B':
			cfg.drm_disabled = true;
			break;
		case 'h':
			usage(argv[0]);
			return 0;
		default:
			usage(argv[0]);
			return 1;
		}
	}

	if (cfg.width == 0 || cfg.height == 0) {
		fprintf(stderr, "Invalid capture size\n");
		return 1;
	}

	signal(SIGINT, handle_signal);
	signal(SIGTERM, handle_signal);

	printf("=== IMX335 Viewer ===\n");
	printf("Video=%s request=%ux%u@%u rotation=%d\n",
	       cfg.video_dev, cfg.width, cfg.height, cfg.fps, cfg.rotation);

	if (camera_open(&cam, &cfg) < 0)
		return 1;

	if (display_open(&disp, &cfg) < 0) {
		fprintf(stderr, "Failed to open DRM/fb display\n");
		goto out_camera;
	}

	display_clear(&disp);
	ret = run_loop(&cam, &disp, &cfg);
	display_clear(&disp);
	display_close(&disp);

out_camera:
	camera_close(&cam);
	printf("Exit: %s\n", ret == 0 ? "ok" : "error");
	return ret == 0 ? 0 : 1;
}
