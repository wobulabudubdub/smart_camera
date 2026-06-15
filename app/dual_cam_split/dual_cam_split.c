/*
 * dual_cam_split.c - split-screen IMX335 visible camera + MLX90640 thermal
 *
 * First-stage integration app:
 *   left  panel: IMX335 via V4L2/RKISP
 *   right panel: MLX90640 via the existing thermal_cam sensor path
 *
 * This intentionally reuses thermal_cam.c internals for the first runnable
 * version. Once the dual path settles, those pieces can be split into common
 * camera/display/thermal modules.
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <pthread.h>
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

#define main thermal_cam_standalone_main
#include "../thermal_cam/thermal_cam.c"
#undef main

#ifdef SCALE_BAR_WIDTH
#undef SCALE_BAR_WIDTH
#endif
#ifdef MARGIN
#undef MARGIN
#endif

#ifndef VIDEO_MAX_PLANES
#define VIDEO_MAX_PLANES 8
#endif

#define DUAL_DEFAULT_VIDEO_DEV "/dev/video1"
#define DUAL_DEFAULT_FB_DEV    "/dev/fb0"
#define DUAL_DEFAULT_CAM_W     640
#define DUAL_DEFAULT_CAM_H     480
#define DUAL_DEFAULT_CAM_FPS   30
#define DUAL_DEFAULT_THERM_FPS 8
#define DUAL_DEFAULT_DISPLAY_FPS 15
#define DUAL_BUFFER_COUNT      4

#define PANEL_PAD      12
#define PANEL_TITLE_H  38
#define PANEL_INFO_H   104
#define DIVIDER_W      2

struct rect {
	int x;
	int y;
	int w;
	int h;
};

struct dual_config {
	const char *video_dev;
	const char *fbdev;
	const char *iiopath;
	unsigned int cam_w;
	unsigned int cam_h;
	unsigned int cam_fps;
	unsigned int thermal_fps;
	unsigned int display_fps;
	unsigned int frame_limit;
	int cam_rotation;
	int thermal_rotation;
	bool keep_aspect;
};

struct rgb_buffer {
	void *start;
	size_t length;
};

struct rgb_camera {
	int fd;
	enum v4l2_buf_type type;
	bool multiplanar;
	unsigned int width;
	unsigned int height;
	unsigned int stride;
	unsigned int sizeimage;
	uint32_t pixfmt;
	struct rgb_buffer buffers[DUAL_BUFFER_COUNT];
	unsigned int buffer_count;
};

struct rgb_state {
	pthread_mutex_t lock;
	uint8_t *frame;
	size_t frame_size;
	unsigned int frames;
	double fps;
	bool have_frame;
	bool error;
};

struct thermal_state {
	pthread_mutex_t lock;
	s32 *frame;
	unsigned int frames;
	double fps;
	s32 raw_min;
	s32 raw_max;
	s32 temp_center;
	bool have_frame;
	bool error;
};

struct rgb_thread_arg {
	struct rgb_camera *cam;
	struct rgb_state *state;
	const struct dual_config *cfg;
};

struct thermal_thread_arg {
	struct sensor_ctx *sensor;
	struct thermal_state *state;
	const struct dual_config *cfg;
};

static int v4l2_xioctl(int fd, unsigned long request, void *arg)
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

static int rgb_camera_open(struct rgb_camera *cam,
			   const struct dual_config *cfg)
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

	if (v4l2_xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
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
		fmt.fmt.pix_mp.width = cfg->cam_w;
		fmt.fmt.pix_mp.height = cfg->cam_h;
		fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
		fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
		fmt.fmt.pix_mp.num_planes = 1;
	} else {
		fmt.fmt.pix.width = cfg->cam_w;
		fmt.fmt.pix.height = cfg->cam_h;
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
		fmt.fmt.pix.field = V4L2_FIELD_NONE;
	}

	if (v4l2_xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
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
		fprintf(stderr, "Only NV12 is supported for RGB camera input\n");
		goto err_close;
	}
	if (cam->stride == 0)
		cam->stride = cam->width;

	memset(&parm, 0, sizeof(parm));
	parm.type = cam->type;
	parm.parm.capture.timeperframe.numerator = 1;
	parm.parm.capture.timeperframe.denominator = cfg->cam_fps;
	if (v4l2_xioctl(fd, VIDIOC_S_PARM, &parm) < 0)
		fprintf(stderr, "VIDIOC_S_PARM failed, continuing: %s\n",
			strerror(errno));

	memset(&req, 0, sizeof(req));
	req.count = DUAL_BUFFER_COUNT;
	req.type = cam->type;
	req.memory = V4L2_MEMORY_MMAP;

	if (v4l2_xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
		perror("VIDIOC_REQBUFS");
		goto err_close;
	}
	if (req.count < 2) {
		fprintf(stderr, "Not enough V4L2 buffers\n");
		goto err_close;
	}

	cam->buffer_count = req.count;
	if (cam->buffer_count > DUAL_BUFFER_COUNT)
		cam->buffer_count = DUAL_BUFFER_COUNT;

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

		if (v4l2_xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
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

		if (v4l2_xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
			perror("VIDIOC_QBUF");
			goto err_unmap;
		}
	}

	if (v4l2_xioctl(fd, VIDIOC_STREAMON, &cam->type) < 0) {
		perror("VIDIOC_STREAMON");
		goto err_unmap;
	}

	cam->fd = fd;

	printf("RGB camera: %s %ux%u NV12 stride=%u buffers=%u %s\n",
	       cfg->video_dev, cam->width, cam->height, cam->stride,
	       cam->buffer_count, cam->multiplanar ? "mplane" : "single-plane");
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

static int rgb_camera_dequeue(struct rgb_camera *cam, unsigned int *index)
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

	if (v4l2_xioctl(cam->fd, VIDIOC_DQBUF, &buf) < 0) {
		if (errno == EAGAIN)
			return 1;
		perror("VIDIOC_DQBUF");
		return -1;
	}

	if (buf.index >= cam->buffer_count) {
		fprintf(stderr, "Invalid V4L2 buffer index %u\n", buf.index);
		return -1;
	}

	*index = buf.index;
	return 0;
}

static int rgb_camera_queue(struct rgb_camera *cam, unsigned int index)
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

	if (v4l2_xioctl(cam->fd, VIDIOC_QBUF, &buf) < 0) {
		perror("VIDIOC_QBUF");
		return -1;
	}

	return 0;
}

static void rgb_camera_close(struct rgb_camera *cam)
{
	unsigned int i;

	if (cam->fd >= 0)
		v4l2_xioctl(cam->fd, VIDIOC_STREAMOFF, &cam->type);

	for (i = 0; i < cam->buffer_count; i++) {
		if (cam->buffers[i].start &&
		    cam->buffers[i].start != MAP_FAILED)
			munmap(cam->buffers[i].start, cam->buffers[i].length);
	}

	if (cam->fd >= 0)
		close(cam->fd);
}

static struct rect fit_rect(struct rect bounds, unsigned int src_w,
			    unsigned int src_h, bool keep_aspect)
{
	struct rect dst = bounds;

	if (!keep_aspect || src_w == 0 || src_h == 0)
		return dst;

	if ((uint64_t)bounds.w * src_h > (uint64_t)bounds.h * src_w) {
		dst.h = bounds.h;
		dst.w = (int)((uint64_t)dst.h * src_w / src_h);
	} else {
		dst.w = bounds.w;
		dst.h = (int)((uint64_t)dst.w * src_h / src_w);
	}

	if (dst.w < 1)
		dst.w = 1;
	if (dst.h < 1)
		dst.h = 1;

	dst.x = bounds.x + (bounds.w - dst.w) / 2;
	dst.y = bounds.y + (bounds.h - dst.h) / 2;
	return dst;
}

static void map_rotated_coord(unsigned int src_w, unsigned int src_h,
			      int rotation, unsigned int u, unsigned int v,
			      unsigned int *src_x, unsigned int *src_y)
{
	switch (rotation) {
	case 90:
		*src_x = v;
		*src_y = src_h - 1 - u;
		break;
	case 180:
		*src_x = src_w - 1 - u;
		*src_y = src_h - 1 - v;
		break;
	case 270:
		*src_x = src_w - 1 - v;
		*src_y = u;
		break;
	case 0:
	default:
		*src_x = u;
		*src_y = v;
		break;
	}
}

static void draw_text_line(const struct fb_info *fb, int x, int y,
			   uint32_t color, const char *text)
{
	fb_draw_text(fb, x, y, color, text);
}

static void draw_static_layout(const struct fb_info *fb,
			       struct rect left, struct rect right)
{
	uint32_t black = rgb_to_pixel(fb, 0, 0, 0);
	uint32_t dim = rgb_to_pixel(fb, 32, 32, 32);
	uint32_t white = rgb_to_pixel(fb, 255, 255, 255);

	fb_fill_rect(fb, 0, 0, fb->vinfo.xres, fb->vinfo.yres, black);
	fb_fill_rect(fb, left.x + left.w, 0, DIVIDER_W, fb->vinfo.yres, dim);

	fb_fill_rect(fb, left.x, left.y, left.w, PANEL_TITLE_H, dim);
	fb_fill_rect(fb, right.x, right.y, right.w, PANEL_TITLE_H, dim);
	draw_text_line(fb, left.x + PANEL_PAD, left.y + 8, white, "RGB IMX335");
	draw_text_line(fb, right.x + PANEL_PAD, right.y + 8, white, "THERMAL MLX90640");
}

static void draw_rgb_panel(const struct fb_info *fb,
			   const struct rgb_camera *cam,
			   const uint8_t *nv12,
			   struct rect panel,
			   const struct dual_config *cfg,
			   unsigned int frame_count,
			   double fps)
{
	struct rect bounds;
	struct rect dst;
	unsigned int view_w;
	unsigned int view_h;
	uint32_t bg = rgb_to_pixel(fb, 0, 0, 0);
	uint32_t white = rgb_to_pixel(fb, 255, 255, 255);
	char line[96];
	int y;

	bounds.x = panel.x + PANEL_PAD;
	bounds.y = panel.y + PANEL_TITLE_H + PANEL_PAD;
	bounds.w = panel.w - PANEL_PAD * 2;
	bounds.h = panel.h - PANEL_TITLE_H - PANEL_INFO_H - PANEL_PAD * 2;

	if (cfg->cam_rotation == 90 || cfg->cam_rotation == 270) {
		view_w = cam->height;
		view_h = cam->width;
	} else {
		view_w = cam->width;
		view_h = cam->height;
	}

	dst = fit_rect(bounds, view_w, view_h, cfg->keep_aspect);

	for (y = 0; y < dst.h; y++) {
		int x;
		unsigned int v = (unsigned int)((uint64_t)y * view_h / dst.h);

		for (x = 0; x < dst.w; x++) {
			unsigned int u;
			unsigned int src_x;
			unsigned int src_y;
			uint8_t r, g, b;

			u = (unsigned int)((uint64_t)x * view_w / dst.w);
			if (u >= view_w)
				u = view_w - 1;
			if (v >= view_h)
				v = view_h - 1;

			map_rotated_coord(cam->width, cam->height,
					  cfg->cam_rotation, u, v,
					  &src_x, &src_y);
			nv12_to_rgb(nv12, cam->stride, cam->width, cam->height,
				    src_x, src_y, &r, &g, &b);
			fb_put_pixel(fb, dst.x + x, dst.y + y,
				     rgb_to_pixel(fb, r, g, b));
		}
	}

	fb_fill_rect(fb, panel.x, panel.y + panel.h - PANEL_INFO_H,
		     panel.w, PANEL_INFO_H, bg);
	snprintf(line, sizeof(line), "RGB %ux%u ROT %d",
		 cam->width, cam->height, cfg->cam_rotation);
	draw_text_line(fb, panel.x + PANEL_PAD,
		       panel.y + panel.h - PANEL_INFO_H + 8, white, line);
	snprintf(line, sizeof(line), "FPS %4.1f FRAMES %u", fps, frame_count);
	draw_text_line(fb, panel.x + PANEL_PAD,
		       panel.y + panel.h - PANEL_INFO_H + 8 + TEXT_LINE_HEIGHT,
		       white, line);
}

static double thermal_sample_bilinear(const s32 *frame, double sx, double sy)
{
	int x0, y0, x1, y1;
	double fx, fy;
	double v00, v10, v01, v11;

	if (sx < 0.0)
		sx = 0.0;
	if (sy < 0.0)
		sy = 0.0;
	if (sx > MLX90640_COLS - 1)
		sx = MLX90640_COLS - 1;
	if (sy > MLX90640_ROWS - 1)
		sy = MLX90640_ROWS - 1;

	x0 = (int)sx;
	y0 = (int)sy;
	x1 = x0 + 1 < MLX90640_COLS ? x0 + 1 : x0;
	y1 = y0 + 1 < MLX90640_ROWS ? y0 + 1 : y0;
	fx = sx - x0;
	fy = sy - y0;

	v00 = frame[y0 * MLX90640_COLS + x0];
	v10 = frame[y0 * MLX90640_COLS + x1];
	v01 = frame[y1 * MLX90640_COLS + x0];
	v11 = frame[y1 * MLX90640_COLS + x1];

	return v00 * (1.0 - fx) * (1.0 - fy) +
	       v10 * fx * (1.0 - fy) +
	       v01 * (1.0 - fx) * fy +
	       v11 * fx * fy;
}

static void map_thermal_rotated(int rotation, double u, double v,
				double *sx, double *sy)
{
	switch (rotation) {
	case 90:
		*sx = v;
		*sy = (MLX90640_ROWS - 1) - u;
		break;
	case 180:
		*sx = (MLX90640_COLS - 1) - u;
		*sy = (MLX90640_ROWS - 1) - v;
		break;
	case 270:
		*sx = (MLX90640_COLS - 1) - v;
		*sy = u;
		break;
	case 0:
	default:
		*sx = u;
		*sy = v;
		break;
	}
}

static void draw_cross(const struct fb_info *fb, int cx, int cy, uint32_t color)
{
	int d;

	for (d = -8; d <= 8; d++) {
		fb_put_pixel(fb, cx + d, cy, color);
		fb_put_pixel(fb, cx, cy + d, color);
	}
}

static void draw_thermal_panel(const struct fb_info *fb,
			       const s32 *frame,
			       struct rect panel,
			       const struct dual_config *cfg,
			       s32 raw_min, s32 raw_max, s32 temp_center,
			       unsigned int frame_count, double fps)
{
	struct rect bounds;
	struct rect dst;
	unsigned int view_w;
	unsigned int view_h;
	s32 display_min;
	s32 display_max;
	s32 range = raw_max - raw_min;
	uint32_t bg = rgb_to_pixel(fb, 0, 0, 0);
	uint32_t white = rgb_to_pixel(fb, 255, 255, 255);
	char line[128];
	int y;

	if (range < 1000)
		range = 1000;
	display_min = raw_min - range / 10;
	display_max = raw_max + range / 10;

	bounds.x = panel.x + PANEL_PAD;
	bounds.y = panel.y + PANEL_TITLE_H + PANEL_PAD;
	bounds.w = panel.w - PANEL_PAD * 2;
	bounds.h = panel.h - PANEL_TITLE_H - PANEL_INFO_H - PANEL_PAD * 2;

	if (cfg->thermal_rotation == 90 || cfg->thermal_rotation == 270) {
		view_w = MLX90640_ROWS;
		view_h = MLX90640_COLS;
	} else {
		view_w = MLX90640_COLS;
		view_h = MLX90640_ROWS;
	}

	dst = fit_rect(bounds, view_w, view_h, cfg->keep_aspect);

	for (y = 0; y < dst.h; y++) {
		int x;
		double v = (double)y / (double)(dst.h > 1 ? dst.h - 1 : 1) *
			   (double)(view_h - 1);

		for (x = 0; x < dst.w; x++) {
			double u;
			double sx;
			double sy;
			double temp;
			uint8_t ci;

			u = (double)x / (double)(dst.w > 1 ? dst.w - 1 : 1) *
			    (double)(view_w - 1);
			map_thermal_rotated(cfg->thermal_rotation, u, v, &sx, &sy);
			temp = thermal_sample_bilinear(frame, sx, sy);
			ci = temp_to_cmap((s32)temp, display_min, display_max);
			fb_put_pixel(fb, dst.x + x, dst.y + y,
				     rgb_to_pixel(fb, colormap[ci].r,
						  colormap[ci].g,
						  colormap[ci].b));
		}
	}

	{
		int cx = dst.x + dst.w / 2;
		int cy = dst.y + dst.h / 2;

		draw_cross(fb, cx, cy, white);
		snprintf(line, sizeof(line), "%4.1fC", temp_center / 1000.0);
		draw_text_line(fb, cx - 34, cy - 34, white, line);
	}

	fb_fill_rect(fb, panel.x, panel.y + panel.h - PANEL_INFO_H,
		     panel.w, PANEL_INFO_H, bg);
	snprintf(line, sizeof(line), "MAX %5.1fC MIN %5.1fC",
		 raw_max / 1000.0, raw_min / 1000.0);
	draw_text_line(fb, panel.x + PANEL_PAD,
		       panel.y + panel.h - PANEL_INFO_H + 8, white, line);
	snprintf(line, sizeof(line), "CTR %5.1fC ROT %d",
		 temp_center / 1000.0, cfg->thermal_rotation);
	draw_text_line(fb, panel.x + PANEL_PAD,
		       panel.y + panel.h - PANEL_INFO_H + 8 + TEXT_LINE_HEIGHT,
		       white, line);
	snprintf(line, sizeof(line), "FPS %4.1f FRAMES %u", fps, frame_count);
	draw_text_line(fb, panel.x + PANEL_PAD,
		       panel.y + panel.h - PANEL_INFO_H + 8 + TEXT_LINE_HEIGHT * 2,
		       white, line);
}

static double elapsed_sec(const struct timespec *now, const struct timespec *old)
{
	return (now->tv_sec - old->tv_sec) +
	       (now->tv_nsec - old->tv_nsec) / 1000000000.0;
}

static void *rgb_capture_thread(void *data)
{
	struct rgb_thread_arg *arg = data;
	struct rgb_camera *cam = arg->cam;
	struct rgb_state *state = arg->state;
	struct timespec fps_t0;
	unsigned int fps_frames = 0;
	unsigned int total_frames = 0;

	clock_gettime(CLOCK_MONOTONIC, &fps_t0);

	while (g_running) {
		fd_set fds;
		struct timeval tv;
		unsigned int index = 0;
		int ret;

		FD_ZERO(&fds);
		FD_SET(cam->fd, &fds);
		tv.tv_sec = 1;
		tv.tv_usec = 0;

		ret = select(cam->fd + 1, &fds, NULL, NULL, &tv);
		if (ret < 0) {
			if (errno == EINTR)
				continue;
			perror("rgb select");
			pthread_mutex_lock(&state->lock);
			state->error = true;
			pthread_mutex_unlock(&state->lock);
			break;
		}
		if (ret == 0)
			continue;

		ret = rgb_camera_dequeue(cam, &index);
		if (ret < 0) {
			pthread_mutex_lock(&state->lock);
			state->error = true;
			pthread_mutex_unlock(&state->lock);
			break;
		}
		if (ret > 0)
			continue;

		pthread_mutex_lock(&state->lock);
		memcpy(state->frame, cam->buffers[index].start, state->frame_size);
		total_frames++;
		fps_frames++;
		state->frames = total_frames;
		state->have_frame = true;

		if (fps_frames >= arg->cfg->cam_fps) {
			struct timespec now;
			double elapsed;

			clock_gettime(CLOCK_MONOTONIC, &now);
			elapsed = elapsed_sec(&now, &fps_t0);
			if (elapsed > 0.0)
				state->fps = fps_frames / elapsed;
			fps_t0 = now;
			fps_frames = 0;
		}
		pthread_mutex_unlock(&state->lock);

		if (rgb_camera_queue(cam, index) < 0) {
			pthread_mutex_lock(&state->lock);
			state->error = true;
			pthread_mutex_unlock(&state->lock);
			break;
		}
	}

	return NULL;
}

static void *thermal_capture_thread(void *data)
{
	struct thermal_thread_arg *arg = data;
	struct thermal_state *state = arg->state;
	struct timespec fps_t0;
	unsigned int fps_frames = 0;
	unsigned int total_frames = 0;
	s32 *local;

	local = malloc(MLX90640_FRAME_BYTES);
	if (!local) {
		pthread_mutex_lock(&state->lock);
		state->error = true;
		pthread_mutex_unlock(&state->lock);
		return NULL;
	}

	clock_gettime(CLOCK_MONOTONIC, &fps_t0);

	while (g_running) {
		s32 raw_min;
		s32 raw_max;
		s32 temp_center;
		int i;

		if (sensor_read_frame(arg->sensor, local) < 0) {
			fprintf(stderr, "Thermal frame read error\n");
			usleep(100000);
			continue;
		}

		raw_min = local[0];
		raw_max = local[0];
		for (i = 1; i < MLX90640_PIXELS; i++) {
			if (local[i] < raw_min)
				raw_min = local[i];
			if (local[i] > raw_max)
				raw_max = local[i];
		}
		temp_center = local[12 * MLX90640_COLS + 16];

		pthread_mutex_lock(&state->lock);
		memcpy(state->frame, local, MLX90640_FRAME_BYTES);
		total_frames++;
		fps_frames++;
		state->frames = total_frames;
		state->raw_min = raw_min;
		state->raw_max = raw_max;
		state->temp_center = temp_center;
		state->have_frame = true;

		if (fps_frames >= arg->cfg->thermal_fps) {
			struct timespec now;
			double elapsed;

			clock_gettime(CLOCK_MONOTONIC, &now);
			elapsed = elapsed_sec(&now, &fps_t0);
			if (elapsed > 0.0)
				state->fps = fps_frames / elapsed;
			fps_t0 = now;
			fps_frames = 0;
		}
		pthread_mutex_unlock(&state->lock);
	}

	free(local);
	return NULL;
}

static void dual_usage(const char *prog)
{
	printf("Dual camera split-screen demo\n");
	printf("Usage: %s [options]\n", prog);
	printf("Options:\n");
	printf("  -v <dev>     RGB V4L2 device (default: %s)\n", DUAL_DEFAULT_VIDEO_DEV);
	printf("  -d <fbdev>   Display fbdev fallback (default: %s)\n", DUAL_DEFAULT_FB_DEV);
	printf("  -i <path>    MLX90640 IIO path (default: auto/direct I2C)\n");
	printf("  -W <width>   RGB capture width (default: %u)\n", DUAL_DEFAULT_CAM_W);
	printf("  -H <height>  RGB capture height (default: %u)\n", DUAL_DEFAULT_CAM_H);
	printf("  -r <fps>     RGB FPS request (default: %u)\n", DUAL_DEFAULT_CAM_FPS);
	printf("  -t <fps>     Thermal refresh rate (default: %u)\n", DUAL_DEFAULT_THERM_FPS);
	printf("  -f <fps>     Display refresh rate (default: %u)\n", DUAL_DEFAULT_DISPLAY_FPS);
	printf("  -R <deg>     RGB rotation 0/90/180/270 (default: 90)\n");
	printf("  -T <deg>     Thermal rotation 0/90/180/270 (default: 90)\n");
	printf("  -n <count>   Stop after count display frames (default: 0/infinite)\n");
	printf("  -s           Stretch panels instead of keeping aspect ratio\n");
	printf("  -h           Show this help\n");
}

static bool valid_rotation(int rotation)
{
	return rotation == 0 || rotation == 90 ||
	       rotation == 180 || rotation == 270;
}

int main(int argc, char **argv)
{
	struct dual_config cfg = {
		.video_dev = DUAL_DEFAULT_VIDEO_DEV,
		.fbdev = DUAL_DEFAULT_FB_DEV,
		.iiopath = NULL,
		.cam_w = DUAL_DEFAULT_CAM_W,
		.cam_h = DUAL_DEFAULT_CAM_H,
		.cam_fps = DUAL_DEFAULT_CAM_FPS,
		.thermal_fps = DUAL_DEFAULT_THERM_FPS,
		.display_fps = DUAL_DEFAULT_DISPLAY_FPS,
		.frame_limit = 0,
		.cam_rotation = 90,
		.thermal_rotation = 90,
		.keep_aspect = true,
	};
	struct rgb_camera rgb_cam;
	struct sensor_ctx thermal_sensor;
	struct fb_info fb;
	struct rect left_panel;
	struct rect right_panel;
	struct rgb_state rgb_state;
	struct thermal_state thermal_state;
	struct rgb_thread_arg rgb_arg;
	struct thermal_thread_arg thermal_arg;
	pthread_t rgb_thread;
	pthread_t thermal_thread;
	uint8_t *rgb_snapshot = NULL;
	s32 *thermal_snapshot = NULL;
	bool rgb_thread_started = false;
	bool thermal_thread_started = false;
	unsigned int display_frames = 0;
	int opt;
	int ret = 1;

	memset(&rgb_state, 0, sizeof(rgb_state));
	memset(&thermal_state, 0, sizeof(thermal_state));
	memset(&rgb_cam, 0, sizeof(rgb_cam));
	memset(&thermal_sensor, 0, sizeof(thermal_sensor));
	thermal_sensor.fd = -1;
	rgb_cam.fd = -1;

	while ((opt = getopt(argc, argv, "v:d:i:W:H:r:t:f:R:T:n:sh")) != -1) {
		switch (opt) {
		case 'v':
			cfg.video_dev = optarg;
			break;
		case 'd':
			cfg.fbdev = optarg;
			break;
		case 'i':
			cfg.iiopath = optarg;
			break;
		case 'W':
			cfg.cam_w = (unsigned int)atoi(optarg);
			break;
		case 'H':
			cfg.cam_h = (unsigned int)atoi(optarg);
			break;
		case 'r':
			cfg.cam_fps = (unsigned int)atoi(optarg);
			if (cfg.cam_fps == 0)
				cfg.cam_fps = DUAL_DEFAULT_CAM_FPS;
			break;
		case 't':
			cfg.thermal_fps = (unsigned int)atoi(optarg);
			if (cfg.thermal_fps == 0)
				cfg.thermal_fps = DUAL_DEFAULT_THERM_FPS;
			break;
		case 'f':
			cfg.display_fps = (unsigned int)atoi(optarg);
			if (cfg.display_fps == 0)
				cfg.display_fps = DUAL_DEFAULT_DISPLAY_FPS;
			break;
		case 'R':
			cfg.cam_rotation = atoi(optarg);
			break;
		case 'T':
			cfg.thermal_rotation = atoi(optarg);
			break;
		case 'n':
			cfg.frame_limit = (unsigned int)atoi(optarg);
			break;
		case 's':
			cfg.keep_aspect = false;
			break;
		case 'h':
			dual_usage(argv[0]);
			return 0;
		default:
			dual_usage(argv[0]);
			return 1;
		}
	}

	if (cfg.cam_w == 0 || cfg.cam_h == 0 ||
	    !valid_rotation(cfg.cam_rotation) ||
	    !valid_rotation(cfg.thermal_rotation)) {
		fprintf(stderr, "Invalid capture size or rotation\n");
		return 1;
	}

	printf("=== Dual Camera Split ===\n");
	printf("RGB=%s %ux%u@%u rot=%d | THERM=%u fps rot=%d | DISPLAY=%u fps\n",
	       cfg.video_dev, cfg.cam_w, cfg.cam_h, cfg.cam_fps,
	       cfg.cam_rotation, cfg.thermal_fps, cfg.thermal_rotation,
	       cfg.display_fps);

	colormap_init();

	if (pthread_mutex_init(&rgb_state.lock, NULL) != 0 ||
	    pthread_mutex_init(&thermal_state.lock, NULL) != 0) {
		fprintf(stderr, "mutex init failed\n");
		return 1;
	}

	if (sensor_open(&thermal_sensor, cfg.iiopath, cfg.thermal_fps) < 0)
		goto out_mutex;

	if (rgb_camera_open(&rgb_cam, &cfg) < 0)
		goto out_sensor;

	rgb_state.frame_size = (size_t)rgb_cam.stride * rgb_cam.height * 3 / 2;
	rgb_state.frame = malloc(rgb_state.frame_size);
	rgb_snapshot = malloc(rgb_state.frame_size);
	thermal_state.frame = malloc(MLX90640_FRAME_BYTES);
	thermal_snapshot = malloc(MLX90640_FRAME_BYTES);
	if (!rgb_state.frame || !rgb_snapshot ||
	    !thermal_state.frame || !thermal_snapshot) {
		fprintf(stderr, "frame buffer allocation failed\n");
		goto out_buffers;
	}

	if (fb_open(&fb, cfg.fbdev) < 0) {
		fprintf(stderr, "Failed to open display\n");
		goto out_buffers;
	}

	left_panel.x = 0;
	left_panel.y = 0;
	left_panel.w = fb.vinfo.xres / 2 - DIVIDER_W / 2;
	left_panel.h = fb.vinfo.yres;
	right_panel.x = left_panel.w + DIVIDER_W;
	right_panel.y = 0;
	right_panel.w = fb.vinfo.xres - right_panel.x;
	right_panel.h = fb.vinfo.yres;

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	draw_static_layout(&fb, left_panel, right_panel);

	rgb_arg.cam = &rgb_cam;
	rgb_arg.state = &rgb_state;
	rgb_arg.cfg = &cfg;
	thermal_arg.sensor = &thermal_sensor;
	thermal_arg.state = &thermal_state;
	thermal_arg.cfg = &cfg;

	if (pthread_create(&rgb_thread, NULL, rgb_capture_thread, &rgb_arg) != 0) {
		perror("pthread_create rgb");
		goto out_display;
	}
	rgb_thread_started = true;

	if (pthread_create(&thermal_thread, NULL, thermal_capture_thread,
			   &thermal_arg) != 0) {
		perror("pthread_create thermal");
		goto out_display;
	}
	thermal_thread_started = true;

	while (g_running) {
		bool have_rgb;
		bool have_thermal;
		bool rgb_error;
		bool thermal_error;
		unsigned int rgb_frames;
		unsigned int thermal_frames;
		double rgb_fps;
		double thermal_fps;
		s32 raw_min;
		s32 raw_max;
		s32 temp_center;

		if (cfg.frame_limit && display_frames >= cfg.frame_limit)
			break;

		pthread_mutex_lock(&rgb_state.lock);
		have_rgb = rgb_state.have_frame;
		rgb_error = rgb_state.error;
		rgb_frames = rgb_state.frames;
		rgb_fps = rgb_state.fps;
		if (have_rgb)
			memcpy(rgb_snapshot, rgb_state.frame, rgb_state.frame_size);
		pthread_mutex_unlock(&rgb_state.lock);

		pthread_mutex_lock(&thermal_state.lock);
		have_thermal = thermal_state.have_frame;
		thermal_error = thermal_state.error;
		thermal_frames = thermal_state.frames;
		thermal_fps = thermal_state.fps;
		raw_min = thermal_state.raw_min;
		raw_max = thermal_state.raw_max;
		temp_center = thermal_state.temp_center;
		if (have_thermal)
			memcpy(thermal_snapshot, thermal_state.frame,
			       MLX90640_FRAME_BYTES);
		pthread_mutex_unlock(&thermal_state.lock);

		if (rgb_error || thermal_error) {
			fprintf(stderr, "Capture thread error\n");
			goto out_display;
		}

		if (have_rgb) {
			draw_rgb_panel(&fb, &rgb_cam, rgb_snapshot, left_panel,
				       &cfg, rgb_frames, rgb_fps);
		}

		if (have_thermal) {
			draw_thermal_panel(&fb, thermal_snapshot, right_panel,
					   &cfg, raw_min, raw_max, temp_center,
					   thermal_frames, thermal_fps);
		}

		display_frames++;
		usleep(1000000 / cfg.display_fps);
	}

	ret = 0;

out_display:
	g_running = 0;
	if (rgb_thread_started)
		pthread_join(rgb_thread, NULL);
	if (thermal_thread_started)
		pthread_join(thermal_thread, NULL);
	fb_fill_rect(&fb, 0, 0, fb.vinfo.xres, fb.vinfo.yres,
		     rgb_to_pixel(&fb, 0, 0, 0));
	fb_close(&fb);
out_buffers:
	free(thermal_snapshot);
	free(thermal_state.frame);
	free(rgb_snapshot);
	free(rgb_state.frame);
	rgb_camera_close(&rgb_cam);
out_sensor:
	if (thermal_sensor.fd >= 0)
		close(thermal_sensor.fd);
out_mutex:
	pthread_mutex_destroy(&thermal_state.lock);
	pthread_mutex_destroy(&rgb_state.lock);
	printf("Exit: %s\n", ret == 0 ? "ok" : "error");
	return ret == 0 ? 0 : 1;
}
