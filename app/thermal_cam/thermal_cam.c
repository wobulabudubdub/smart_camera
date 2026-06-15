/*
 * thermal_cam.c - MLX90640 Thermal Camera Display Application
 *
 * Reads MLX90640 32x24 thermal sensor data and displays a color-mapped
 * thermal image on the Linux framebuffer.
 *
 * Target: Alientek AtomPi-CA1 (RK3568) with MD0550M 5.5" 1080x1920 MIPI LCD
 *
 * Features:
 *   - Ironbow color-mapped thermal image with bilinear upscaling
 *   - Min/Max/Center temperature markers
 *   - Temperature color scale bar
 *   - FPS counter
 *   - Auto-detect framebuffer and IIO device
 *
 * Usage:
 *   thermal_cam [options]
 *     -d <fbdev>    Framebuffer device (default: /dev/fb0)
 *     -i <iiodev>   IIO device path (default: auto-detect)
 *     -r <fps>      Target refresh rate (default: 4)
 *     -h            Show help
 *
 * Build (on target / cross-compile):
 *   aarch64-linux-gcc -O2 -o thermal_cam thermal_cam.c -lm
 *
 * License: GPL v2
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <signal.h>
#include <time.h>
#include <math.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <ctype.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <dirent.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include <drm_mode.h>

/* ------------------------------------------------------------------ */
/* Type definitions (kernel-style, for clarity with sensor data)      */
/* ------------------------------------------------------------------ */
typedef int32_t  s32;
typedef int64_t  s64;

/* ------------------------------------------------------------------ */
/* MLX90640 sensor constants                                          */
/* ------------------------------------------------------------------ */
#define MLX90640_COLS      32
#define MLX90640_ROWS      24
#define MLX90640_PIXELS    768
#define MLX90640_FRAME_BYTES  (MLX90640_PIXELS * sizeof(int32_t))

#define MLX90640_EE_WORDS       832
#define MLX90640_FRAME_WORDS    834
#define MLX90640_AUX_WORDS      64
#define MLX90640_DEFAULT_I2C_BUS  1
#define MLX90640_DEFAULT_I2C_ADDR 0x33

#define MLX90640_EEPROM_START   0x2400
#define MLX90640_PIXEL_START    0x0400
#define MLX90640_AUX_START      0x0700
#define MLX90640_STATUS_REG     0x8000
#define MLX90640_CTRL_REG       0x800D
#define MLX90640_STATUS_CLEAR   0x0030
#define MLX90640_DATA_READY     0x0008
#define MLX90640_FRAME_MASK     0x0001

#define MLX90640_CTRL_MEAS_MODE_MASK 0x1000
#define MLX90640_CTRL_RES_MASK       0x0C00
#define MLX90640_CTRL_RES_SHIFT      10
#define MLX90640_CTRL_REFRESH_MASK   0x0380
#define MLX90640_CTRL_REFRESH_SHIFT  7

#define MLX_MS_BYTE(x)          (((x) >> 8) & 0xff)
#define MLX_LS_BYTE(x)          ((x) & 0xff)
#define MLX_NIBBLE1(x)          ((x) & 0x000f)
#define MLX_NIBBLE2(x)          (((x) >> 4) & 0x000f)
#define MLX_NIBBLE3(x)          (((x) >> 8) & 0x000f)
#define MLX_NIBBLE4(x)          (((x) >> 12) & 0x000f)
#define MLX_MSBITS_6(x)         (((x) >> 10) & 0x003f)
#define MLX_LSBITS_10(x)        ((x) & 0x03ff)
#define MLX_SCALE_ALPHA         0.000001f

/* ------------------------------------------------------------------ */
/* Color map constants                                                */
/* ------------------------------------------------------------------ */
#define CMAP_SIZE          256
#define FONT_SCALE         2
#define FONT_WIDTH         5
#define FONT_HEIGHT        7
#define FONT_SPACING       1
#define TEXT_ADVANCE       ((FONT_WIDTH + FONT_SPACING) * FONT_SCALE)
#define TEXT_LINE_HEIGHT   ((FONT_HEIGHT + 5) * FONT_SCALE)

/*
 * Debug display mode:
 *   1 = draw raw 32x24 MLX90640 pixels as enlarged color blocks
 *   0 = draw the previous bilinear-interpolated image
 */
#define RAW_BLOCK_RENDER   0

/* ------------------------------------------------------------------ */
/* Framebuffer helpers                                                */
/* ------------------------------------------------------------------ */

struct fb_info {
	int fd;
	struct fb_var_screeninfo vinfo;
	struct fb_fix_screeninfo finfo;
	void *map;
	size_t size;
	int bpp;
	bool is_drm;
	uint32_t drm_conn_id;
	uint32_t drm_crtc_id;
	uint32_t drm_fb_id;
	uint32_t drm_handle;
	drmModeCrtcPtr drm_saved_crtc;
};

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

static int drm_open_display(struct fb_info *fb)
{
	const char *card = "/dev/dri/card0";
	drmModeResPtr res = NULL;
	drmModeConnectorPtr conn = NULL;
	drmModeModeInfo mode;
	struct drm_mode_create_dumb creq;
	struct drm_mode_map_dumb mreq;
	uint32_t handles[4] = {0};
	uint32_t pitches[4] = {0};
	uint32_t offsets[4] = {0};
	int fd = -1;
	int i, ret;

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
	if (drm_find_crtc(fd, res, conn, &fb->drm_crtc_id) < 0)
		goto err_conn;

	memset(&creq, 0, sizeof(creq));
	creq.width = mode.hdisplay;
	creq.height = mode.vdisplay;
	creq.bpp = 32;

	if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) < 0)
		goto err_conn;

	fb->drm_handle = creq.handle;
	handles[0] = creq.handle;
	pitches[0] = creq.pitch;

	ret = drmModeAddFB2(fd, creq.width, creq.height, DRM_FORMAT_XRGB8888,
			    handles, pitches, offsets, &fb->drm_fb_id, 0);
	if (ret < 0) {
		ret = drmModeAddFB(fd, creq.width, creq.height, 24, 32,
				   creq.pitch, creq.handle, &fb->drm_fb_id);
		if (ret < 0)
			goto err_destroy;
	}

	memset(&mreq, 0, sizeof(mreq));
	mreq.handle = creq.handle;
	if (ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) < 0)
		goto err_rmfb;

	fb->map = mmap(NULL, creq.size, PROT_READ | PROT_WRITE,
		       MAP_SHARED, fd, mreq.offset);
	if (fb->map == MAP_FAILED)
		goto err_rmfb;

	fb->drm_conn_id = conn->connector_id;
	fb->drm_saved_crtc = drmModeGetCrtc(fd, fb->drm_crtc_id);

	if (drmModeSetCrtc(fd, fb->drm_crtc_id, fb->drm_fb_id, 0, 0,
			   &fb->drm_conn_id, 1, &mode) < 0)
		goto err_munmap;

	memset(&fb->vinfo, 0, sizeof(fb->vinfo));
	fb->vinfo.xres = creq.width;
	fb->vinfo.yres = creq.height;
	fb->vinfo.xres_virtual = creq.pitch / 4;
	fb->vinfo.yres_virtual = creq.height;
	fb->vinfo.bits_per_pixel = 32;
	fb->vinfo.red.offset = 16;
	fb->vinfo.red.length = 8;
	fb->vinfo.green.offset = 8;
	fb->vinfo.green.length = 8;
	fb->vinfo.blue.offset = 0;
	fb->vinfo.blue.length = 8;

	fb->fd = fd;
	fb->size = creq.size;
	fb->bpp = 32;
	fb->is_drm = true;

	printf("DRM/KMS: %s connector=%u crtc=%u fb=%u\n",
	       card, fb->drm_conn_id, fb->drm_crtc_id, fb->drm_fb_id);
	printf("  Resolution: %dx%d  Pitch: %u bytes\n",
	       fb->vinfo.xres, fb->vinfo.yres, creq.pitch);

	drmModeFreeConnector(conn);
	drmModeFreeResources(res);
	return 0;

err_munmap:
	munmap(fb->map, creq.size);
	if (fb->drm_saved_crtc)
		drmModeFreeCrtc(fb->drm_saved_crtc);
err_rmfb:
	if (fb->drm_fb_id)
		drmModeRmFB(fd, fb->drm_fb_id);
err_destroy:
	{
		struct drm_mode_destroy_dumb dreq;
		memset(&dreq, 0, sizeof(dreq));
		dreq.handle = creq.handle;
		ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
	}
err_conn:
	drmModeFreeConnector(conn);
err_res:
	drmModeFreeResources(res);
err_close:
	close(fd);
	return -1;
}

static int fbdev_open(struct fb_info *fb, const char *dev)
{
	fb->fd = open(dev, O_RDWR);
	if (fb->fd < 0) {
		perror("open framebuffer");
		return -1;
	}

	if (ioctl(fb->fd, FBIOGET_VSCREENINFO, &fb->vinfo) < 0) {
		perror("FBIOGET_VSCREENINFO");
		close(fb->fd);
		return -1;
	}

	if (ioctl(fb->fd, FBIOGET_FSCREENINFO, &fb->finfo) < 0) {
		perror("FBIOGET_FSCREENINFO");
		close(fb->fd);
		return -1;
	}

	fb->bpp  = fb->vinfo.bits_per_pixel;
	fb->size = fb->finfo.smem_len;

	fb->map = mmap(NULL, fb->size, PROT_READ | PROT_WRITE,
		       MAP_SHARED, fb->fd, 0);
	if (fb->map == MAP_FAILED) {
		perror("mmap framebuffer");
		close(fb->fd);
		return -1;
	}

	printf("Framebuffer: %s\n", dev);
	printf("  Resolution: %dx%d\n", fb->vinfo.xres, fb->vinfo.yres);
	printf("  Virtual:    %dx%d\n", fb->vinfo.xres_virtual, fb->vinfo.yres_virtual);
	printf("  BPP:        %d\n", fb->bpp);
	printf("  R/G/B offset: %d/%d/%d  length: %d/%d/%d\n",
	       fb->vinfo.red.offset, fb->vinfo.green.offset, fb->vinfo.blue.offset,
	       fb->vinfo.red.length, fb->vinfo.green.length, fb->vinfo.blue.length);

	return 0;
}

static int fb_open(struct fb_info *fb, const char *dev)
{
	memset(fb, 0, sizeof(*fb));
	fb->fd = -1;
	fb->map = MAP_FAILED;

	if (drm_open_display(fb) == 0)
		return 0;

	fprintf(stderr, "DRM/KMS open failed, falling back to %s\n", dev);
	return fbdev_open(fb, dev);
}

static void fb_close(struct fb_info *fb)
{
	if (fb->is_drm && fb->fd >= 0) {
		struct drm_mode_destroy_dumb dreq;

		if (fb->drm_saved_crtc) {
			drmModeSetCrtc(fb->fd, fb->drm_saved_crtc->crtc_id,
				       fb->drm_saved_crtc->buffer_id,
				       fb->drm_saved_crtc->x,
				       fb->drm_saved_crtc->y,
				       &fb->drm_conn_id, 1,
				       &fb->drm_saved_crtc->mode);
			drmModeFreeCrtc(fb->drm_saved_crtc);
		}
		if (fb->map && fb->map != MAP_FAILED)
			munmap(fb->map, fb->size);
		if (fb->drm_fb_id)
			drmModeRmFB(fb->fd, fb->drm_fb_id);
		if (fb->drm_handle) {
			memset(&dreq, 0, sizeof(dreq));
			dreq.handle = fb->drm_handle;
			ioctl(fb->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
		}
		close(fb->fd);
		return;
	}

	if (fb->map && fb->map != MAP_FAILED)
		munmap(fb->map, fb->size);
	if (fb->fd >= 0)
		close(fb->fd);
}

/*
 * Convert 24-bit RGB (0-255 each) to framebuffer pixel format.
 * Supports RGB565 and XRGB8888.
 */
static inline uint32_t rgb_to_pixel(const struct fb_info *fb,
				     uint8_t r, uint8_t g, uint8_t b)
{
	if (fb->bpp == 16) {
		/* RGB565 */
		uint16_t r5 = (r >> 3) & 0x1f;
		uint16_t g6 = (g >> 2) & 0x3f;
		uint16_t b5 = (b >> 3) & 0x1f;
		return (r5 << 11) | (g6 << 5) | b5;
	} else {
		/* XRGB8888 (or BGR888 depending on offset) */
		uint8_t r_len = fb->vinfo.red.length;
		uint8_t g_len = fb->vinfo.green.length;
		uint8_t b_len = fb->vinfo.blue.length;
		uint32_t r_val = (r >> (8 - r_len)) & ((1 << r_len) - 1);
		uint32_t g_val = (g >> (8 - g_len)) & ((1 << g_len) - 1);
		uint32_t b_val = (b >> (8 - b_len)) & ((1 << b_len) - 1);
		return (r_val << fb->vinfo.red.offset) |
		       (g_val << fb->vinfo.green.offset) |
		       (b_val << fb->vinfo.blue.offset);
	}
}

static inline void fb_put_pixel(const struct fb_info *fb,
				 int x, int y, uint32_t pixel)
{
	uint32_t *fb32;
	uint16_t *fb16;
	off_t offset;

	if (x < 0 || x >= (int)fb->vinfo.xres ||
	    y < 0 || y >= (int)fb->vinfo.yres)
		return;

	if (fb->bpp == 16) {
		fb16 = (uint16_t *)fb->map;
		offset = y * fb->vinfo.xres_virtual + x;
		fb16[offset] = (uint16_t)pixel;
	} else {
		fb32 = (uint32_t *)fb->map;
		offset = y * fb->vinfo.xres_virtual + x;
		fb32[offset] = pixel;
	}
}

static void fb_fill_rect(const struct fb_info *fb,
			  int x, int y, int w, int h, uint32_t color)
{
	int row, col;
	uint32_t *fb32;
	uint16_t *fb16;
	uint32_t line_virt = fb->vinfo.xres_virtual;

	if (fb->bpp == 16) {
		fb16 = (uint16_t *)fb->map;
		for (row = y; row < y + h && row < (int)fb->vinfo.yres; row++) {
			for (col = x; col < x + w && col < (int)fb->vinfo.xres; col++) {
				fb16[row * line_virt + col] = (uint16_t)color;
			}
		}
	} else {
		fb32 = (uint32_t *)fb->map;
		for (row = y; row < y + h && row < (int)fb->vinfo.yres; row++) {
			for (col = x; col < x + w && col < (int)fb->vinfo.xres; col++) {
				fb32[row * line_virt + col] = color;
			}
		}
	}
}

/*
 * Simple bitmap font for overlay text (5x7 pixels per char, ASCII 32-90)
 * We use a minimal set to draw temperature labels.
 */
static const uint8_t font5x7[][5] = {
	[0]  = {0x00,0x00,0x00,0x00,0x00}, /* space */
	[1]  = {0x00,0x00,0x5f,0x00,0x00}, /* ! */
	[2]  = {0x00,0x07,0x00,0x07,0x00}, /* " */
	[3]  = {0x14,0x7f,0x14,0x7f,0x14}, /* # */
	[4]  = {0x24,0x2a,0x7f,0x2a,0x12}, /* $ */
	[5]  = {0x23,0x13,0x08,0x64,0x62}, /* % */
	[6]  = {0x36,0x49,0x55,0x22,0x50}, /* & */
	[7]  = {0x00,0x05,0x03,0x00,0x00}, /* ' */
	[8]  = {0x00,0x1c,0x22,0x41,0x00}, /* ( */
	[9]  = {0x00,0x41,0x22,0x1c,0x00}, /* ) */
	[10] = {0x14,0x08,0x3e,0x08,0x14}, /* * */
	[11] = {0x08,0x08,0x3e,0x08,0x08}, /* + */
	[12] = {0x00,0x50,0x30,0x00,0x00}, /* , */
	[13] = {0x08,0x08,0x08,0x08,0x08}, /* - */
	[14] = {0x00,0x60,0x60,0x00,0x00}, /* . */
	[15] = {0x20,0x10,0x08,0x04,0x02}, /* / */
	/* 0-9 */
	[16] = {0x3e,0x51,0x49,0x45,0x3e},
	[17] = {0x00,0x42,0x7f,0x40,0x00},
	[18] = {0x42,0x61,0x51,0x49,0x46},
	[19] = {0x21,0x41,0x45,0x4b,0x31},
	[20] = {0x18,0x14,0x12,0x7f,0x10},
	[21] = {0x27,0x45,0x45,0x45,0x39},
	[22] = {0x3c,0x4a,0x49,0x49,0x30},
	[23] = {0x01,0x71,0x09,0x05,0x03},
	[24] = {0x36,0x49,0x49,0x49,0x36},
	[25] = {0x06,0x49,0x49,0x29,0x1e},
	/* : ; < = > ? @ */
	[26] = {0x00,0x36,0x36,0x00,0x00}, /* : */
	[27] = {0x00,0x56,0x36,0x00,0x00}, /* ; */
	[28] = {0x08,0x14,0x22,0x41,0x00}, /* < */
	[29] = {0x14,0x14,0x14,0x14,0x14}, /* = */
	[30] = {0x00,0x41,0x22,0x14,0x08}, /* > */
	[31] = {0x02,0x01,0x51,0x09,0x06}, /* ? */
	[32] = {0x32,0x49,0x79,0x41,0x3e}, /* @ */
	/* A-Z */
	[33] = {0x7e,0x11,0x11,0x11,0x7e}, /* A */
	[34] = {0x7f,0x49,0x49,0x49,0x36}, /* B */
	[35] = {0x3e,0x41,0x41,0x41,0x22}, /* C */
	[36] = {0x7f,0x41,0x41,0x22,0x1c}, /* D */
	[37] = {0x7f,0x49,0x49,0x49,0x41}, /* E */
	[38] = {0x7f,0x09,0x09,0x09,0x01}, /* F */
	[39] = {0x3e,0x41,0x49,0x49,0x7a}, /* G */
	[40] = {0x7f,0x08,0x08,0x08,0x7f}, /* H */
	[41] = {0x00,0x41,0x7f,0x41,0x00}, /* I */
	[42] = {0x20,0x40,0x41,0x3f,0x01}, /* J */
	[43] = {0x7f,0x08,0x14,0x22,0x41}, /* K */
	[44] = {0x7f,0x40,0x40,0x40,0x40}, /* L */
	[45] = {0x7f,0x02,0x0c,0x02,0x7f}, /* M */
	[46] = {0x7f,0x04,0x08,0x10,0x7f}, /* N */
	[47] = {0x3e,0x41,0x41,0x41,0x3e}, /* O */
	[48] = {0x7f,0x09,0x09,0x09,0x06}, /* P */
	[49] = {0x3e,0x41,0x51,0x21,0x5e}, /* Q */
	[50] = {0x7f,0x09,0x19,0x29,0x46}, /* R */
	[51] = {0x46,0x49,0x49,0x49,0x31}, /* S */
	[52] = {0x01,0x01,0x7f,0x01,0x01}, /* T */
	[53] = {0x3f,0x40,0x40,0x40,0x3f}, /* U */
	[54] = {0x1f,0x20,0x40,0x20,0x1f}, /* V */
	[55] = {0x3f,0x40,0x38,0x40,0x3f}, /* W */
	[56] = {0x63,0x14,0x08,0x14,0x63}, /* X */
	[57] = {0x07,0x08,0x70,0x08,0x07}, /* Y */
	[58] = {0x61,0x51,0x49,0x45,0x43}, /* Z */
};

/* Map ASCII to font index */
static int font_index(char c)
{
	if (c == ' ')
		return 0;
	if (c >= '0' && c <= '9')
		return 16 + (c - '0');
	if (c >= 'A' && c <= 'Z')
		return 33 + (c - 'A');
	if (c >= 'a' && c <= 'z')
		return 33 + (c - 'a');
	if (c == '.') return 14;
	if (c == '-') return 13;
	if (c == '+') return 11;
	if (c == ':') return 26;
	if (c == '!') return 1;
	if (c == '%') return 5;
	if (c == 'C') return 35;
	if (c == 'F') return 38;
	if (c == 'o') return 47;
	return 0; /* unknown -> space */
}

static void fb_draw_char(const struct fb_info *fb, int x, int y,
			  uint32_t color, char c)
{
	int idx = font_index(c);
	int row, col, dy, dx;

	for (row = 0; row < FONT_HEIGHT; row++) {
		for (col = 0; col < FONT_WIDTH; col++) {
			if (!(font5x7[idx][col] & (1 << row)))
				continue;

			for (dy = 0; dy < FONT_SCALE; dy++) {
				for (dx = 0; dx < FONT_SCALE; dx++) {
					fb_put_pixel(fb,
						     x + col * FONT_SCALE + dx,
						     y + row * FONT_SCALE + dy,
						     color);
				}
			}
		}
	}
}

static void fb_draw_text(const struct fb_info *fb, int x, int y,
			  uint32_t color, const char *text)
{
	while (*text) {
		fb_draw_char(fb, x, y, color, *text);
		x += TEXT_ADVANCE;
		text++;
	}
}

/* ------------------------------------------------------------------ */
/* Green-red thermal colormap (256 entries, RGB888)                   */
/* ------------------------------------------------------------------ */

struct rgb888 {
	uint8_t r, g, b;
};

static struct rgb888 colormap[CMAP_SIZE];

/*
 * Generate a green-red thermal colormap:
 *   0-63:   dark green -> green
 *   64-127: green -> yellow-green
 *   128-191: yellow -> orange
 *   192-255: orange -> red -> white
 */
static void colormap_init(void)
{
	int i;
	double t;

	for (i = 0; i < CMAP_SIZE; i++) {
		t = (double)i / (CMAP_SIZE - 1);

		if (t < 0.25) {
			/* dark green -> green */
			double s = t / 0.25;
			colormap[i].r = 0;
			colormap[i].g = (uint8_t)(45 + 160 * s);
			colormap[i].b = 0;
		} else if (t < 0.45) {
			/* green -> yellow-green */
			double s = (t - 0.25) / 0.20;
			colormap[i].r = (uint8_t)(180 * s);
			colormap[i].g = (uint8_t)(205 + 50 * s);
			colormap[i].b = 0;
		} else if (t < 0.85) {
			/* yellow -> orange -> red */
			double s = (t - 0.45) / 0.40;
			colormap[i].r = (uint8_t)(180 + 75 * s);
			colormap[i].g = (uint8_t)(255 * (1.0 - s));
			colormap[i].b = 0;
		} else {
			/* red -> white */
			double s = (t - 0.85) / 0.15;
			colormap[i].r = 255;
			colormap[i].g = (uint8_t)(255 * s);
			colormap[i].b = (uint8_t)(255 * s);
		}
	}
}

/*
 * Map a temperature (millidegree C) to a colormap index [0,255].
 * temp_min -> index 0 (dark green)
 * temp_max -> index 255 (white-hot)
 */
static inline uint8_t temp_to_cmap(s32 temp, s32 temp_min, s32 temp_max)
{
	double t;

	if (temp_max <= temp_min)
		return 127;

	t = (double)(temp - temp_min) / (double)(temp_max - temp_min);
	if (t < 0.0) t = 0.0;
	if (t > 1.0) t = 1.0;

	return (uint8_t)(t * (CMAP_SIZE - 1));
}

/* ------------------------------------------------------------------ */
/* MLX90640 official userspace temperature calculation                 */
/* ------------------------------------------------------------------ */

struct mlx90640_params {
	int16_t k_vdd;
	int16_t vdd25;
	float kv_ptat;
	float kt_ptat;
	uint16_t v_ptat25;
	float alpha_ptat;
	int16_t gain_ee;
	float tgc;
	float cp_kv;
	float cp_kta;
	uint8_t resolution_ee;
	uint8_t calibration_mode_ee;
	float ks_ta;
	float ks_to[5];
	int16_t ct[5];
	uint16_t alpha[MLX90640_PIXELS];
	uint8_t alpha_scale;
	int16_t offset[MLX90640_PIXELS];
	int8_t kta[MLX90640_PIXELS];
	uint8_t kta_scale;
	int8_t kv[MLX90640_PIXELS];
	uint8_t kv_scale;
	float cp_alpha[2];
	int16_t cp_offset[2];
	float il_chess_c[3];
	uint16_t broken_pixels[5];
	uint16_t outlier_pixels[5];
};

struct sensor_ctx {
	int fd;
	bool direct_i2c;
	int i2c_bus;
	int i2c_addr;
	char frame_path[PATH_MAX];
	struct mlx90640_params params;
	float last_to[MLX90640_PIXELS];
	bool have_last_to;
};

static float pow2f_i(int exp)
{
	return ldexpf(1.0f, exp);
}

static int mlx90640_i2c_read_words(int fd, uint16_t reg, uint16_t *buf, int len)
{
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[2];
	uint8_t reg_buf[2];
	int i;

	reg_buf[0] = reg >> 8;
	reg_buf[1] = reg & 0xff;

	messages[0].addr = MLX90640_DEFAULT_I2C_ADDR;
	messages[0].flags = 0;
	messages[0].len = sizeof(reg_buf);
	messages[0].buf = reg_buf;
	messages[1].addr = MLX90640_DEFAULT_I2C_ADDR;
	messages[1].flags = I2C_M_RD;
	messages[1].len = len * 2;
	messages[1].buf = (uint8_t *)buf;

	packets.msgs = messages;
	packets.nmsgs = 2;

	if (ioctl(fd, I2C_RDWR, &packets) < 0)
		return -1;

	for (i = 0; i < len; i++)
		buf[i] = (uint16_t)((buf[i] >> 8) | (buf[i] << 8));

	return 0;
}

static int mlx90640_i2c_write_word(int fd, uint16_t reg, uint16_t value)
{
	uint8_t buf[4];

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = value >> 8;
	buf[3] = value & 0xff;

	return write(fd, buf, sizeof(buf)) == (ssize_t)sizeof(buf) ? 0 : -1;
}

static uint8_t mlx90640_refresh_code_for_fps(int fps)
{
	if (fps <= 1)
		return 1;	/* 1 Hz */
	if (fps <= 2)
		return 2;	/* 2 Hz */
	if (fps <= 4)
		return 3;	/* 4 Hz */
	if (fps <= 8)
		return 4;	/* 8 Hz */
	if (fps <= 16)
		return 5;	/* 16 Hz */
	if (fps <= 32)
		return 6;	/* 32 Hz */
	return 7;		/* 64 Hz */
}

static int mlx90640_set_refresh_rate(int fd, int fps)
{
	uint16_t ctrl;
	uint8_t code = mlx90640_refresh_code_for_fps(fps);

	if (mlx90640_i2c_read_words(fd, MLX90640_CTRL_REG, &ctrl, 1) < 0)
		return -1;

	ctrl &= ~MLX90640_CTRL_REFRESH_MASK;
	ctrl |= (uint16_t)code << MLX90640_CTRL_REFRESH_SHIFT;

	return mlx90640_i2c_write_word(fd, MLX90640_CTRL_REG, ctrl);
}

static int mlx90640_wait_frame_ready(int fd)
{
	uint16_t status;
	int tries;

	for (tries = 0; tries < 200; tries++) {
		if (mlx90640_i2c_read_words(fd, MLX90640_STATUS_REG, &status, 1) < 0)
			return -1;
		if (status & MLX90640_DATA_READY)
			return 0;
		usleep(5000);
	}

	errno = ETIMEDOUT;
	return -1;
}

static int mlx90640_read_i2c_frame(int fd, uint16_t frame_data[MLX90640_FRAME_WORDS])
{
	uint16_t status;
	uint16_t aux[MLX90640_AUX_WORDS];
	uint16_t ctrl;
	int i;

	if (mlx90640_wait_frame_ready(fd) < 0)
		return -1;

	if (mlx90640_i2c_read_words(fd, MLX90640_STATUS_REG, &status, 1) < 0)
		return -1;
	if (mlx90640_i2c_write_word(fd, MLX90640_STATUS_REG, MLX90640_STATUS_CLEAR) < 0)
		return -1;
	if (mlx90640_i2c_read_words(fd, MLX90640_PIXEL_START, frame_data,
				    MLX90640_PIXELS) < 0)
		return -1;
	if (mlx90640_i2c_read_words(fd, MLX90640_AUX_START, aux, MLX90640_AUX_WORDS) < 0)
		return -1;
	if (mlx90640_i2c_read_words(fd, MLX90640_CTRL_REG, &ctrl, 1) < 0)
		return -1;

	for (i = 0; i < MLX90640_AUX_WORDS; i++)
		frame_data[MLX90640_PIXELS + i] = aux[i];

	frame_data[832] = ctrl;
	frame_data[833] = status & MLX90640_FRAME_MASK;

	return 0;
}

static int mlx90640_check_adjacent_pixels(uint16_t pix1, uint16_t pix2)
{
	int pix_pos_diff = pix1 > pix2 ? pix1 - pix2 : pix2 - pix1;

	return pix_pos_diff == 1 || pix_pos_diff == 32 ? -1 : 0;
}

static int mlx90640_is_pixel_bad(uint16_t pixel, const struct mlx90640_params *p)
{
	int i;

	for (i = 0; i < 5; i++) {
		if (p->broken_pixels[i] == pixel || p->outlier_pixels[i] == pixel)
			return 1;
	}

	return 0;
}

static float mlx90640_median4(float values[4])
{
	float tmp;
	int i, j;

	for (i = 0; i < 3; i++) {
		for (j = i + 1; j < 4; j++) {
			if (values[j] < values[i]) {
				tmp = values[i];
				values[i] = values[j];
				values[j] = tmp;
			}
		}
	}

	return (values[1] + values[2]) * 0.5f;
}

static void mlx90640_bad_pixels_correction(uint16_t *pixels, float *to,
					   int mode, const struct mlx90640_params *params)
{
	float ap[4];
	uint8_t pix = 0;
	uint8_t line;
	uint8_t column;
	uint16_t p;

	while (pix < 5 && pixels[pix] != 0xffff) {
		p = pixels[pix];
		line = p >> 5;
		column = p - (line << 5);

		if (mode == 1) {
			if (line == 0) {
				to[p] = column == 0 ? to[33] :
					(column == 31 ? to[62] : (to[p + 31] + to[p + 33]) * 0.5f);
			} else if (line == 23) {
				to[p] = column == 0 ? to[705] :
					(column == 31 ? to[734] : (to[p - 33] + to[p - 31]) * 0.5f);
			} else if (column == 0) {
				to[p] = (to[p - 31] + to[p + 33]) * 0.5f;
			} else if (column == 31) {
				to[p] = (to[p - 33] + to[p + 31]) * 0.5f;
			} else {
				ap[0] = to[p - 33];
				ap[1] = to[p - 31];
				ap[2] = to[p + 31];
				ap[3] = to[p + 33];
				to[p] = mlx90640_median4(ap);
			}
		} else {
			if (column == 0) {
				to[p] = to[p + 1];
			} else if (column == 1 || column == 30) {
				to[p] = (to[p - 1] + to[p + 1]) * 0.5f;
			} else if (column == 31) {
				to[p] = to[p - 1];
			} else if (!mlx90640_is_pixel_bad(p - 2, params) &&
				   !mlx90640_is_pixel_bad(p + 2, params)) {
				ap[0] = to[p + 1] - to[p + 2];
				ap[1] = to[p - 1] - to[p - 2];
				to[p] = fabsf(ap[0]) > fabsf(ap[1]) ?
					to[p - 1] + ap[1] : to[p + 1] + ap[0];
			} else {
				to[p] = (to[p - 1] + to[p + 1]) * 0.5f;
			}
		}

		pix++;
	}
}

static int mlx90640_extract_deviating_pixels(uint16_t *ee, struct mlx90640_params *p)
{
	uint16_t pix_cnt = 0;
	uint16_t broken_cnt = 0;
	uint16_t outlier_cnt = 0;
	int i;

	for (pix_cnt = 0; pix_cnt < 5; pix_cnt++) {
		p->broken_pixels[pix_cnt] = 0xffff;
		p->outlier_pixels[pix_cnt] = 0xffff;
	}

	for (pix_cnt = 0; pix_cnt < MLX90640_PIXELS && broken_cnt < 5 && outlier_cnt < 5;
	     pix_cnt++) {
		if (ee[pix_cnt + 64] == 0)
			p->broken_pixels[broken_cnt++] = pix_cnt;
		else if (ee[pix_cnt + 64] & 0x0001)
			p->outlier_pixels[outlier_cnt++] = pix_cnt;
	}

	if (broken_cnt > 4 || outlier_cnt > 4 || broken_cnt + outlier_cnt > 4)
		return -1;

	for (pix_cnt = 0; pix_cnt < broken_cnt; pix_cnt++)
		for (i = pix_cnt + 1; i < broken_cnt; i++)
			if (mlx90640_check_adjacent_pixels(p->broken_pixels[pix_cnt],
							   p->broken_pixels[i]) != 0)
				return -1;

	for (pix_cnt = 0; pix_cnt < outlier_cnt; pix_cnt++)
		for (i = pix_cnt + 1; i < outlier_cnt; i++)
			if (mlx90640_check_adjacent_pixels(p->outlier_pixels[pix_cnt],
							   p->outlier_pixels[i]) != 0)
				return -1;

	return 0;
}

static void mlx90640_extract_params(uint16_t *ee, struct mlx90640_params *p)
{
	int acc_row[MLX90640_ROWS], acc_col[MLX90640_COLS];
	int occ_row[MLX90640_ROWS], occ_col[MLX90640_COLS];
	float alpha_tmp[MLX90640_PIXELS], kta_tmp[MLX90640_PIXELS];
	float kv_tmp[MLX90640_PIXELS], tmp;
	int8_t kv_t[4], kta_rc[4];
	int i, j, n, pix, split;
	uint8_t scale, scale2;
	int alpha_ref, offset_ref;
	int alpha_scale, ks_to_scale;
	int step;

	memset(p, 0, sizeof(*p));

	p->k_vdd = 32 * (int8_t)MLX_MS_BYTE(ee[51]);
	p->vdd25 = (((int16_t)MLX_LS_BYTE(ee[51]) - 256) << 5) - 8192;

	tmp = (ee[50] & 0xfc00) >> 10;
	if (tmp > 31)
		tmp -= 64;
	p->kv_ptat = tmp / 4096.0f;
	tmp = MLX_LSBITS_10(ee[50]);
	if (tmp > 511)
		tmp -= 1024;
	p->kt_ptat = tmp / 8.0f;
	p->v_ptat25 = ee[49];
	p->alpha_ptat = (ee[16] & 0xf000) / pow2f_i(14) + 8.0f;
	p->gain_ee = (int16_t)ee[48];
	p->tgc = (int8_t)MLX_LS_BYTE(ee[60]) / 32.0f;
	p->resolution_ee = (ee[56] & 0x3000) >> 12;
	p->ks_ta = (int8_t)MLX_MS_BYTE(ee[60]) / 8192.0f;

	step = ((ee[63] & 0x3000) >> 12) * 10;
	p->ct[0] = -40;
	p->ct[1] = 0;
	p->ct[2] = MLX_NIBBLE2(ee[63]) * step;
	p->ct[3] = p->ct[2] + MLX_NIBBLE3(ee[63]) * step;
	p->ct[4] = 400;
	ks_to_scale = 1 << (MLX_NIBBLE1(ee[63]) + 8);
	p->ks_to[0] = (int8_t)MLX_LS_BYTE(ee[61]) / (float)ks_to_scale;
	p->ks_to[1] = (int8_t)MLX_MS_BYTE(ee[61]) / (float)ks_to_scale;
	p->ks_to[2] = (int8_t)MLX_LS_BYTE(ee[62]) / (float)ks_to_scale;
	p->ks_to[3] = (int8_t)MLX_MS_BYTE(ee[62]) / (float)ks_to_scale;
	p->ks_to[4] = -0.0002f;

	alpha_scale = MLX_NIBBLE4(ee[32]) + 27;
	p->cp_offset[0] = MLX_LSBITS_10(ee[58]);
	if (p->cp_offset[0] > 511)
		p->cp_offset[0] -= 1024;
	p->cp_offset[1] = MLX_MSBITS_6(ee[58]);
	if (p->cp_offset[1] > 31)
		p->cp_offset[1] -= 64;
	p->cp_offset[1] += p->cp_offset[0];
	p->cp_alpha[0] = MLX_LSBITS_10(ee[57]);
	if (p->cp_alpha[0] > 511)
		p->cp_alpha[0] -= 1024;
	p->cp_alpha[0] /= pow2f_i(alpha_scale);
	p->cp_alpha[1] = MLX_MSBITS_6(ee[57]);
	if (p->cp_alpha[1] > 31)
		p->cp_alpha[1] -= 64;
	p->cp_alpha[1] = (1.0f + p->cp_alpha[1] / 128.0f) * p->cp_alpha[0];
	p->cp_kta = (int8_t)MLX_LS_BYTE(ee[59]) / pow2f_i(MLX_NIBBLE2(ee[56]) + 8);
	p->cp_kv = (int8_t)MLX_MS_BYTE(ee[59]) / pow2f_i(MLX_NIBBLE3(ee[56]));

	p->calibration_mode_ee = ((ee[10] & 0x0800) >> 4) ^ 0x80;
	p->il_chess_c[0] = ee[53] & 0x003f;
	if (p->il_chess_c[0] > 31)
		p->il_chess_c[0] -= 64;
	p->il_chess_c[0] /= 16.0f;
	p->il_chess_c[1] = (ee[53] & 0x07c0) >> 6;
	if (p->il_chess_c[1] > 15)
		p->il_chess_c[1] -= 32;
	p->il_chess_c[1] /= 2.0f;
	p->il_chess_c[2] = (ee[53] & 0xf800) >> 11;
	if (p->il_chess_c[2] > 15)
		p->il_chess_c[2] -= 32;
	p->il_chess_c[2] /= 8.0f;

	scale2 = MLX_NIBBLE1(ee[32]);
	scale = MLX_NIBBLE2(ee[32]);
	n = MLX_NIBBLE3(ee[32]);
	alpha_scale = MLX_NIBBLE4(ee[32]) + 30;
	alpha_ref = ee[33];
	for (i = 0; i < 6; i++) {
		pix = i * 4;
		acc_row[pix + 0] = MLX_NIBBLE1(ee[34 + i]);
		acc_row[pix + 1] = MLX_NIBBLE2(ee[34 + i]);
		acc_row[pix + 2] = MLX_NIBBLE3(ee[34 + i]);
		acc_row[pix + 3] = MLX_NIBBLE4(ee[34 + i]);
	}
	for (i = 0; i < MLX90640_ROWS; i++)
		if (acc_row[i] > 7)
			acc_row[i] -= 16;
	for (i = 0; i < 8; i++) {
		pix = i * 4;
		acc_col[pix + 0] = MLX_NIBBLE1(ee[40 + i]);
		acc_col[pix + 1] = MLX_NIBBLE2(ee[40 + i]);
		acc_col[pix + 2] = MLX_NIBBLE3(ee[40 + i]);
		acc_col[pix + 3] = MLX_NIBBLE4(ee[40 + i]);
	}
	for (i = 0; i < MLX90640_COLS; i++)
		if (acc_col[i] > 7)
			acc_col[i] -= 16;
	for (i = 0; i < MLX90640_ROWS; i++) {
		for (j = 0; j < MLX90640_COLS; j++) {
			pix = i * MLX90640_COLS + j;
			alpha_tmp[pix] = (ee[64 + pix] & 0x03f0) >> 4;
			if (alpha_tmp[pix] > 31)
				alpha_tmp[pix] -= 64;
			alpha_tmp[pix] = alpha_tmp[pix] * (1 << scale2);
			alpha_tmp[pix] = alpha_ref + (acc_row[i] << n) +
				(acc_col[j] << scale) + alpha_tmp[pix];
			alpha_tmp[pix] = alpha_tmp[pix] / pow2f_i(alpha_scale);
			alpha_tmp[pix] -= p->tgc * (p->cp_alpha[0] + p->cp_alpha[1]) * 0.5f;
			alpha_tmp[pix] = MLX_SCALE_ALPHA / alpha_tmp[pix];
		}
	}
	tmp = alpha_tmp[0];
	for (i = 1; i < MLX90640_PIXELS; i++)
		if (alpha_tmp[i] > tmp)
			tmp = alpha_tmp[i];
	p->alpha_scale = 0;
	while (tmp < 32767.4f) {
		tmp *= 2.0f;
		p->alpha_scale++;
	}
	for (i = 0; i < MLX90640_PIXELS; i++)
		p->alpha[i] = (uint16_t)(alpha_tmp[i] * pow2f_i(p->alpha_scale) + 0.5f);

	scale2 = MLX_NIBBLE1(ee[16]);
	scale = MLX_NIBBLE2(ee[16]);
	n = MLX_NIBBLE3(ee[16]);
	offset_ref = (int16_t)ee[17];
	for (i = 0; i < 6; i++) {
		pix = i * 4;
		occ_row[pix + 0] = MLX_NIBBLE1(ee[18 + i]);
		occ_row[pix + 1] = MLX_NIBBLE2(ee[18 + i]);
		occ_row[pix + 2] = MLX_NIBBLE3(ee[18 + i]);
		occ_row[pix + 3] = MLX_NIBBLE4(ee[18 + i]);
	}
	for (i = 0; i < MLX90640_ROWS; i++)
		if (occ_row[i] > 7)
			occ_row[i] -= 16;
	for (i = 0; i < 8; i++) {
		pix = i * 4;
		occ_col[pix + 0] = MLX_NIBBLE1(ee[24 + i]);
		occ_col[pix + 1] = MLX_NIBBLE2(ee[24 + i]);
		occ_col[pix + 2] = MLX_NIBBLE3(ee[24 + i]);
		occ_col[pix + 3] = MLX_NIBBLE4(ee[24 + i]);
	}
	for (i = 0; i < MLX90640_COLS; i++)
		if (occ_col[i] > 7)
			occ_col[i] -= 16;
	for (i = 0; i < MLX90640_ROWS; i++) {
		for (j = 0; j < MLX90640_COLS; j++) {
			pix = i * MLX90640_COLS + j;
			p->offset[pix] = MLX_MSBITS_6(ee[64 + pix]);
			if (p->offset[pix] > 31)
				p->offset[pix] -= 64;
			p->offset[pix] = p->offset[pix] * (1 << scale2);
			p->offset[pix] += offset_ref + (occ_row[i] << n) +
				(occ_col[j] << scale);
		}
	}

	kta_rc[0] = (int8_t)MLX_MS_BYTE(ee[54]);
	kta_rc[2] = (int8_t)MLX_LS_BYTE(ee[54]);
	kta_rc[1] = (int8_t)MLX_MS_BYTE(ee[55]);
	kta_rc[3] = (int8_t)MLX_LS_BYTE(ee[55]);
	scale = MLX_NIBBLE2(ee[56]) + 8;
	scale2 = MLX_NIBBLE1(ee[56]);
	for (i = 0; i < MLX90640_ROWS; i++) {
		for (j = 0; j < MLX90640_COLS; j++) {
			pix = i * MLX90640_COLS + j;
			split = 2 * (pix / 32 - (pix / 64) * 2) + pix % 2;
			kta_tmp[pix] = (ee[64 + pix] & 0x000e) >> 1;
			if (kta_tmp[pix] > 3)
				kta_tmp[pix] -= 8;
			kta_tmp[pix] = kta_rc[split] + kta_tmp[pix] * (1 << scale2);
			kta_tmp[pix] /= pow2f_i(scale);
		}
	}
	tmp = fabsf(kta_tmp[0]);
	for (i = 1; i < MLX90640_PIXELS; i++)
		if (fabsf(kta_tmp[i]) > tmp)
			tmp = fabsf(kta_tmp[i]);
	p->kta_scale = 0;
	while (tmp < 63.4f) {
		tmp *= 2.0f;
		p->kta_scale++;
	}
	for (i = 0; i < MLX90640_PIXELS; i++) {
		tmp = kta_tmp[i] * pow2f_i(p->kta_scale);
		p->kta[i] = (int8_t)(tmp < 0 ? tmp - 0.5f : tmp + 0.5f);
	}

	kv_t[0] = MLX_NIBBLE4(ee[52]);
	kv_t[2] = MLX_NIBBLE3(ee[52]);
	kv_t[1] = MLX_NIBBLE2(ee[52]);
	kv_t[3] = MLX_NIBBLE1(ee[52]);
	for (i = 0; i < 4; i++)
		if (kv_t[i] > 7)
			kv_t[i] -= 16;
	scale = MLX_NIBBLE3(ee[56]);
	for (i = 0; i < MLX90640_ROWS; i++) {
		for (j = 0; j < MLX90640_COLS; j++) {
			pix = i * MLX90640_COLS + j;
			split = 2 * (pix / 32 - (pix / 64) * 2) + pix % 2;
			kv_tmp[pix] = kv_t[split] / pow2f_i(scale);
		}
	}
	tmp = fabsf(kv_tmp[0]);
	for (i = 1; i < MLX90640_PIXELS; i++)
		if (fabsf(kv_tmp[i]) > tmp)
			tmp = fabsf(kv_tmp[i]);
	p->kv_scale = 0;
	while (tmp < 63.4f) {
		tmp *= 2.0f;
		p->kv_scale++;
	}
	for (i = 0; i < MLX90640_PIXELS; i++) {
		tmp = kv_tmp[i] * pow2f_i(p->kv_scale);
		p->kv[i] = (int8_t)(tmp < 0 ? tmp - 0.5f : tmp + 0.5f);
	}

	mlx90640_extract_deviating_pixels(ee, p);
}

static float mlx90640_get_vdd(uint16_t *frame, const struct mlx90640_params *p)
{
	uint16_t res_ram = (frame[832] & MLX90640_CTRL_RES_MASK) >> MLX90640_CTRL_RES_SHIFT;
	float res_corr = pow2f_i(p->resolution_ee) / pow2f_i(res_ram);

	return (res_corr * (int16_t)frame[810] - p->vdd25) / p->k_vdd + 3.3f;
}

static float mlx90640_get_ta(uint16_t *frame, const struct mlx90640_params *p)
{
	float vdd = mlx90640_get_vdd(frame, p);
	int16_t ptat = (int16_t)frame[800];
	float ptat_art = (ptat / (ptat * p->alpha_ptat + (int16_t)frame[768])) *
			 pow2f_i(18);

	return (ptat_art / (1.0f + p->kv_ptat * (vdd - 3.3f)) -
		p->v_ptat25) / p->kt_ptat + 25.0f;
}

static void mlx90640_calculate_to(uint16_t *frame, const struct mlx90640_params *p,
				  float emissivity, float tr, float *result)
{
	float vdd = mlx90640_get_vdd(frame, p);
	float ta = mlx90640_get_ta(frame, p);
	float ta4 = ta + 273.15f;
	float tr4 = tr + 273.15f;
	float ta_tr, gain, ir_cp[2], ir_data, alpha_comp;
	float sx, to, alpha_corr_r[4], kta_scale, kv_scale, alpha_scale;
	float kta, kv;
	uint8_t mode = (frame[832] & MLX90640_CTRL_MEAS_MODE_MASK) >> 5;
	int8_t il_pattern, chess_pattern, pattern, conversion_pattern, range;
	uint16_t subpage = frame[833];
	int pix;

	ta4 *= ta4;
	ta4 *= ta4;
	tr4 *= tr4;
	tr4 *= tr4;
	ta_tr = tr4 - (tr4 - ta4) / emissivity;

	kta_scale = pow2f_i(p->kta_scale);
	kv_scale = pow2f_i(p->kv_scale);
	alpha_scale = pow2f_i(p->alpha_scale);

	alpha_corr_r[0] = 1.0f / (1.0f + p->ks_to[0] * 40.0f);
	alpha_corr_r[1] = 1.0f;
	alpha_corr_r[2] = 1.0f + p->ks_to[1] * p->ct[2];
	alpha_corr_r[3] = alpha_corr_r[2] *
		(1.0f + p->ks_to[2] * (p->ct[3] - p->ct[2]));

	gain = (float)p->gain_ee / (int16_t)frame[778];

	ir_cp[0] = (int16_t)frame[776] * gain;
	ir_cp[1] = (int16_t)frame[808] * gain;
	ir_cp[0] -= p->cp_offset[0] * (1.0f + p->cp_kta * (ta - 25.0f)) *
		(1.0f + p->cp_kv * (vdd - 3.3f));
	if (mode == p->calibration_mode_ee) {
		ir_cp[1] -= p->cp_offset[1] * (1.0f + p->cp_kta * (ta - 25.0f)) *
			(1.0f + p->cp_kv * (vdd - 3.3f));
	} else {
		ir_cp[1] -= (p->cp_offset[1] + p->il_chess_c[0]) *
			(1.0f + p->cp_kta * (ta - 25.0f)) *
			(1.0f + p->cp_kv * (vdd - 3.3f));
	}

	for (pix = 0; pix < MLX90640_PIXELS; pix++) {
		il_pattern = pix / 32 - (pix / 64) * 2;
		chess_pattern = il_pattern ^ (pix - (pix / 2) * 2);
		conversion_pattern = ((pix + 2) / 4 - (pix + 3) / 4 +
				      (pix + 1) / 4 - pix / 4) *
				     (1 - 2 * il_pattern);
		pattern = mode == 0 ? il_pattern : chess_pattern;

		if (pattern != (int8_t)subpage)
			continue;

		ir_data = (int16_t)frame[pix] * gain;
		kta = p->kta[pix] / kta_scale;
		kv = p->kv[pix] / kv_scale;
		ir_data -= p->offset[pix] * (1.0f + kta * (ta - 25.0f)) *
			(1.0f + kv * (vdd - 3.3f));

		if (mode != p->calibration_mode_ee)
			ir_data += p->il_chess_c[2] * (2 * il_pattern - 1) -
				p->il_chess_c[1] * conversion_pattern;

		ir_data -= p->tgc * ir_cp[subpage];
		ir_data /= emissivity;

		alpha_comp = MLX_SCALE_ALPHA * alpha_scale / p->alpha[pix];
		alpha_comp *= 1.0f + p->ks_ta * (ta - 25.0f);

		sx = alpha_comp * alpha_comp * alpha_comp *
			(ir_data + alpha_comp * ta_tr);
		sx = sqrtf(sqrtf(sx)) * p->ks_to[1];
		to = sqrtf(sqrtf(ir_data /
			(alpha_comp * (1.0f - p->ks_to[1] * 273.15f) + sx) +
			ta_tr)) - 273.15f;

		if (to < p->ct[1])
			range = 0;
		else if (to < p->ct[2])
			range = 1;
		else if (to < p->ct[3])
			range = 2;
		else
			range = 3;

		to = sqrtf(sqrtf(ir_data /
			(alpha_comp * alpha_corr_r[range] *
			 (1.0f + p->ks_to[range] * (to - p->ct[range]))) +
			ta_tr)) - 273.15f;
		result[pix] = to;
	}

	mlx90640_bad_pixels_correction((uint16_t *)p->broken_pixels,
				       result, mode, p);
	mlx90640_bad_pixels_correction((uint16_t *)p->outlier_pixels,
				       result, mode, p);
}

/* ------------------------------------------------------------------ */
/* MLX90640 sensor access                                             */
/* ------------------------------------------------------------------ */

/*
 * Find the MLX90640 IIO device and return the path to its sysfs directory.
 */
static char *find_mlx90640_device(void)
{
	const char *base = "/sys/bus/iio/devices";
	DIR *dir;
	struct dirent *entry;
	char path[512];
	char name[128];
	FILE *fp;
	int found = 0;
	static char result[512];

	dir = opendir(base);
	if (!dir)
		return NULL;

	while ((entry = readdir(dir)) != NULL) {
		if (entry->d_name[0] == '.')
			continue;

		snprintf(path, sizeof(path), "%s/%s/name", base, entry->d_name);
		fp = fopen(path, "r");
		if (!fp)
			continue;

		if (fgets(name, sizeof(name), fp)) {
			if (strstr(name, "mlx90640")) {
				snprintf(result, sizeof(result), "%s/%s",
					 base, entry->d_name);
				found = 1;
				fclose(fp);
				break;
			}
		}
		fclose(fp);
	}
	closedir(dir);

	return found ? result : NULL;
}

static int find_mlx90640_i2c(int *bus, int *addr)
{
	char dev_path[PATH_MAX];
	char link_path[PATH_MAX];
	char *base;
	char *dash;
	char *end;
	ssize_t len;

	char *iio_path = find_mlx90640_device();
	if (!iio_path)
		return -1;

	snprintf(dev_path, sizeof(dev_path), "%s/device", iio_path);
	len = readlink(dev_path, link_path, sizeof(link_path) - 1);
	if (len < 0)
		return -1;

	link_path[len] = '\0';
	base = strrchr(link_path, '/');
	base = base ? base + 1 : link_path;
	dash = strchr(base, '-');
	if (!dash)
		return -1;

	*bus = strtol(base, &end, 10);
	if (end != dash)
		return -1;
	*addr = strtol(dash + 1, &end, 16);
	if (*bus < 0 || *addr <= 0)
		return -1;

	return 0;
}

static int mlx90640_direct_i2c_open(struct sensor_ctx *sensor, int refresh_rate)
{
	char dev[64];
	uint16_t ee[MLX90640_EE_WORDS];
	int i;

	if (find_mlx90640_i2c(&sensor->i2c_bus, &sensor->i2c_addr) < 0) {
		sensor->i2c_bus = MLX90640_DEFAULT_I2C_BUS;
		sensor->i2c_addr = MLX90640_DEFAULT_I2C_ADDR;
	}

	snprintf(dev, sizeof(dev), "/dev/i2c-%d", sensor->i2c_bus);
	sensor->fd = open(dev, O_RDWR);
	if (sensor->fd < 0)
		return -1;

	if (ioctl(sensor->fd, I2C_SLAVE_FORCE, sensor->i2c_addr) < 0)
		goto fail;

	if (mlx90640_set_refresh_rate(sensor->fd, refresh_rate) < 0)
		fprintf(stderr, "Warning: failed to set MLX90640 refresh rate: %s\n",
			strerror(errno));

	for (i = 0; i < MLX90640_EE_WORDS; i++) {
		if (mlx90640_i2c_read_words(sensor->fd, MLX90640_EEPROM_START + i,
					    &ee[i], 1) < 0)
			goto fail;
	}

	mlx90640_extract_params(ee, &sensor->params);
	sensor->direct_i2c = true;
	sensor->have_last_to = false;

	printf("Sensor data: direct I2C %s addr 0x%02x (official MLX90640 math, target %d fps)\n",
	       dev, sensor->i2c_addr, refresh_rate);
	return 0;

fail:
	close(sensor->fd);
	sensor->fd = -1;
	return -1;
}

static int sensor_open(struct sensor_ctx *sensor, const char *dev_path,
		       int refresh_rate)
{
	char path[PATH_MAX];
	int len;

	memset(sensor, 0, sizeof(*sensor));
	sensor->fd = -1;

	if (!dev_path && mlx90640_direct_i2c_open(sensor, refresh_rate) == 0)
		return 0;

	if (!dev_path)
		fprintf(stderr, "Direct I2C open failed, falling back to IIO frame_data: %s\n",
			strerror(errno));

	if (dev_path) {
		len = snprintf(path, sizeof(path), "%s/frame_data", dev_path);
	} else {
		char *auto_path = find_mlx90640_device();
		if (!auto_path) {
			fprintf(stderr, "MLX90640 IIO device not found.\n");
			return -1;
		}
		printf("IIO device: %s\n", auto_path);
		len = snprintf(path, sizeof(path), "%s/frame_data", auto_path);
	}

	if (len < 0 || (size_t)len >= sizeof(path)) {
		fprintf(stderr, "IIO frame_data path is too long\n");
		return -1;
	}

	/* Save path for sensor_read_frame re-open */
	snprintf(sensor->frame_path, sizeof(sensor->frame_path), "%s", path);

	sensor->fd = open(path, O_RDONLY);
	if (sensor->fd < 0) {
		fprintf(stderr, "Cannot open %s: %s\n", path, strerror(errno));
		fprintf(stderr, "Make sure the mlx90640 driver is loaded.\n");
		return -1;
	}

	printf("Sensor data: %s\n", path);
	return 0;
}

/*
 * Read a complete frame from the sysfs frame_data attribute.
 * sysfs does not support lseek — after a read the file position
 * stays at EOF.  Close + re-open resets position to 0 each time.
 */
static int sensor_read_frame(struct sensor_ctx *sensor, s32 *frame)
{
	ssize_t ret;
	uint16_t raw[MLX90640_FRAME_WORDS];
	int i;

	if (sensor->direct_i2c) {
		if (mlx90640_read_i2c_frame(sensor->fd, raw) < 0)
			return -1;

		if (!sensor->have_last_to) {
			for (i = 0; i < MLX90640_PIXELS; i++)
				sensor->last_to[i] = 25.0f;
			sensor->have_last_to = true;
		}

		mlx90640_calculate_to(raw, &sensor->params, 0.95f,
				      mlx90640_get_ta(raw, &sensor->params) - 8.0f,
				      sensor->last_to);

		for (i = 0; i < MLX90640_PIXELS; i++) {
			float t = sensor->last_to[i];
			if (!isfinite(t))
				t = 25.0f;
			frame[i] = (s32)lrintf(t * 1000.0f);
		}

		return 0;
	}

	close(sensor->fd);
	sensor->fd = open(sensor->frame_path, O_RDONLY);
	if (sensor->fd < 0) {
		fprintf(stderr, "Frame re-open failed: %s\n", strerror(errno));
		return -1;
	}

	ret = read(sensor->fd, frame, MLX90640_FRAME_BYTES);
	if (ret != MLX90640_FRAME_BYTES) {
		if (ret >= 0)
			fprintf(stderr, "Frame short read: %zd / %zu bytes\n",
				ret, (size_t)MLX90640_FRAME_BYTES);
		return -1;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* Thermal image rendering                                            */
/* ------------------------------------------------------------------ */

/*
 * Upscale 32x24 sensor data to display resolution using thermal colormap.
 * RAW_BLOCK_RENDER can bypass interpolation so every sensor pixel is shown
 * as one enlarged block.
 *
 * The image is centered in the framebuffer with the correct aspect ratio.
 */
static void render_thermal_image(const struct fb_info *fb,
				  const s32 *frame, s32 temp_min, s32 temp_max)
{
	uint32_t display_w = fb->vinfo.xres;
	uint32_t display_h = fb->vinfo.yres;
	uint32_t img_w, img_h, offset_x, offset_y;
	uint32_t px, py;

	/*
	 * Calculate image area: maintain 4:3 aspect ratio,
	 * fit within the display with margins for the scale bar.
	 */
#define SCALE_BAR_WIDTH  40
#define MARGIN           20

	uint32_t avail_w = display_w - SCALE_BAR_WIDTH - MARGIN * 2;
	uint32_t avail_h = display_h - MARGIN * 2;

	double aspect = (double)MLX90640_COLS / (double)MLX90640_ROWS;

	if ((double)avail_w / avail_h > aspect) {
		img_h = avail_h;
		img_w = (uint32_t)(img_h * aspect);
	} else {
		img_w = avail_w;
		img_h = (uint32_t)(img_w / aspect);
	}

	offset_x = (avail_w - img_w) / 2 + MARGIN;
	offset_y = (display_h - img_h) / 2;

	/*
	 * For portrait-oriented display (1080x1920), rotate the
	 * image: map sensor rows to display columns and sensor cols
	 * to display rows. This fills more of the screen.
	 *
	 * If display is portrait (height > width * 1.5), use rotated layout.
	 */
	int rotated = (display_h > display_w * 3 / 2);

	if (rotated) {
		/*
		 * Rotate 90°: sensor columns (32) -> display rows,
		 *              sensor rows (24) -> display columns.
		 */
		uint32_t r_img_w = img_h;
		uint32_t r_img_h = img_w;
		uint32_t r_offset_x = (avail_w - r_img_w) / 2 + MARGIN;
		uint32_t r_offset_y = (display_h - r_img_h) / 2;

		if (RAW_BLOCK_RENDER) {
			uint32_t sx, sy;

			for (sy = 0; sy < MLX90640_ROWS; sy++) {
				for (sx = 0; sx < MLX90640_COLS; sx++) {
					uint32_t x0 = r_offset_x +
						(MLX90640_ROWS - 1 - sy) * r_img_w / MLX90640_ROWS;
					uint32_t x1 = r_offset_x +
						(MLX90640_ROWS - sy) * r_img_w / MLX90640_ROWS;
					uint32_t y0 = r_offset_y +
						sx * r_img_h / MLX90640_COLS;
					uint32_t y1 = r_offset_y +
						(sx + 1) * r_img_h / MLX90640_COLS;
					uint8_t ci = temp_to_cmap(frame[sy * MLX90640_COLS + sx],
								  temp_min, temp_max);
					uint32_t pixel = rgb_to_pixel(fb,
						colormap[ci].r, colormap[ci].g, colormap[ci].b);

					fb_fill_rect(fb, x0, y0, x1 - x0, y1 - y0, pixel);
				}
			}
		} else {
			for (py = 0; py < r_img_h; py++) {
				for (px = 0; px < r_img_w; px++) {
					/* Reverse mapping: display (px,py) -> sensor (col,row) */
					double sx = (double)py / (double)r_img_h *
						    (MLX90640_COLS - 1);
					double sy = (1.0 - (double)px / (double)r_img_w) *
						    (MLX90640_ROWS - 1);

					/* Bilinear interpolation */
					int x0 = (int)sx;
					int y0 = (int)sy;
					int x1 = (x0 + 1 < MLX90640_COLS) ? x0 + 1 : x0;
					int y1 = (y0 + 1 < MLX90640_ROWS) ? y0 + 1 : y0;
					double fx = sx - x0;
					double fy = sy - y0;

					double v00 = frame[y0 * MLX90640_COLS + x0];
					double v10 = frame[y0 * MLX90640_COLS + x1];
					double v01 = frame[y1 * MLX90640_COLS + x0];
					double v11 = frame[y1 * MLX90640_COLS + x1];

					double val = v00 * (1 - fx) * (1 - fy) +
						     v10 * fx * (1 - fy) +
						     v01 * (1 - fx) * fy +
						     v11 * fx * fy;

					uint8_t ci = temp_to_cmap((s32)val, temp_min, temp_max);
					uint32_t pixel = rgb_to_pixel(fb,
						colormap[ci].r, colormap[ci].g, colormap[ci].b);

					fb_put_pixel(fb, r_offset_x + px,
						     r_offset_y + py, pixel);
				}
			}
		}

		/* Draw scale bar in rotated position */
		int bar_x = display_w - SCALE_BAR_WIDTH;
		int bar_y = MARGIN;
		int bar_h = display_h - MARGIN * 2;
		int i;

		for (i = 0; i < bar_h; i++) {
			double t = 1.0 - (double)i / (double)(bar_h - 1);
			uint8_t ci = (uint8_t)(t * (CMAP_SIZE - 1));
			uint32_t color = rgb_to_pixel(fb,
				colormap[ci].r, colormap[ci].g, colormap[ci].b);

			fb_fill_rect(fb, bar_x, bar_y + i,
				     SCALE_BAR_WIDTH - 10, 1, color);
		}

		/* Scale labels */
		{
			char label[32];
			int text_color = rgb_to_pixel(fb, 255, 255, 255);
			snprintf(label, sizeof(label), "%4.1fC", temp_max / 1000.0);
			fb_draw_text(fb, bar_x, bar_y, text_color, label);
			snprintf(label, sizeof(label), "%4.1fC", temp_min / 1000.0);
			fb_draw_text(fb, bar_x, bar_y + bar_h - FONT_HEIGHT * FONT_SCALE,
				     text_color, label);
		}

		/* Center crosshair */
		{
			int cx = r_offset_x + r_img_w / 2;
			int cy = r_offset_y + r_img_h / 2;
			uint32_t cross_color = rgb_to_pixel(fb, 255, 255, 255);

			s32 center_temp = frame[12 * MLX90640_COLS + 16];
			char clabel[32];
			snprintf(clabel, sizeof(clabel), "%4.1fC", center_temp / 1000.0);
			fb_draw_text(fb, cx - 30, cy - 32, cross_color, clabel);

			/* Small cross */
			int dx;
			for (dx = -5; dx <= 5; dx++) {
				fb_put_pixel(fb, cx + dx, cy, cross_color);
				fb_put_pixel(fb, cx, cy + dx, cross_color);
			}
		}
	} else {
		/* Landscape: standard mapping */
		if (RAW_BLOCK_RENDER) {
			uint32_t sx, sy;

			for (sy = 0; sy < MLX90640_ROWS; sy++) {
				for (sx = 0; sx < MLX90640_COLS; sx++) {
					uint32_t x0 = offset_x +
						sx * img_w / MLX90640_COLS;
					uint32_t x1 = offset_x +
						(sx + 1) * img_w / MLX90640_COLS;
					uint32_t y0 = offset_y +
						sy * img_h / MLX90640_ROWS;
					uint32_t y1 = offset_y +
						(sy + 1) * img_h / MLX90640_ROWS;
					uint8_t ci = temp_to_cmap(frame[sy * MLX90640_COLS + sx],
								  temp_min, temp_max);
					uint32_t pixel = rgb_to_pixel(fb,
						colormap[ci].r, colormap[ci].g, colormap[ci].b);

					fb_fill_rect(fb, x0, y0, x1 - x0, y1 - y0, pixel);
				}
			}
		} else {
			for (py = 0; py < img_h; py++) {
				for (px = 0; px < img_w; px++) {
					double sx = (double)px / (double)img_w *
						    (MLX90640_COLS - 1);
					double sy = (double)py / (double)img_h *
						    (MLX90640_ROWS - 1);

					int x0 = (int)sx;
					int y0 = (int)sy;
					int x1 = (x0 + 1 < MLX90640_COLS) ? x0 + 1 : x0;
					int y1 = (y0 + 1 < MLX90640_ROWS) ? y0 + 1 : y0;
					double fx = sx - x0;
					double fy = sy - y0;

					double v00 = frame[y0 * MLX90640_COLS + x0];
					double v10 = frame[y0 * MLX90640_COLS + x1];
					double v01 = frame[y1 * MLX90640_COLS + x0];
					double v11 = frame[y1 * MLX90640_COLS + x1];

					double val = v00 * (1 - fx) * (1 - fy) +
						     v10 * fx * (1 - fy) +
						     v01 * (1 - fx) * fy +
						     v11 * fx * fy;

					uint8_t ci = temp_to_cmap((s32)val, temp_min, temp_max);
					uint32_t pixel = rgb_to_pixel(fb,
						colormap[ci].r, colormap[ci].g, colormap[ci].b);

					fb_put_pixel(fb, offset_x + px, offset_y + py, pixel);
				}
			}
		}

		/* Scale bar on the right */
		int bar_x = offset_x + img_w + 10;
		int bar_y = offset_y;
		int bar_h = img_h;
		int i;

		for (i = 0; i < bar_h; i++) {
			double t = 1.0 - (double)i / (double)(bar_h - 1);
			uint8_t ci = (uint8_t)(t * (CMAP_SIZE - 1));
			uint32_t color = rgb_to_pixel(fb,
				colormap[ci].r, colormap[ci].g, colormap[ci].b);
			fb_fill_rect(fb, bar_x, bar_y + i, 25, 1, color);
		}

		/* Labels */
		{
			char label[32];
			uint32_t text_color = rgb_to_pixel(fb, 255, 255, 255);
			snprintf(label, sizeof(label), "%.1fC", temp_max / 1000.0);
			fb_draw_text(fb, bar_x, bar_y - FONT_HEIGHT * FONT_SCALE,
				     text_color, label);
			snprintf(label, sizeof(label), "%.1fC", temp_min / 1000.0);
			fb_draw_text(fb, bar_x, bar_y + bar_h + 2, text_color, label);
		}
	}
}

/*
 * Draw info overlay: min/max/center temps, FPS, sensor status
 */
static void render_overlay(const struct fb_info *fb,
			    s32 temp_min, s32 temp_max, s32 temp_center,
			    double fps, int frame_count)
{
	uint32_t txt_color = rgb_to_pixel(fb, 255, 255, 255);
	uint32_t bg_color  = rgb_to_pixel(fb, 0, 0, 0);
	char line[128];
	int y = fb->vinfo.yres - TEXT_LINE_HEIGHT * 4 - 8;
	int x = 10;

	/* Background bar for text */
	fb_fill_rect(fb, x - 2, y - 2,
		     fb->vinfo.xres - x + 2, TEXT_LINE_HEIGHT * 4 + 6, bg_color);

	snprintf(line, sizeof(line), "MAX: %5.1fC  MIN: %5.1fC  CTR: %5.1fC",
		 temp_max / 1000.0, temp_min / 1000.0, temp_center / 1000.0);
	fb_draw_text(fb, x, y, txt_color, line);

	snprintf(line, sizeof(line), "FPS: %4.1f  Frames: %d",
		 fps, frame_count);
	fb_draw_text(fb, x, y + TEXT_LINE_HEIGHT, txt_color, line);

	snprintf(line, sizeof(line), "MLX90640 32x24 Thermal Camera");
	fb_draw_text(fb, x, y + TEXT_LINE_HEIGHT * 2, txt_color, line);

	snprintf(line, sizeof(line), "AtomPi-CA1 | MD0550M LCD");
	fb_draw_text(fb, x, y + TEXT_LINE_HEIGHT * 3, txt_color, line);
}

/* ------------------------------------------------------------------ */
/* Signal handling                                                    */
/* ------------------------------------------------------------------ */

static volatile int g_running = 1;

static void signal_handler(int sig)
{
	(void)sig;
	g_running = 0;
}

/* ------------------------------------------------------------------ */
/* Main                                                               */
/* ------------------------------------------------------------------ */

static void usage(const char *prog)
{
	printf("MLX90640 Thermal Camera Display\n");
	printf("Usage: %s [options]\n", prog);
	printf("Options:\n");
	printf("  -d <fbdev>   Framebuffer device (default: /dev/fb0)\n");
	printf("  -i <path>    IIO device sysfs path (default: auto-detect)\n");
	printf("  -r <fps>     Target refresh rate (default: 16, choices up to 64)\n");
	printf("  -h           Show this help\n");
}

int main(int argc, char *argv[])
{
	struct fb_info fb;
	const char *fbdev = "/dev/fb0";
	const char *iiopath = NULL;
	struct sensor_ctx sensor;
	s32 *frame;
	int refresh_rate = 16;
	int opt;
	struct timespec frame_time, prev_time;
	double fps = 0.0;
	int frame_count = 0;

	while ((opt = getopt(argc, argv, "d:i:r:h")) != -1) {
		switch (opt) {
		case 'd':
			fbdev = optarg;
			break;
		case 'i':
			iiopath = optarg;
			break;
		case 'r':
			refresh_rate = atoi(optarg);
			if (refresh_rate < 1 || refresh_rate > 64)
				refresh_rate = 16;
			break;
		case 'h':
		default:
			usage(argv[0]);
			return opt == 'h' ? 0 : 1;
		}
	}

	printf("=== MLX90640 Thermal Camera ===\n");
	printf("Build: %s %s | FONT_SCALE=%d | RAW_BLOCK_RENDER=%d | userspace_math=1\n",
	       __DATE__, __TIME__, FONT_SCALE, RAW_BLOCK_RENDER);

	/* Initialize colormap */
	colormap_init();

	/* Allocate frame buffer */
	frame = malloc(MLX90640_FRAME_BYTES);
	if (!frame) {
		fprintf(stderr, "malloc failed\n");
		return 1;
	}

	/* Open sensor */
	if (sensor_open(&sensor, iiopath, refresh_rate) < 0) {
		free(frame);
		return 1;
	}

	/* Open framebuffer */
	if (fb_open(&fb, fbdev) < 0) {
		fprintf(stderr, "Failed to open framebuffer.\n");
		fprintf(stderr, "If Wayland is running, stop it first:\n");
		fprintf(stderr, "  systemctl stop weston\n");
		if (sensor.fd >= 0)
			close(sensor.fd);
		free(frame);
		return 1;
	}

	/* Signal handlers */
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	/* Clear screen */
	fb_fill_rect(&fb, 0, 0, fb.vinfo.xres, fb.vinfo.yres,
		     rgb_to_pixel(&fb, 0, 0, 0));

	clock_gettime(CLOCK_MONOTONIC, &prev_time);

	printf("\nRunning... Press Ctrl+C to stop.\n\n");

	/* Main loop */
	while (g_running) {
		int i;
		s32 raw_min, raw_max, temp_center;
		s32 display_min, display_max;
		s64 temp_sum = 0;

		/* Read sensor frame */
		if (sensor_read_frame(&sensor, frame) < 0) {
			fprintf(stderr, "Frame read error\n");
			usleep(100000);
			continue;
		}

		/* Find min/max/center */
		raw_min = frame[0];
		raw_max = frame[0];
		for (i = 0; i < MLX90640_PIXELS; i++) {
			if (frame[i] < raw_min) raw_min = frame[i];
			if (frame[i] > raw_max) raw_max = frame[i];
			temp_sum += frame[i];
		}

		/* Center pixel (row 12, col 16) */
		temp_center = frame[12 * MLX90640_COLS + 16];

			/* Auto display color scale with a small margin. */
			{
				s32 range = raw_max - raw_min;

				if (range < 1000)
					range = 1000;
				display_min = raw_min - range / 10;
				display_max = raw_max + range / 10;
			}

		/* Render thermal image */
		render_thermal_image(&fb, frame, display_min, display_max);

		/* Render info overlay */
		render_overlay(&fb, raw_min, raw_max, temp_center,
			       fps, frame_count);

		frame_count++;

		/* FPS calculation */
		clock_gettime(CLOCK_MONOTONIC, &frame_time);
		{
			double elapsed = (frame_time.tv_sec - prev_time.tv_sec) +
					 (frame_time.tv_nsec - prev_time.tv_nsec) / 1e9;
			if (elapsed > 0.5) {
				fps = frame_count / elapsed;
				prev_time = frame_time;
				frame_count = 0;
			}
		}

		/* Direct I2C already blocks until DATA_READY. */
		if (!sensor.direct_i2c)
			usleep(1000000 / refresh_rate);
	}

	/* Cleanup */
	fb_fill_rect(&fb, 0, 0, fb.vinfo.xres, fb.vinfo.yres,
		     rgb_to_pixel(&fb, 0, 0, 0));

	printf("\nShutting down...\n");

	fb_close(&fb);
	if (sensor.fd >= 0)
		close(sensor.fd);
	free(frame);

	return 0;
}
