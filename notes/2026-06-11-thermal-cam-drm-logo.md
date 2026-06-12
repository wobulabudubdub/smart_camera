---
type: debug-case-note
date: 2026-06-11
tags:
  - embedded
  - debug
  - linux
  - drm
  - framebuffer
  - mlx90640
status: resolved
---

# MLX90640 热像程序写入 fb0 但屏幕仍显示 ALIENTEK

## 背景

- 板卡：RK3568 AtomPi-CA1 / ATK-DLRK356X
- 屏幕：5.5 寸 1080x1920 MIPI DSI
- 应用：`app/thermal_cam/thermal_cam.c`
- 传感器：MLX90640，IIO 设备为 `/sys/bus/iio/devices/iio:device1`
- 目标：运行 `thermal_cam` 后在 LCD 上显示热像图

## 现象

执行 `thermal_cam` 后，屏幕没有出现热像图，始终是黑底白字 `ALIENTEK`。

中间曾出现几个误导现象：

- `thermal_cam` 进程确实存在；
- `/dev/fb0` 能打开；
- MLX90640 的 `frame_data` 能读出温度数据；
- 把 `/dev/fb0` 抓回主机后，图片里确实有热像图；
- 但实际 LCD 肉眼仍然只显示 `ALIENTEK`。

## 关键证据

确认 app 正常写入 `/dev/fb0`：

```sh
dd if=/dev/fb0 bs=8294400 count=1 of=/tmp/fb0.raw
```

将 `/dev/fb0` 原始内容转图后，可以看到热像画面和底部文字，说明 app 的渲染逻辑已经生效。

同时查看 DRM 状态：

```sh
cat /sys/kernel/debug/dri/0/summary
```

问题状态下显示：

```text
Video Port1: ACTIVE
Connector: DSI-1
Smart0-win0: ACTIVE
format: RG16 little-endian
src: pos[0, 0] rect[654 x 270]
dst: pos[213, 825] rect[654 x 270]
```

这说明 MIPI DSI 当前显示的不是 `/dev/fb0` 对应的全屏 framebuffer，而是启动 logo 使用的一个 654x270 的 `RG16` DRM framebuffer。

## 根因

`thermal_cam` 原来只写 Linux fbdev `/dev/fb0`，但当前 LCD 实际扫描输出的是 DRM/KMS 里的启动 logo plane。

所以：

- app 写 `/dev/fb0` 是成功的；
- `/dev/fb0` 里确实有热像；
- 但 DSI 屏幕没有切到这个 framebuffer；
- 用户看到的仍是 DRM boot logo 层，即 `ALIENTEK`。

这不是 MLX90640 驱动读数问题，也不是 Makefile 编译问题，而是显示链路问题：fbdev 写入层和当前 DSI 显示层不是同一个。

## 修复

把 `thermal_cam` 改成优先使用 DRM/KMS：

- 打开 `/dev/dri/card0`
- 找到已连接的 DSI connector
- 创建 1080x1920 XRGB8888 dumb buffer
- 使用 `drmModeSetCrtc()` 将 DSI-1 直接切到该 buffer
- 保留原来的热图和文字绘制逻辑，只把底层绘制内存从 fbdev mmap 换成 KMS dumb buffer

修复后 DRM 状态变为：

```text
Video Port1: ACTIVE
Connector: DSI-1
Smart0-win0: ACTIVE
format: XR24 little-endian
src: pos[0, 0] rect[1080 x 1920]
dst: pos[0, 0] rect[1080 x 1920]
```

这说明程序已经接管 DSI 全屏显示。

## Makefile 相关修复

原先把新程序部署到 `/tmp/thermal_cam`，重启后会丢失，导致系统又运行旧的 `/usr/bin/thermal_cam`。

最终改为持久安装：

```sh
make deploy
```

安装目标：

```text
/usr/bin/thermal_cam
```

并使用 buildroot 工具链链接 `libdrm`：

```make
TOOLCHAIN_DIR := buildroot/output/rockchip_rk3568/host/bin
SYSROOT := buildroot/output/rockchip_rk3568/host/aarch64-buildroot-linux-gnu/sysroot
LDFLAGS += -ldrm
```

## 常用验证命令

查看热像程序进程：

```sh
ps -ef | grep thermal_cam | grep -v grep
pidof thermal_cam
```

停止旧进程：

```sh
pkill -9 -x thermal_cam
```

查看 DRM 当前显示层：

```sh
cat /sys/kernel/debug/dri/0/summary
```

查看程序是否链接 DRM：

```sh
ldd /usr/bin/thermal_cam
```

期望看到：

```text
libdrm.so.2 => /usr/lib64/libdrm.so.2
```

## 调试教训

- 看到 `/dev/fb0` 有内容，不代表 LCD 正在显示 `/dev/fb0`。
- 在 DRM/KMS 系统上，启动 logo、fbcon、应用 framebuffer 可能不是同一个 plane/framebuffer。
- 如果屏幕固定显示 boot logo，优先查：
  - `/sys/kernel/debug/dri/0/summary`
  - 当前 active plane 的 format、src/dst rect、fb id
  - app 是否只是写了 fbdev，而没有接管 KMS
- 部署到 `/tmp` 只适合临时测试，重启后必须安装到 `/usr/bin` 或其他持久路径。

## 后续 TODO

- MLX90640 驱动仍需要继续修：
  - subpage0/subpage1 合成
  - EEPROM 完整标定算法
  - 当前简化温度公式会导致棋盘/固定纹理
- `thermal_cam` 可以后续加命令行选项：
  - `--drm`
  - `--fbdev`
  - `--device /dev/dri/card0`
  - `--connector DSI-1`
