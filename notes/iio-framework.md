# Linux IIO 框架入门

## 一句话理解

> **IIO = Industrial I/O = 传感器驱动通用框架。**
> 它帮你处理了"字符设备、sysfs、buffer、trigger"这些所有传感器驱动都要做的重复工作，你只写"这颗传感器独有的逻辑"。

---

## 1. 为什么需要 IIO？

没有 IIO 时，10 个传感器驱动各写各的，用户接口碎片化：

```
传感器A:  cat /dev/sensorA  → 原始字节流，单位未知
传感器B:  cat /sys/xxx/temp  → 摄氏度浮点
传感器C:  ioctl(fd, GET_TEMP) → 整数，0.1°C
```

**有 IIO 后，所有传感器接口统一**：

```bash
cat /sys/bus/iio/devices/iio:device0/in_temp_input
# 返回: 25000  含义永远是"毫摄氏度"，格式永远是十进制整数
```

一套用户空间程序操作所有 IIO 设备。

---

## 2. 架构全景图

```
用户空间
══════════════════════════════════════════════════════
内核空间
                ┌─ /dev/iio:deviceX ── 批量读取（buffer 模式）
                │
    iio_dev ────┤
                │
                └─ /sys/bus/iio/devices/iio:deviceX/
                      ├── name
                      ├── in_temp_ambient_input   ← 单值读取
                      ├── in_temp_object0_input
                      ├── in_temp_object1_input
                      ├── ...
                      ├── frame_data              ← 自定义属性
                      └── buffer/
                            ├── enable
                            └── length
```

**IIO 一次性给你两种访问方式：**
- **sysfs**：`cat` 一下读一个值（慢，方便调试）
- **`/dev/iio:deviceX`**：内核 buffer + trigger 连续高速采集（快，生产环境用）

---

## 3. 核心概念对照表

| 概念 | 是什么 | 类比 |
|---|---|---|
| `struct iio_dev` | 一个 IIO 设备 | 手写驱动的 `struct cdev` |
| `struct iio_chan_spec` | 描述"我有什么数据通道" | 菜单 — 告诉框架"我有温度1、温度2…" |
| `struct iio_info` | 驱动回调函数表 | 手写驱动的 `struct file_operations` |
| `read_raw()` | "当前值是多少？" | 替代 `.read` |
| `write_raw()` | "把这个值写进去" | 替代 `.write` |
| **Buffer** | 内核环形缓冲区 | 高速采集时不丢数据的队列 |
| **Trigger** | 触发源（定时器/外部引脚） | "什么时候该采下一帧了"的信号 |
| `trigger_handler()` | trigger 触发后采数据 | buffer 模式下的数据生产者 |

---

## 4. 驱动作者要写的函数

以一个假想的温度传感器为例，IIO 驱动只需写这些：

```

┌──────────────────────────────────────────────┐
│ 1.  硬件操作函数                               │
│     i2c_read()   — 读 I2C 寄存器               │
│     i2c_write()  — 写 I2C 寄存器               │
│     校准算法     — 传感器特有的计算逻辑           │
│──────────────────────────────────────────────│
│ 2.  IIO 核心回调                               │
│     read_raw()         — "当前值是多少？"        │
│     write_raw()        — "写这个值进去"          │
│──────────────────────────────────────────────│
│ 3.  IIO 通道描述                               │
│     iio_chan_spec[]    — "我有什么数据"          │
│──────────────────────────────────────────────│
│ 4.  Buffer 相关（可选）                         │
│     trigger_handler()  — trigger 响了，采数据     │
│     buffer_setup_ops   — buffer 使能/关闭回调   │
│──────────────────────────────────────────────│
│ 5.  driver 结构体                              │
│     iio_info           — 把上面串起来的总表       │
│     probe() / remove() — 初始化/卸载            │
└──────────────────────────────────────────────┘
```

**你不需要写的：**
- `open()` / `release()` / `read()` — IIO 核心包了
- 设备号分配、cdev 注册 — IIO 核心包了
- sysfs 文件创建/删除 — IIO 核心根据 `iio_chan_spec` 自动生成
- poll / watermark — IIO 核心包了

---

## 5. 数据流向图

### 方式一：单值读取（sysfs Direct Mode）

```
用户:  cat /sys/bus/iio/devices/iio:device0/in_temp_input
         │
         ▼
     VFS → sysfs → IIO核心
         │
         │  "channel=0, mask=PROCESSED, 值是多少？"
         ▼
     你的 read_raw()
         │
         ├─ 读传感器 I2C 寄存器
         ├─ 跑校准算法
         ├─ *val = 25000
         └─ return IIO_VAL_INT
         │
         ▼
     IIO核心格式化 → "25000\n" → 返回给用户
```

### 方式二：高速连续采集（Buffer + Trigger 模式）

```
Trigger (hrtimer 100Hz)
    │
    │ 每隔 10ms 触发一次
    ▼
你的 trigger_handler()
    │
    ├─ 读传感器一整帧数据
    ├─ kmalloc 临时 buf
    ├─ buf[0]=环境温度, buf[1]=像素0, buf[2]=像素1 ...
    └─ iio_push_to_buffers_with_timestamp()
         │
         ▼
    IIO 环形 buffer（内核态）
         │
         │ 用户 read(/dev/iio:device0)
         ▼
    用户空间拿到结构化数据
```

---

## 6. 关键函数签名的直觉含义

```c
// read_raw: "框架问你：通道 channel 的值是多少？填进 *val 和 *val2"
// mask 区分：要原始值(IIO_CHAN_INFO_RAW)？处理后值(IIO_CHAN_INFO_PROCESSED)？scale？
static int my_read_raw(struct iio_dev *indio_dev,
                        struct iio_chan_spec const *channel,
                        int *val, int *val2, long mask)
{
    switch (mask) {
    case IIO_CHAN_INFO_PROCESSED:
        *val = read_sensor_and_calc();   // 你的硬件读取逻辑
        return IIO_VAL_INT;              // 告诉框架：值是一个 int，放在 *val
    }
}

// trigger_handler: "trigger 触发了，采一帧数据推给 buffer"
static irqreturn_t my_trigger_handler(int irq, void *p)
{
    s32 buf[CHANNEL_COUNT];
    buf[0] = read_channel_0();
    buf[1] = read_channel_1();
    iio_push_to_buffers_with_timestamp(indio_dev, buf, timestamp);
    iio_trigger_notify_done(indio_dev->trig);
    return IRQ_HANDLED;
}

// iio_chan_spec: "菜单 — 告诉框架我的数据长什么样"
struct iio_chan_spec my_channels[] = {
    {
        .type       = IIO_TEMP,            // 温度类型
        .indexed    = 1,
        .channel    = 0,                   // 通道编号
        .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),  // 支持处理后的值
        .scan_index = 0,                   // buffer 中排第几个
    },
};
```

---

## 7. 问自己这几个问题就能确定要不要用 IIO

| 问题 | Yes | No |
|---|---|---|
| 是传感器/ADC/DAC 吗？ | → IIO | → 别的框架 |
| 需要读数值吗？ | → IIO | → 可能不适合 |
| 需要连续高速采样吗？ | → IIO buffer + trigger | → sysfs 就够了 |
| 是摄像头吗？ | → V4L2，别用 IIO | |
| 是声卡吗？ | → ALSA，别用 IIO | |

---

## 8. 再看 mlx90640，全部对上了

```
mlx90640_probe()
    │
    ├─ ① devm_iio_device_alloc()              分配 iio_dev
    ├─ ② mlx90640_read_eeprom()               读校准数据（硬件特有）
    ├─ ③ mlx90640_extract_calibration()       解析校准参数（硬件特有）
    ├─ ④ mlx90640_build_channels()            构建通道菜单（IIO 通道描述）
    ├─ ⑤ indio_dev->info = &mlx90640_info     挂上回调表
    ├─ ⑥ iio_triggered_buffer_setup()         建立 buffer + trigger
    └─ ⑦ iio_device_register()                【一键生成所有用户接口】
              │
              ├── /dev/iio:device0
              ├── /sys/bus/iio/devices/iio:device0/in_temp_ambient_input
              ├── /sys/bus/iio/devices/iio:device0/in_temp_object0_input
              ├── ... (共 769 个 sysfs 文件，都是自动的)
              └── /sys/bus/iio/devices/iio:device0/frame_data  (自定义)
```

---

## 9. 总结

```
没有任何框架:  你要写 open、read、release、设备号分配、cdev、sysfs、buffer、poll
               驱动 A 和驱动 B 的用户接口不一样，碎片化
               工作量 = 硬件逻辑 + 字符设备 plumbing

用 IIO 框架:    你写 read_raw、write_raw、iio_chan_spec、trigger_handler
               所有 IIO 设备用户接口统一
               工作量 = 硬件逻辑（只有传感器特有的部分）
```

**IIO 就是传感器世界的"统一操作系统" — 你只负责描述你的传感器，剩下的由框架完成。**
