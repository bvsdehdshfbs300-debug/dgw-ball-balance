# 平衡球 ESP32-S3 端 (配合 MaixCAM2 `平衡球6.0_UDP.py`)

## 架构

```
手机热点
 ├─ MaixCAM2  --UDP广播 5005--> ESP32-S3 --UART--> 张大头 Emm42_V5 --> 摆杆
 │      └─ 视频流 --> 电脑(赛题要求1)
 └─ 电脑也可监听 5005 端口看球位置数据
```

## 接线

| ESP32-S3 | Emm42_V5 |
|---|---|
| GPIO17 (TX) | RX |
| GPIO18 (RX) | TX |
| GND | GND |

引脚可在 `main/config.h` 改。

## Emm42 驱动器屏幕菜单必须设置

- 串口波特率: 115200（与 config.h 一致）
- 地址: 1
- 校验方式: 0x6B（固定校验）
- 回应模式: Receive
- 细分: 16（= config.h 的 MICROSTEP，改了要同步改）
- 工作模式: 闭环

## 编译烧录

```
idf.py set-target esp32s3
idf.py menuconfig   # 可跳过, 默认即可
idf.py build flash monitor
```

先改 `main/config.h` 里的 `WIFI_SSID` / `WIFI_PASS`。

## 上电/测试流程

1. **手动把摆杆放平**（程序上电会把当前位置清零当作 0 度）
2. ESP32 上电 → 串口监视器应看到 `emm42 enabled` → `got ip`
3. MaixCAM2 连同一热点，运行 `平衡球6.0_UDP.py`，点屏幕 START
4. 串口监视器每 200ms 打印一行 `PD ball=.. tgt=.. ang=..`

## 首次上电必须验证的三件事

1. **方向**: 手动把球放在 +5cm 一侧不动，摆杆应当向"让球滚回中心"的方向倾斜。
   反了就把 `config.h` 的 `ANGLE_SIGN` 改成 `-1.0f`。
2. **清零指令**: `emm42_zero_here()` 用的是 `addr 0A 6D 6B`，如果上电后摆杆
   位置基准不对，对照你手上《Emm_V5.0 说明书》"通讯控制"章节核对功能码。
3. **绝对模式负位置**: `emm42_move_abs()` 里负位置用 方向字节=1 表示，
   如果摆杆只往一边动，需按说明书核对绝对模式下方向字节的定义。

## 断联保护

- 超过 300ms 没收到球位置（热点掉线 / MaixCAM 死机 / 丢球）→ 摆杆以 15°/s 匀速回平并保持。

## 电脑上抓包看数据（调试用）

```python
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("", 5005))
while True:
    print(s.recvfrom(64)[0].decode(), end="")
```

## 调参

参数都在 `main/config.h`：KP_DEG_PER_CM / KD_DEG_PER_CMS / MAX_CTRL_ANGLE_DEG 等，
含义和 MaixCAM 5.0 版一致，之前调过的经验直接沿用。
