| Supported Targets | ESP32-C3 |
| ----------------- | -------- |

# ESP32-C3 BLE LED Controller (60 LEDs, string control)

本项目基于 ESP32-C3，通过 BLE GATT 特征接收最多 60 位数字字符串，驱动 WS2812 灯带显示颜色模式。每个字符 `0~7` 代表一颗 LED 的颜色索引，零延迟解析并更新灯带。

## 功能概要

- **BLE 服务**
  - Service UUID `0x00FF`
  - Characteristic UUID `0xFF01`
  - Read / Write / Notify
- **字符串控制**
  - 写入连续 ASCII 数字串（`0~7`），长度 1~60
  - 不足 60 自动把剩余 LED 熄灭，超出 60 只取前 60
- **WS2812 驱动**
  - GPIO1 输出，RMT 精确时序
  - 预置 8 种颜色（0=灭，1=红，…，7=紫）

## 快速上手

```bash
idf.py set-target esp32c3
idf.py build
idf.py -p COMx flash monitor
```

## BLE 操作

1. 使用 nRF Connect / LightBlue 扫描并连接 `ESP-LED`
2. 定位 Service `0x00FF` → Characteristic `0xFF01`
3. 写入示例：
   - 全灭：`000000000000000000000000000000000000000000000000000000000000`
   - 全红：`111111111111111111111111111111111111111111111111111111111111`
   - 彩虹：`01234567012345670123456701234567012345670123456701234567`
4. 读取（Read）或订阅（Notify）可查看当前 60 位状态

## Python 示例

```python
import asyncio
from bleak import BleakClient

ADDR = "84:F7:03:18:DB:C6"   # 替换为设备 MAC
CHAR = "0000ff01-0000-1000-8000-00805f9b34fb"

async def demo():
    async with BleakClient(ADDR) as client:
        await client.write_gatt_char(CHAR, b"01234567")
        await asyncio.sleep(1)
        await client.write_gatt_char(CHAR, b"1111111111")
        data = await client.read_gatt_char(CHAR)
        print("Current:", data.decode(errors="ignore"))

asyncio.run(demo())
```

## 硬件连接

```
ESP32-C3 GPIO1  ->  WS2812 DIN
ESP32-C3 5V     ->  WS2812 VCC
ESP32-C3 GND    ->  WS2812 GND
```

建议使用稳定 5V 供电并确保共地，灯带共 60 颗。

## 常见问题

- **未解析**：写入必须是纯数字字符串（0~7），不接受空格或逗号
- **只亮部分**：检查字符串长度；不足 60 时其余灯会熄灭
- **设备不可见**：重启开发板或扫描器，确保串口日志无错误

## 更新日志

### v1.1.0 (2025-11-26)
- 切换为 60 颗 LED、字符串控制
- 精简 BLE/LED 驱动与说明文档

### v1.0.0 (2025-11-25)
- 初始版本：32 位整型控制、WS2812 输出
