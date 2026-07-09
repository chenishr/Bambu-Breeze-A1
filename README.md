# Bambu-Breeze-A1 (拓竹 A1 智能封箱排风排烟控制器)

Bambu-Breeze-A1 是一款基于 **ESP32-C3** 芯片与 **ESP-IDF** 框架开发的智能打印机封箱排风控制器。它能够通过局域网安全加密的 MQTT (MQTTS) 协议实时订阅拓竹 Bambu Lab A1 打印机的工作状态，并利用 ESP32-C3 的高频 PWM (LEDC) 功能，依据当前打印的耗材类型动态调整 12V 机箱排风风扇的转速，以达到合理调控舱内温度、减少异味与排烟的目的。

---

## 🌟 核心特性

- 📶 **智能无线连接**：支持 Wi-Fi 自动连接及断线后指数级退避重连机制。
- 🔒 **安全局域网 MQTT (MQTTS)**：使用 `esp-mqtt` 客户端，通过 `8883` 端口同打印机建立加密连接，配置自动忽略自签证书。
- 🔄 **开机全量同步**：连接成功后，主控会自动向打印机发送 `pushall` 请求以立刻获取完整打印机状态，实现开机即同步。
- 📦 **稳健的 JSON 解析**：使用 `cJSON` 解析器处理拓竹复杂的嵌套状态，能准确识别当前活跃料位（兼容 AMS Lite 多槽位选择以及外挂虚拟挂架 `vt_tray`），即使收到高频的温度增量包也能持久维持工作状态。
- ❄️ **耗材联动控温**：
  - **PLA** 材质：100% 占空比（风扇全速运行，降低封箱温度防止堵头）。
  - **PETG** 材质：33% 占空比（风扇低速温和排风）。
  - **ABS / ASA** 材质：0% 占空比（风扇关闭，维持箱体内部高温以防止翘边）。
  - **其他材质**：50% 占空比（默认安全速度）。
- ⏱️ **延迟关机保护**：打印任务进入 `FINISH`（完成）或 `IDLE`（空闲）状态后，触发一个 5 分钟（300秒）的 FreeRTOS 软件定时器。若 5 分钟内无新任务则彻底关闭风扇，防止热风和烟气瞬间散出，同时避免无谓的功耗。
- ⚡ **高频静音 PWM 驱动**：使用 LEDC 驱动器在 `GPIO 5` 输出 20kHz 频率、8位分辨率（0-255）的低频噪声控制信号，非常适合驱动 12V 风扇的 MOS 管驱动板。

---

## 🔌 硬件接线指南

本方案使用 ESP32-C3 的 GPIO 5 引脚输出 PWM 调速信号。由于 12V 风扇功率较大，必须通过 **MOS 管驱动模块（如 LR7843 或 D4184 模块）** 或三极管电路进行隔离驱动：

```text
    +-------------------+        +---------------------+
    |                   |        |   MOS管驱动模块      |
    |     ESP32-C3      |        |                     |
    |                   |        |                     |
    |      GPIO 5  -----+--------> PWM (控制信号)      |
    |      GND     -----+--------> GND (共地)          |
    |                   |        |                     |
    +-------------------+        |  V+ (输入) <========  12V电源正极
                                 |  V- (输入) <========  12V电源负极
                                 |                     |
                                 |  OUT+ (输出) =======> 12V风扇正极
                                 |  OUT- (输出) =======> 12V风扇负极
                                 +---------------------+
```

> [!IMPORTANT]
> ESP32-C3 的 GND、MOS 模块的 GND 以及 12V 供电电源的负极**必须共地**，否则 PWM 调速信号将无法正常工作。

---

## 🛠️ 项目文件结构

- [main.c](file:///Users/admin/work/test/esp32/Bambu-Breeze-A1/main/main.c)：存放了所有业务逻辑，包含 Wi-Fi 初始化、MQTT 客户端创建、数据解析、软件定时器和 LEDC 初始化。
- [sdkconfig.defaults](file:///Users/admin/work/test/esp32/Bambu-Breeze-A1/sdkconfig.defaults)：配置了 ESP-TLS 的参数，允许连接至自签名证书的局域网 MQTTS 代理。
- [main/CMakeLists.txt](file:///Users/admin/work/test/esp32/Bambu-Breeze-A1/main/CMakeLists.txt)：声明了 `json`、`mqtt`、`esp_wifi`、`driver` 等核心依赖组件。
- [CMakeLists.txt](file:///Users/admin/work/test/esp32/Bambu-Breeze-A1/CMakeLists.txt)：项目级构建脚本。

---

## ⚙️ 软件配置参数

在烧录之前，请根据您家中的局域网环境以及打印机信息，修改 [main.c](file:///Users/admin/work/test/esp32/Bambu-Breeze-A1/main/main.c) 文件头部的宏定义：

```c
// Wi-Fi 配置信息
#define WIFI_SSID           "ChinaNet-Pxp6"    // 您的 Wi-Fi 名称
#define WIFI_PASS           "an6ughx2"          // 您的 Wi-Fi 密码

// 拓竹 A1 打印机连接参数
#define BAMBU_PRINTER_IP      "192.168.1.54"      // 打印机局域网 IP
#define BAMBU_ACCESS_CODE     "bb96681d"          // 打印机的 Access Code (在打印机屏幕设置中获取)
#define BAMBU_PRINTER_SERIAL  "03919D520902254"   // 打印机序列号 (用于拼装精准的 MQTT 订阅主题)
```

---

## 🚀 编译与烧录

本项目基于 **ESP-IDF v5.x** 进行开发。

### 1. 安装环境（首次运行）
如果您的电脑中尚未配置该版本的 ESP-IDF 的 Python 虚拟环境与工具链，需要先运行安装脚本进行初始化：
```bash
# 运行安装脚本（建议指定 esp32 以节省空间和下载时间）
~/.espressif/v5.5.4/esp-idf/install.sh esp32
```

### 2. 载入 ESP-IDF 工具链环境变量
在当前终端中执行以下命令，加载编译所需的环境变量（请根据您的实际安装路径进行微调）：
```bash
. ~/.espressif/v5.5.4/esp-idf/export.sh
```

### 3. 编译项目
在项目根目录运行以下命令进行全量编译：
```bash
idf.py build
```

### 4. 烧录固件并开启串口监视器
将 ESP32-C3 主控板通过 USB 接口连接至电脑，运行以下命令（请根据您的实际串口号进行替换，例如 `/dev/tty.usbmodem14201`）：
```bash
idf.py -p /dev/tty.usbmodem14201 flash monitor
```

> [!TIP]
> 进入 Monitor 界面后，如需退出，可使用键盘快捷键 `Ctrl + ]`。
