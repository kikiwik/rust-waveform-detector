# Rust 波形检测器 (rust-wave-detector)

这是一个基于 Rust 和 ESP-IDF 框架，运行在 **ESP32-S3** 上的嵌入式项目。项目需求来源是一个电赛的题目。

它通过 ADC 采集模拟信号，利用快速傅里叶变换 (FFT) 对信号进行实时分析，并将识别出的波形类型（正弦波、方波、三角波）、频率和幅度（峰峰值）显示在一个 SSD1306 OLED 屏幕上。

## 主要功能

  * **实时信号采样**: 使用 ADC 以 256 kHz 的采样率采集信号。
  * **FFT 频谱分析**:
      * 采集 256 个样本点。
      * 应用汉宁窗 (Hanning Window) 以减少频谱泄漏。
      * 使用 `microfft::real::rfft_256` 进行实数 FFT。
  * **波形分类**:
      * 分析基波和奇次谐波（最高到5次）的能量分布。
      * 根据谐波特征将波形分类为正弦波 (Sine)、方波 (Square) 或三角波 (Triangle)。
  * **参数测量**:
      * 计算信号的基波频率（kHz）。
      * 计算信号的幅度（Vpp）。
  * **OLED 显示**: 使用 `ssd1306` 库将分析结果实时显示在 128x64 分辨率的 OLED 屏幕上。
  * **日志输出**: 通过串行端口输出详细的分析日志。

## 硬件要求

  * **MCU**: ESP32-S3 (根据 `.cargo/config.toml` 配置)。
  * **显示屏**: SSD1306 128x64 OLED (SPI 接口)。
  * **信号输入**: 一个模拟信号源（例如函数发生器）。

### 引脚连接 (Pinout)

根据 `src/main.rs` 中的配置，引脚连接如下：

| 功能 | ESP32-S3 引脚 |
| :--- | :--- |
| **ADC 信号输入** | `GPIO 1` |
| **OLED (SPI)** | |
| SCLK (时钟) | `GPIO 7` |
| MOSI (数据) | `GPIO 6` |
| DC (数据/命令) | `GPIO 4` |
| RST (复位) | `GPIO 5` |
| CS (片选) | `GPIO 10` |

## 软件与环境配置

本项目使用 Rust 的 `esp-idf` 框架。

  * **Rust 工具链**: `esp` (推荐使用 `esp-rs/rust-build` 进行安装)。
  * **Rust 版本**: 1.77+。
  * **ESP-IDF 版本**: v5.3.3 (在 `.cargo/config.toml` 中指定)。
  * **目标平台**: `xtensa-esp32s3-espidf`。

## 如何构建和运行

### 1\. 安装 ESP Rust 环境

请确保您已根据 [esp-rs 指南](https://www.google.com/search?q=https://esp-rs.github.io/book/installation/index.html) 正确安装了 Rust `esp` 工具链和 `ldproxy`。

### 2\. 克隆项目

```bash
git clone <your-repo-url>
cd rust-wave-detector
```

### 3\. 构建

```bash
cargo build --release
```

### 4\. 刷写与监视

连接您的 ESP32-S3 开发板，然后运行：

```bash
cargo run --release
```

该命令将自动编译、刷写固件，并启动串行监视器。您将在监视器中看到日志输出。

```bash
# .cargo/config.toml 中定义的 runner
runner = "espflash flash --monitor"
```

## 项目依赖

本项目的主要依赖库包括：

  * `esp-idf-svc`: ESP-IDF 服务的 Rust 抽象层。
  * `esp-idf-hal`: ESP32-S3 的硬件抽象层 (HAL)。
  * `embedded-graphics`, `ssd1306`: 用于 OLED 显示。
  * `microfft`: 用于在 `no_std` 环境下进行 FFT 计算。
  * `heapless`: 用于无堆栈分配的数据结构（如字符串）。
  * `embuild`: 用于辅助 ESP-IDF 构建。
## 项目进展
- 点亮亮屏
- 添加初始化动画功能
- adc采样（基于freertos的adc_read,非连续采样模式）
- 基本实现频域波形分析功能
## todo
- 频域分析精度不高，波形识别函数有缺陷，10kHz以上波形识别不准确
- 采样率还未达到设想的20kHZ～100kHZ标准。rust没有封装好的adc连续采样库，还在考虑解决办法
- 信号输出功能
