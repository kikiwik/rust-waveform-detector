use anyhow::Result;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::Text,
};
use esp_idf_svc::hal::adc::oneshot::{config::AdcChannelConfig, AdcChannelDriver, AdcDriver};
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::gpio::*;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::spi::{self, SpiDeviceDriver, SpiDriverConfig};
// FFT相关
use libm::sqrtf;
use microfft::real::rfft_256;
// --- ADC 相关导入 ---
use ssd1306::{
    mode::BufferedGraphicsMode, prelude::*, rotation::DisplayRotation, size::DisplaySize128x64,
    Ssd1306,
};
use display_interface_spi::SPIInterface;

// WaveformType Enum
#[derive(Clone, Copy, PartialEq)]
enum WaveformType {
    Sine,
    Square,
    Triangle,
    Unknown,
}

impl WaveformType {
    fn as_str(&self) -> &str {
        match self {
            WaveformType::Sine => "Sine",
            WaveformType::Square => "Square",
            WaveformType::Triangle => "Triangle",
            WaveformType::Unknown => "Unknown",
        }
    }
}

// WaveformResult Struct
struct WaveformResult {
    waveform_type: WaveformType,
    frequency: f32,
    amplitude: f32,
}

// FFTWaveformAnalyzer Struct and impl
struct FFTWaveformAnalyzer {
    sample_buffer: [f32; 256],
    sample_count: usize,
    sample_rate: u32,
}

impl FFTWaveformAnalyzer {
    fn new(sample_rate: u32) -> Self {
        Self {
            sample_buffer: [0.0; 256],
            sample_count: 0,
            sample_rate,
        }
    }

    fn add_sample(&mut self, adc_value: u16) {
        if self.sample_count < self.sample_buffer.len() {
            let voltage = (adc_value as f32 / 4095.0) * 2.5;
            self.sample_buffer[self.sample_count] = voltage;
            self.sample_count += 1;
        }
    }

    fn reset(&mut self) {
        self.sample_count = 0;
    }

    fn is_full(&self) -> bool {
        self.sample_count >= self.sample_buffer.len()
    }

    fn analyze(&mut self) -> WaveformResult {
        if self.sample_count < 256 {
            return WaveformResult {
                waveform_type: WaveformType::Unknown,
                frequency: 0.0,
                amplitude: 0.0,
            };
        }

        let dc_offset = self.remove_dc_offset();
        self.apply_hanning_window();
        let spectrum = self.perform_fft();
        let (fundamental_freq, _fundamental_magnitude) = self.find_fundamental_frequency(&spectrum);
        let harmonics = self.analyze_harmonics(&spectrum, fundamental_freq);
        let waveform_type = self.classify_waveform(&harmonics);
        let amplitude = self.calculate_amplitude(dc_offset);

        WaveformResult {
            waveform_type,
            frequency: fundamental_freq / 1000.0,
            amplitude,
        }
    }

    fn remove_dc_offset(&mut self) -> f32 {
        let sum: f32 = self.sample_buffer[..self.sample_count].iter().sum();
        let dc_offset = sum / self.sample_count as f32;
        for sample in self.sample_buffer[..self.sample_count].iter_mut() {
            *sample -= dc_offset;
        }
        dc_offset
    }

    fn apply_hanning_window(&mut self) {
        use core::f32::consts::PI;
        for i in 0..self.sample_count {
            let window =
                0.5 * (1.0 - libm::cosf(2.0 * PI * i as f32 / (self.sample_count - 1) as f32));
            self.sample_buffer[i] *= window;
        }
    }

    // ▼▼▼ 修改这里 ▼▼▼
    fn perform_fft(&mut self) -> [f32; 128] {
        let fft_result = rfft_256(&mut self.sample_buffer);
        // ▼▼▼ 修改这里 ▼▼▼
        let mut spectrum = [0.0f32; 128];
        // ▼▼▼ 修改这里 ▼▼▼
        for i in 0..128 {
            let val = fft_result[i];
            spectrum[i] = sqrtf(val.re * val.re + val.im * val.im);
        }
        spectrum
    }

    fn find_fundamental_frequency(&self, spectrum: &[f32; 128]) -> (f32, f32) {
        let freq_resolution = self.sample_rate as f32 / 256.0;

        // --- 1. 使用 spectrum.len() 来确定边界 ---
        let last_index = spectrum.len() - 1; // 数组的最后一个有效索引 (128)

        let min_bin = (20000.0 / freq_resolution).round() as usize;
        let max_bin = (100000.0 / freq_resolution).round() as usize;

        // 搜索范围不能超过数组的最后一个索引
        let search_end = max_bin.min(last_index);

        let mut max_magnitude = 0.0;
        let mut max_bin_index = min_bin;
        // 确保 min_bin 不会一开始就越界
        for i in min_bin..=search_end {
            if spectrum[i] > max_magnitude {
                max_magnitude = spectrum[i];
                max_bin_index = i;
            }
        }

        // --- 2. 使用一个更清晰的条件来进行插值计算 ---
        // 条件：峰值索引必须大于0且小于最后一个索引，这样才能安全地访问-1和+1
        let freq = if max_bin_index > 0 && max_bin_index < last_index {
            let alpha = spectrum[max_bin_index - 1];
            let beta = spectrum[max_bin_index];
            let gamma = spectrum[max_bin_index + 1];

            // 避免除以零
            if (alpha - 2.0 * beta + gamma).abs() < 1e-6 {
                max_bin_index as f32 * freq_resolution
            } else {
                let delta = 0.5 * (alpha - gamma) / (alpha - 2.0 * beta + gamma);
                (max_bin_index as f32 + delta) * freq_resolution
            }
        } else {
            // 如果峰值在数组的边界上（第一个或最后一个），则不进行插值
            max_bin_index as f32 * freq_resolution
        };

        (freq, max_magnitude)
    }

    // ▼▼▼ 修改函数签名这里的类型 ▼▼▼
    fn analyze_harmonics(&self, spectrum: &[f32; 128], fundamental: f32) -> [f32; 5] {
        let freq_resolution = self.sample_rate as f32 / 256.0;
        let mut harmonics = [0.0f32; 5];
        for i in 1..=5 {
            let harmonic_freq = fundamental * i as f32;
            let harmonic_bin = (harmonic_freq / freq_resolution).round() as usize;
            // ▼▼▼ 修改这里的边界检查 ▼▼▼
            if harmonic_bin < 128 {
                harmonics[i - 1] = spectrum[harmonic_bin];
            }
        }
        let fundamental_magnitude = harmonics[0];
        if fundamental_magnitude > 1e-6 {
            for h in harmonics.iter_mut() {
                *h /= fundamental_magnitude;
            }
        }
        harmonics
    }

    fn classify_waveform(&self, harmonics: &[f32; 5]) -> WaveformType {
        let h1 = harmonics[0];
        let h2 = harmonics[1];
        let h3 = harmonics[2];
        let h5 = harmonics[4];
        let thd = sqrtf(h2 * h2 + h3 * h3 + h5 * h5);
        log::info!(
            "Harmonics: 1={:.3}, 2={:.3}, 3={:.3}, 5={:.3}, THD={:.3}",
            h1,
            h2,
            h3,
            h5,
            thd
        );
        if thd < 0.1 {
            WaveformType::Sine
        } else if h3 > 0.15 && h5 > 0.05 {
            if h3 > 0.25 {
                WaveformType::Square
            } else {
                WaveformType::Triangle
            }
        } else if h2 < 0.1 && h3 > 0.1 {
            WaveformType::Square
        } else {
            WaveformType::Unknown
        }
    }

    fn calculate_amplitude(&self, dc_offset: f32) -> f32 {
        let mut max_val = -f32::MAX;
        let mut min_val = f32::MAX;
        for i in 0..self.sample_count {
            let val = self.sample_buffer[i] + dc_offset;
            if val > max_val {
                max_val = val;
            }
            if val < min_val {
                min_val = val;
            }
        }
        let vpp = max_val - min_val;
        if min_val < 0.1 { vpp * 2.0 } else { vpp }
    }
}

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("ESP32-S3 FFT Wave Detector Starting...");

    let peripherals = Peripherals::take().unwrap();

    // --- Display Init ---
    let sclk = peripherals.pins.gpio7;
    let mosi = peripherals.pins.gpio6;
    let dc = PinDriver::output(peripherals.pins.gpio4)?;
    let mut rst = PinDriver::output(peripherals.pins.gpio5)?;
    let cs = PinDriver::output(peripherals.pins.gpio10)?;

    let spi_config = spi::config::Config::new()
        .baudrate(10.MHz().into())
        .data_mode(spi::config::Mode {
            polarity: spi::config::Polarity::IdleLow,
            phase: spi::config::Phase::CaptureOnFirstTransition,
        });

    let spi = SpiDeviceDriver::new_single(
        peripherals.spi2,
        sclk,
        mosi,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &SpiDriverConfig::new(),
        &spi_config,
    )?;

    let interface = SPIInterface::new(spi, dc, cs);

    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    rst.set_high()?;
    FreeRtos::delay_ms(10);
    rst.set_low()?;
    FreeRtos::delay_ms(10);
    rst.set_high()?;
    FreeRtos::delay_ms(10);

    display.init().unwrap();
    log::info!("Display initialized!");

    show_boot_animation(&mut display);

    // --- ADC Init ---
    log::info!("Initializing ADC...");
    let adc = AdcDriver::new(peripherals.adc1)?;

    // 使用默认配置
    let adc_config = AdcChannelConfig::default();
    let mut adc_channel = AdcChannelDriver::new(&adc, peripherals.pins.gpio1, &adc_config)?;

    log::info!("ADC initialized on GPIO1!");

    let sample_rate = 256_000;
    let mut analyzer = FFTWaveformAnalyzer::new(sample_rate);
    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    log::info!("Starting FFT analysis loop...");
    log::info!("Sample rate: {} Hz, FFT size: 256", sample_rate);

    // --- Main Loop ---
    loop {
        analyzer.reset();
        let sample_interval_us = 1_000_000 / sample_rate;

        while !analyzer.is_full() {
            match adc.read(&mut adc_channel) {
                Ok(value) => analyzer.add_sample(value),
                Err(e) => log::error!("ADC read error: {:?}", e),
            }
            unsafe { esp_idf_svc::sys::esp_rom_delay_us(sample_interval_us) };
        }

        let result = analyzer.analyze();
        display.clear(BinaryColor::Off).unwrap();

        let mut line0 = heapless::String::<32>::new();
        let mut line1 = heapless::String::<32>::new();
        let mut line2 = heapless::String::<32>::new();
        let mut line3 = heapless::String::<32>::new();

        use core::fmt::Write;
        write!(line0, "FFT Analyzer").unwrap();
        write!(line1, "Wave: {}", result.waveform_type.as_str()).unwrap();
        write!(line2, "Freq: {:.1} kHz", result.frequency).unwrap();
        write!(line3, "Amp:  {:.2} V", result.amplitude).unwrap();

        Text::new(&line0, Point::new(5, 10), text_style)
            .draw(&mut display)
            .unwrap();
        Text::new(&line1, Point::new(5, 25), text_style)
            .draw(&mut display)
            .unwrap();
        Text::new(&line2, Point::new(5, 40), text_style)
            .draw(&mut display)
            .unwrap();
        Text::new(&line3, Point::new(5, 55), text_style)
            .draw(&mut display)
            .unwrap();

        display.flush().unwrap();

        log::info!(
            "[FFT] {} | {:.2} kHz | {:.2} Vpp",
            result.waveform_type.as_str(),
            result.frequency,
            result.amplitude
        );

        FreeRtos::delay_ms(300);
    }
}

fn show_boot_animation<DI>(
    display: &mut Ssd1306<DI, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>,
) where
    DI: display_interface::WriteOnlyDataCommand,
{
    display.clear(BinaryColor::Off).unwrap();
    let bar_width: u32 = 100;
    let bar_height: u32 = 10;
    let bar_x = (128 - bar_width as i32) / 2;
    let bar_y = (64 - bar_height as i32) / 2;
    let border_style = PrimitiveStyleBuilder::new()
        .stroke_color(BinaryColor::On)
        .stroke_width(1)
        .build();
    Rectangle::new(
        Point::new(bar_x, bar_y),
        Size::new(bar_width, bar_height),
    )
    .into_styled(border_style)
    .draw(display)
    .unwrap();
    display.flush().unwrap();
    FreeRtos::delay_ms(200);
    let fill_style = PrimitiveStyleBuilder::new()
        .fill_color(BinaryColor::On)
        .build();
    for i in 1..=bar_width {
        Rectangle::new(Point::new(bar_x, bar_y), Size::new(i, bar_height))
            .into_styled(fill_style)
            .draw(display)
            .unwrap();
        display.flush().unwrap();
        FreeRtos::delay_ms(10);
    }
    FreeRtos::delay_ms(500);
}