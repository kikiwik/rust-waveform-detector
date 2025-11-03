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
// 定时器相关
use esp_idf_svc::hal::timer::{TimerDriver, TimerConfig};
// 同步原语
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
// FFT相关
use libm::sqrtf;
use microfft::real::rfft_256;
use ssd1306::{
    mode::BufferedGraphicsMode, prelude::*, rotation::DisplayRotation, size::DisplaySize128x64,
    Ssd1306,
};
use display_interface_spi::SPIInterface;

// === 全局静态变量 ===
// 双缓冲区
static mut BUFFER_A: [u16; 256] = [0; 256];
static mut BUFFER_B: [u16; 256] = [0; 256];

// 当前使用哪个缓冲区进行采样 (false=A, true=B)
static ACTIVE_BUFFER: AtomicBool = AtomicBool::new(false);

// 当前采样计数
static SAMPLE_INDEX: AtomicUsize = AtomicUsize::new(0);

// 缓冲区是否已满,可供分析
static BUFFER_READY: AtomicBool = AtomicBool::new(false);

// ADC全局指针（用于ISR访问）
use core::ptr::null_mut;
static mut ADC_DRIVER_PTR: *mut AdcDriver<'static, esp_idf_svc::hal::adc::ADC1> = null_mut();
static mut ADC_CHANNEL_PTR: *mut AdcChannelDriver<'static, esp_idf_svc::hal::gpio::Gpio1, &'static AdcDriver<'static, esp_idf_svc::hal::adc::ADC1>> = null_mut();

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
    sample_rate: u32,
}

impl FFTWaveformAnalyzer {
    fn new(sample_rate: u32) -> Self {
        Self {
            sample_buffer: [0.0; 256],
            sample_rate,
        }
    }

    // 从原始ADC缓冲区加载数据
    fn load_from_buffer(&mut self, buffer: &[u16; 256]) {
        for (i, &adc_value) in buffer.iter().enumerate() {
            let voltage = (adc_value as f32 / 4095.0) * 2.5;
            self.sample_buffer[i] = voltage;
        }
    }

    fn analyze(&mut self) -> WaveformResult {
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
        let sum: f32 = self.sample_buffer.iter().sum();
        let dc_offset = sum / 256.0;
        for sample in self.sample_buffer.iter_mut() {
            *sample -= dc_offset;
        }
        dc_offset
    }

    fn apply_hanning_window(&mut self) {
        use core::f32::consts::PI;
        for i in 0..256 {
            let window = 0.5 * (1.0 - libm::cosf(2.0 * PI * i as f32 / 255.0));
            self.sample_buffer[i] *= window;
        }
    }

    fn perform_fft(&mut self) -> [f32; 128] {
        let fft_result = rfft_256(&mut self.sample_buffer);
        let mut spectrum = [0.0f32; 128];

        for i in 0..128 {
            let val = fft_result[i];
            spectrum[i] = sqrtf(val.re * val.re + val.im * val.im);
        }
        spectrum
    }

    fn find_fundamental_frequency(&self, spectrum: &[f32; 128]) -> (f32, f32) {
        let freq_resolution = self.sample_rate as f32 / 256.0;
        let last_index = spectrum.len() - 1;

        let min_bin = (100.0 / freq_resolution).round() as usize;  // 降低最小频率到100Hz
        let max_bin = (20000.0 / freq_resolution).round() as usize; // 最大20kHz
        let search_end = max_bin.min(last_index);

        let mut max_magnitude = 0.0;
        let mut max_bin_index = min_bin;
        
        for i in min_bin..=search_end {
            if spectrum[i] > max_magnitude {
                max_magnitude = spectrum[i];
                max_bin_index = i;
            }
        }

        let freq = if max_bin_index > 0 && max_bin_index < last_index {
            let alpha = spectrum[max_bin_index - 1];
            let beta = spectrum[max_bin_index];
            let gamma = spectrum[max_bin_index + 1];

            if (alpha - 2.0 * beta + gamma).abs() < 1e-6 {
                max_bin_index as f32 * freq_resolution
            } else {
                let delta = 0.5 * (alpha - gamma) / (alpha - 2.0 * beta + gamma);
                (max_bin_index as f32 + delta) * freq_resolution
            }
        } else {
            max_bin_index as f32 * freq_resolution
        };

        (freq, max_magnitude)
    }

    fn analyze_harmonics(&self, spectrum: &[f32; 128], fundamental: f32) -> [f32; 5] {
        let freq_resolution = self.sample_rate as f32 / 256.0;
        let mut harmonics = [0.0f32; 5];
        
        for i in 1..=5 {
            let harmonic_freq = fundamental * i as f32;
            let harmonic_bin = (harmonic_freq / freq_resolution).round() as usize;
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
            h1, h2, h3, h5, thd
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
        
        for &val in self.sample_buffer.iter() {
            let actual_val = val + dc_offset;
            if actual_val > max_val {
                max_val = actual_val;
            }
            if actual_val < min_val {
                min_val = actual_val;
            }
        }
        
        let vpp = max_val - min_val;
        if min_val < 0.1 { 
            vpp * 2.0 
        } else { 
            vpp 
        }
    }
}

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("ESP32-S3 FFT Wave Detector Starting (Timer ISR Mode)...");

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
    
    // 将ADC驱动转换为裸指针
    let adc_ptr = Box::into_raw(Box::new(adc));
    
    // 创建ADC通道时需要引用ADC驱动
    let adc_config = AdcChannelConfig::default();
    let adc_channel = unsafe {
        AdcChannelDriver::new(&*adc_ptr, peripherals.pins.gpio1, &adc_config)?
    };
    
    // 存储指针供ISR使用
    unsafe {
        ADC_DRIVER_PTR = adc_ptr;
        ADC_CHANNEL_PTR = Box::into_raw(Box::new(adc_channel));
    }

    log::info!("ADC initialized on GPIO1!");

    // --- 定时器设置 ---
    let sample_rate = 40_000u32; // 40 kHz采样率
    let timer_config = TimerConfig::new();
    let mut timer = TimerDriver::new(peripherals.timer00, &timer_config)?;
    
    // 设置定时器周期 (单位: 微秒)
    // 40kHz = 25us per sample
    let timer_period_us = 1_000_000u64 / sample_rate as u64;
    
    timer.set_alarm(timer_period_us)?;
    timer.enable_alarm(true)?;
    timer.enable_interrupt()?;
    
    // 订阅定时器中断
    unsafe {
        timer.subscribe(timer_isr_callback)?;
    }
    
    timer.enable(true)?;
    
    log::info!("Timer initialized! Sample rate: {} Hz (period: {} us)", 
               sample_rate, timer_period_us);

    let mut analyzer = FFTWaveformAnalyzer::new(sample_rate);
    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    log::info!("Starting FFT analysis loop...");

    // --- Main Loop ---
    loop {
        // 等待缓冲区准备好
        while !BUFFER_READY.load(Ordering::Acquire) {
            FreeRtos::delay_ms(1);
        }
        
        // 读取完整的缓冲区
        // 双缓冲机制确保了ISR写活动缓冲区时，主线程读非活动缓冲区
        // 原子操作保证了标志位的正确切换，无需额外的临界区保护
        let buffer_to_analyze: [u16; 256];
        unsafe {
            // 读取非活动缓冲区(已填充完成的那个)
            if ACTIVE_BUFFER.load(Ordering::Acquire) {
                // 当前在用B,读取A
                buffer_to_analyze = BUFFER_A;
            } else {
                // 当前在用A,读取B
                buffer_to_analyze = BUFFER_B;
            }
        }
        
        // 重置ready标志，允许下一次采样
        BUFFER_READY.store(false, Ordering::Release);

        // 分析数据
        analyzer.load_from_buffer(&buffer_to_analyze);
        let result = analyzer.analyze();
        
        // 更新显示
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
    }
}

// === 定时器中断服务程序 ===
fn timer_isr_callback() {
    unsafe {
        // 检查指针是否有效
        if ADC_DRIVER_PTR.is_null() || ADC_CHANNEL_PTR.is_null() {
            return;
        }
        
        // 通过裸指针访问ADC（避免借用检查问题）
        let adc_driver = &*ADC_DRIVER_PTR;
        let adc_channel = &mut *ADC_CHANNEL_PTR;
        
        if let Ok(value) = adc_driver.read(adc_channel) {
            let idx = SAMPLE_INDEX.load(Ordering::Relaxed);
            
            if idx < 256 {
                // 写入当前活动缓冲区
                if ACTIVE_BUFFER.load(Ordering::Relaxed) {
                    BUFFER_B[idx] = value;
                } else {
                    BUFFER_A[idx] = value;
                }
                
                SAMPLE_INDEX.store(idx + 1, Ordering::Relaxed);
                
                // 缓冲区满了
                if idx + 1 >= 256 {
                    // 切换缓冲区
                    ACTIVE_BUFFER.fetch_xor(true, Ordering::Release);
                    SAMPLE_INDEX.store(0, Ordering::Relaxed);
                    BUFFER_READY.store(true, Ordering::Release);
                }
            }
        }
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