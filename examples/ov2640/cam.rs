use esp_idf_hal::delay::Ets;
use esp_idf_hal::gpio;
use esp_idf_hal::i2c;
use esp_idf_hal::i2s;
use esp_idf_hal::ledc::*;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::prelude::*;

use esp_idf_sys::EspError;

use crate::sensor::*;

pub struct EspCamera {}

impl EspCamera {
    pub fn init(
        sda: gpio::AnyIOPin,
        scl: gpio::AnyIOPin,
        i2c: impl Peripheral<P = impl i2c::I2c> + 'static,
        pwdn: gpio::AnyIOPin,
        //reset: gpio::AnyIOPin,
        xclk: gpio::AnyOutputPin,
        ledc_timer: TIMER0,
        ledc_channel: CHANNEL0,
        i2s: i2s::I2S0,
        vsync: gpio::AnyIOPin,
        href: gpio::AnyIOPin,
        pclk: gpio::AnyIOPin,
        sd0: gpio::AnyInputPin,
        sd1: gpio::AnyInputPin,
        sd2: gpio::AnyInputPin,
        sd3: gpio::AnyInputPin,
        sd4: gpio::AnyInputPin,
        sd5: gpio::AnyInputPin,
        sd6: gpio::AnyInputPin,
        sd7: gpio::AnyInputPin,
        xclk_freq: Hertz,
        pixel_format: PixFormat,
        frame_size: FrameSize,
        jpeg_quality: u8,
        fb_count: usize,
        //fb_location,
        //grab_mode,
    ) -> Result<i2s::CameraDriver<'static>, EspError> {
        println!("Configuring LEDC output channel");

        let ledc_timer_config = config::TimerConfig::new()
            .frequency(xclk_freq)
            .speed_mode(SpeedMode::LowSpeed)
            .resolution(Resolution::Bits1);

        let mut channel = LedcDriver::new(
            ledc_channel,
            LedcTimerDriver::new(ledc_timer, &ledc_timer_config)?,
            xclk,
        )?;

        let max_duty = channel.get_max_duty();
        channel.set_duty(max_duty)?;

        println!("Starting I2C OV2640 test");

        let mut i2c0 = i2c::I2cDriver::new(
            i2c,
            sda,
            scl,
            &i2c::I2cConfig::new().baudrate(8.kHz().into()),
        )?;
        let mut pwdn = gpio::PinDriver::output(pwdn)?;
        pwdn.set_low()?; // power-up = low

        // SCCB_Probe
        /*
        Ov2640Pid = 0x26,
        OV2640_SCCB_ADDR   = 0x30
        */
        let slave_addr = 0x30;
        let ov2640_pid: u8 = 0x26;
        let byte1: &mut [u8; 1] = &mut [0_u8];
        let byte2: &mut [u8; 1] = &mut [0_u8];
        let i2c_timeout: u32 = 1000;

        i2c0.write(slave_addr, &[0xff, 0x01], i2c_timeout);
        let i2c_res1 = i2c0.write(slave_addr, &[0x12, 0x80], i2c_timeout);
        //        println!("write -> {:#?}", i2c_res1);
        Ets::delay_ms(10 as u32);
        i2c0.write_read(slave_addr, &[0x0a], byte1, i2c_timeout);
        let i2c_res = i2c0.write_read(slave_addr, &[0x0b], byte2, i2c_timeout);
        println!(
            "return command: PID={} VER={} -> {:#?}",
            byte1[0], byte2[0], i2c_res
        );

        if ov2640_pid == byte1[0] {
            println!("OV2640 found");
        } else {
            println!("OV2640 mismatch");
        }

        println!("Start I2S config");
        //// ll_cam_set_pin
        // I2S init
        let i2s_config = i2s::config::Config {
            dma_buffer: Some(&[0_u8; 20]),
            vsync_invert: false,
            vsync_isr: None, // Todo: compiles, but will crash due to missing isr!
            dma_isr: None,
        };
        let mut cam_slave = i2s::CameraDriver::new(
            i2s, vsync, href, pclk, sd0, sd1, sd2, sd3, sd4, sd5, sd6, sd7, i2s_config,
        )
        .unwrap();

        //// ll_cam_config
        // reset
        //  i2sregs.set_conf(0x0000_000a); // rx_rst, rx_fifo_rst
        //  i2sregs.set_conf(0x0000_0000);
        //  i2sregs.set_lcConf(0x0000_000d);
        //  i2sregs.set_lcConf(0);

        // conf slave mode
        //  i2sregs.set_conf(0x0000_0080);
        //  i2sregs.set_conf2(0x0000_0021);
        //  i2sregs.set_cklmConf(0x0000_0002);
        //  i2sregs.set_fifoConf(0x0013_1410); // sampling mode = 0A00_0B00 = 3
        //  i2sregs.set_confChan(0x0000_0004);
        //  i2sregs.set_sampleRateConf(0x0001_0186);
        //  i2sregs.set_timing(0x0020_0000);

        //// ll_cam_start

        //// re-read memory
        unsafe {
            let mem = &mut *(0x3FF4_F000 as *mut [u32; 48]);
            let mut addr = 0_u16;

            println!("===================================================");
            for reg in mem {
                println!("mem(0x3FF4_F{:03X}): {:#010x}", addr, reg);
                addr = addr + 4;
            }
            println!("===================================================");
        }

        Ok(cam_slave)
    }

    pub fn fb_get() {
        todo!()
    }

    pub fn fb_return() {
        todo!()
    }

    /*
    Missing funcs:
    - sensor_get
    - save_to_nvs,
    - load_from_nvs
    - return_all
     */
}

impl Drop for EspCamera {
    fn drop(&mut self) {
        todo!()
    }
}
