use esp_idf_sys::EspError;

#[cfg(all(esp32, esp_idf_version_major = "4"))]
pub mod camera {
    use esp_idf_hal::delay;
    use esp_idf_hal::delay::Ets;
    use esp_idf_hal::gpio;
    use esp_idf_hal::i2c;
    use esp_idf_hal::i2s;
    use esp_idf_hal::ledc::*;
    use esp_idf_hal::peripheral::Peripheral;
    use esp_idf_hal::prelude::*;

    use esp_idf_sys::EspError;

    pub enum FrameSize {
        Framesize96x96,   // 96x96
        FramesizeQqvga,   // 160x120
        FramesizeQcif,    // 176x144
        FramesizeHqvga,   // 240x176
        Framesize240x240, // 240x240
        FramesizeQvga,    // 320x240
        FramesizeCif,     // 400x296
        FramesizeHvga,    // 480x320
        FramesizeVga,     // 640x480
        FramesizeSvga,    // 800x600
        FramesizeXga,     // 1024x768
        FramesizeHd,      // 1280x720
        FramesizeSxga,    // 1280x1024
        FramesizeUxga,    // 1600x1200
        // 3MP Sensors
        FramesizeFhd,  // 1920x1080
        FramesizePHd,  //  720x1280
        FramesizeP3mp, //  864x1536
        FramesizeQxga, // 2048x1536
        // 5MP Sensors
        FramesizeQhd,   // 2560x1440
        FramesizeWqxga, // 2560x1600
        FramesizePFhd,  // 1080x1920
        FramesizeQsxga, // 2560x1920
        FramesizeInvalid,
    }

    /*  // This needs to be abstracted for the usage with the driver
        pub struct CameraDriver<'d> {
            i2s: i2s_port_t,

            // uint32_t dma_bytes_per_item;
            // uint32_t dma_buffer_size;
            dma_half_buffer_size: u32,
            // uint32_t dma_half_buffer_cnt;
            // uint32_t dma_node_buffer_size;
            // uint32_t dma_node_cnt;
            // uint32_t frame_copy_cnt;
            //
            // //for JPEG mode
            _dma: Vec<lldesc_t>,
            // uint8_t  *dma_buffer;
            //
            // cam_frame_t *frames;
            //
            // QueueHandle_t event_queue;
            // QueueHandle_t frame_buffer_queue;
            // TaskHandle_t task_handle;
            // intr_handle_t cam_intr_handle;
            //
            // uint8_t dma_num;//ESP32-S3
            // intr_handle_t dma_intr_handle;//ESP32-S3
            //
            // uint8_t jpeg_mode;
            vsync_pin: i32, // was uin8_t
            vsync_invert: bool,
            // uint32_t frame_cnt;
            // uint32_t recv_size;
            _swap_data: bool,
            // bool psram_mode;
            //
            // //for RGB/YUV modes
            // uint16_t width;
            // uint16_t height;
            // #if CONFIG_CAMERA_CONVERTER_ENABLED
            // float in_bytes_per_pixel;
            // float fb_bytes_per_pixel;
            // camera_conv_mode_t conv_mode;
            // #else
            // uint8_t in_bytes_per_pixel;
            // uint8_t fb_bytes_per_pixel;
            // #endif
            // uint32_t fb_size;
            //
            // cam_state_t state;

            //config: config::Config,
            _p: PhantomData<&'d mut ()>,
        }
    */
    pub fn setup(
        sda: gpio::AnyIOPin,
        scl: gpio::AnyIOPin,
        i2c: impl Peripheral<P = impl i2c::I2c> + 'static,
        pwdn: gpio::AnyIOPin,
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
    ) -> Result<i2s::CameraDriver<'static>, EspError> {
        println!("Configuring LEDC output channel");

        let ledc_timer_config = config::TimerConfig::new()
            .frequency(16.MHz().into())
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
        OV2640_PID = 0x26,
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
        let mut cam_slave = i2s::CameraDriver::new(
            i2s, vsync, href, pclk, sd0, sd1, sd2, sd3, sd4, sd5, sd6, sd7,
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
}

#[cfg(all(esp32, esp_idf_version_major = "4"))]
fn run() {
    use esp_idf_hal::delay;
    use esp_idf_hal::delay::Ets;
    use esp_idf_hal::gpio;
    use esp_idf_hal::i2c;
    use esp_idf_hal::i2s;
    use esp_idf_hal::ledc::*;
    use esp_idf_hal::peripheral::Peripheral;
    use esp_idf_hal::prelude::*;

    #[allow(unused)]
    let peripherals = Peripherals::take().unwrap();
    #[allow(unused)]
    let pins = peripherals.pins;

    println!("Before OV2640 connect");
    esp_idf_sys::link_patches();

    let i2c = peripherals.i2c1;
    let sda = pins.gpio13;
    let scl = pins.gpio12;
    let pwdn = pins.gpio26;
    let xclk = pins.gpio32;

    // I2S
    let vsync = pins.gpio27;
    //    let hsync = not used
    //    let rst = not used
    let href = pins.gpio25;
    let pclk = pins.gpio19;
    let sd0 = pins.gpio5;
    let sd1 = pins.gpio14;
    let sd2 = pins.gpio4;
    let sd3 = pins.gpio15;
    let sd4 = pins.gpio18;
    let sd5 = pins.gpio23;
    let sd6 = pins.gpio36;
    let sd7 = pins.gpio39;

    let cam_slave = camera::setup(
        sda.into(),
        scl.into(),
        i2c,
        pwdn.into(),
        xclk.into(),
        peripherals.ledc.timer0,
        peripherals.ledc.channel0,
        peripherals.i2s0,
        vsync.into(),
        href.into(),
        pclk.into(),
        sd0.into(),
        sd1.into(),
        sd2.into(),
        sd3.into(),
        sd4.into(),
        sd5.into(),
        sd6.into(),
        sd7.into(),
    )
    .unwrap();

    /* done by setup above

       println!("Starting I2C SSD1306 test");

       let config = <i2c::config::MasterConfig as Default>::default().baudrate(8.kHz().into());
       let mut i2c0 = i2c::Master::<i2c::I2C1, _, _>::new(i2c, i2c::MasterPins { sda, scl }, config)
           .expect("i2c1 init failed");
       let mut pwdn = pwdn.into_output()?;
       pwdn.set_low()?; // power-up = low

       // SCCB_Probe
       / *
       OV2640_PID = 0x26,
       OV2640_SCCB_ADDR   = 0x30
       * /
       let slave_addr = 0x30;
       let ov2640_pid: u8 = 0x26;
       let buffer: &mut [u8; 2] = &mut [0_u8, 0_u8];
       let byte1: &mut [u8; 1] = &mut [0_u8];
       let byte2: &mut [u8; 1] = &mut [0_u8];

       let i2c_res1 = i2c0.write(slave_addr, &[0xff, 0x01]);
       //        delay.delay_ms(10 as u32);
       //        let i2c_res1 = i2c0.write(slave_addr, &[0x12, 0x80]);
       //        println!("write -> {:#?}", i2c_res1);
       //        delay.delay_ms(50 as u32);
       let i2c_res = i2c0.write_read(slave_addr, &[0x0a], byte1);
       let i2c_res = i2c0.write_read(slave_addr, &[0x0b], byte2);
       println!(
           "return command: {} {} -> {:#?}",
           byte1[0], byte2[0], i2c_res
       );

       //SCCB_Write(slv_addr, 0xFF, 0x01);//bank sensor
       //uint16_t PID = SCCB_Read(slv_addr, 0x0A);
       if ov2640_pid == byte1[0] {
           println!("OV2640 found");
       } else {
           println!("OV2640 mismatch");
       }

    */
    println!("After OV2640 connect");

    //cam_slave.config(i2s::FrameSize::FramesizeVga, 0x26); // TODO: move to camera; NOT driver
    // Todo: missing impl of cam_config in cam_hal.c; start() needs this to be done before exec! -> also start needs to be at camera
    println!("OV2640 started...");

    cam_slave.start(0);
    println!("OV2640 started...");

    cam_slave.stop();
    println!("OV2640 stopped...");

    loop {} // halt system intentionally

    //Err(0_u8)
}

fn main() -> Result<(), u8> {
    esp_idf_sys::link_patches();

    #[cfg(all(esp32, esp_idf_version_major = "4"))]
    run();

    Ok(())
}
