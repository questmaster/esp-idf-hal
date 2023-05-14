#[cfg(all(esp32, esp_idf_version_major = "4"))]
pub mod camera {
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

    // This needs to be abstracted for the usage with the driver
    pub struct Camera {
        //i2s: i2s_port_t,
        dma_bytes_per_item: u32,
        dma_buffer_size: u32,
        dma_half_buffer_size: u32,
        dma_half_buffer_cnt: u32,
        dma_node_buffer_size: u32,
        // uint32_t dma_node_cnt;
        // uint32_t frame_copy_cnt;
        //
        // //for JPEG mode
        //_dma: Vec<lldesc_t>,
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
        jpeg_mode: bool,
        //vsync_pin: i32, // was uin8_t
        //vsync_invert: bool,
        // uint32_t frame_cnt;
        // uint32_t recv_size;
        //_swap_data: bool,
        // bool psram_mode;

        //for RGB/YUV modes
        width: u16,
        height: u16,
        in_bytes_per_pixel: u8,
        fb_bytes_per_pixel: u8,
        fb_size: u32,
        //
        // cam_state_t state;

        //config: config::Config,
    }

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

    pub fn calc_rgb_dma(cam: &mut Camera) -> bool {
        let dma_half_buffer_max = CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX / 2 / cam.dma_bytes_per_item;
        let dma_buffer_max = 2 * dma_half_buffer_max;
        let node_max = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE / cam.dma_bytes_per_item;

        let line_width = cam.width * cam.in_bytes_per_pixel;
        let image_size = cam.height * line_width;
        if image_size > (4 * 1024 * 1024) || (line_width > dma_half_buffer_max) {
            ESP_LOGE(TAG, "Resolution too high");
            return false;
        }

        let node_size = node_max;
        let mut nodes_per_line = 1;
        let mut lines_per_node = 1;
        let mut lines_per_half_buffer = 1;
        let dma_half_buffer_min = node_max;
        let dma_half_buffer = dma_half_buffer_max;
        let dma_buffer_size = dma_buffer_max;

        // Calculate DMA Node Size so that it's dividable by or divisor of the line width
        if line_width >= node_max {
            // One or more nodes will be required for one line
            for i in node_max..0 {
                if (line_width % i) == 0 {
                    node_size = i;
                    nodes_per_line = line_width / node_size;
                    break;
                }
            }
        } else {
            // One or more lines can fit into one node
            for i in node_max..0 {
                if (i % line_width) == 0 {
                    node_size = i;
                    lines_per_node = node_size / line_width;
                    while (cam.height % lines_per_node) != 0 {
                        lines_per_node = lines_per_node - 1;
                        node_size = lines_per_node * line_width;
                    }
                    break;
                }
            }
        }
        // Calculate minimum EOF size = max(mode_size, line_size)
        dma_half_buffer_min = node_size * nodes_per_line;
        // Calculate max EOF size divisable by node size
        dma_half_buffer = (dma_half_buffer_max / dma_half_buffer_min) * dma_half_buffer_min;
        // Adjust EOF size so that height will be divisable by the number of lines in each EOF
        lines_per_half_buffer = dma_half_buffer / line_width;
        while (cam.height % lines_per_half_buffer) != 0 {
            dma_half_buffer = dma_half_buffer - dma_half_buffer_min;
            lines_per_half_buffer = dma_half_buffer / line_width;
        }
        // Calculate DMA size
        dma_buffer_size = (dma_buffer_max / dma_half_buffer) * dma_half_buffer;

        ESP_LOGI(TAG, "node_size: %4u, nodes_per_line: %u, lines_per_node: %u, dma_half_buffer_min: %5u, dma_half_buffer: %5u, lines_per_half_buffer: %2u, dma_buffer_size: %5u, image_size: %u",
                 node_size * cam.dma_bytes_per_item, nodes_per_line, lines_per_node,
                 dma_half_buffer_min * cam.dma_bytes_per_item, dma_half_buffer * cam.dma_bytes_per_item,
                 lines_per_half_buffer, dma_buffer_size * cam.dma_bytes_per_item, image_size);

        cam.dma_buffer_size = dma_buffer_size * cam.dma_bytes_per_item;
        cam.dma_half_buffer_size = dma_half_buffer * cam.dma_bytes_per_item;
        cam.dma_node_buffer_size = node_size * cam.dma_bytes_per_item;
        cam.dma_half_buffer_cnt = cam.dma_buffer_size / cam.dma_half_buffer_size;
        return true;
    }

    fn dma_sizes(cam: &mut Camera) -> bool {
        cam.dma_bytes_per_item = i2s::CameraDriver::bytes_per_sample(sampling_mode); // todo: get from camera driver
        if cam.jpeg_mode {
            cam.dma_half_buffer_cnt = 8;
            cam.dma_node_buffer_size = 2048;
            cam.dma_half_buffer_size = cam.dma_node_buffer_size * 2;
            cam.dma_buffer_size = cam.dma_half_buffer_cnt * cam.dma_half_buffer_size;
        } else {
            return calc_rgb_dma(cam);
        }
        return true;
    }
    fn set_sample_mode(
        cam: &mut Camera,
        pix_format: PixFormat,
        xclk_freq_hz: u32,
        sensor_pid: u16,
    ) -> EspError {
        if pix_format == PIXFORMAT_GRAYSCALE {
            if sensor_pid == OV3660_PID
                || sensor_pid == OV5640_PID
                || sensor_pid == NT99141_PID
                || sensor_pid == SC031GS_PID
            {
                if xclk_freq_hz > 10000000 {
                    sampling_mode = i2s::SamplingMode::SM_0A00_0B00;
                    dma_filter = ll_cam_dma_filter_yuyv_highspeed;
                } else {
                    sampling_mode = i2s::SamplingMode::SM_0A0B_0C0D;
                    dma_filter = ll_cam_dma_filter_yuyv;
                }
                cam.in_bytes_per_pixel = 1; // camera sends Y8
            } else {
                if xclk_freq_hz > 10000000 && sensor_pid != OV7725_PID {
                    sampling_mode = i2s::SamplingMode::SM_0A00_0B00;
                    dma_filter = ll_cam_dma_filter_grayscale_highspeed;
                } else {
                    sampling_mode = i2s::SamplingMode::SM_0A0B_0C0D;
                    dma_filter = ll_cam_dma_filter_grayscale;
                }
                cam.in_bytes_per_pixel = 2; // camera sends YU/YV
            }
            self.fb_bytes_per_pixel = 1; // frame buffer stores Y8
        } else if pix_format == PIXFORMAT_YUV422 || pix_format == PIXFORMAT_RGB565 {
            if xclk_freq_hz > 10000000 && sensor_pid != OV7725_PID {
                if sensor_pid == OV7670_PID {
                    sampling_mode = i2s::SamplingMode::SM_0A0B_0B0C;
                } else {
                    sampling_mode = i2s::SamplingMode::SM_0A00_0B00;
                }
                dma_filter = ll_cam_dma_filter_yuyv_highspeed;
            } else {
                sampling_mode = i2s::SamplingMode::SM_0A0B_0C0D;
                dma_filter = ll_cam_dma_filter_yuyv;
            }
            cam.in_bytes_per_pixel = 2; // camera sends YU/YV
            cam.fb_bytes_per_pixel = 2; // frame buffer stores YU/YV/RGB565
        } else if pix_format == PIXFORMAT_JPEG {
            cam.in_bytes_per_pixel = 1;
            cam.fb_bytes_per_pixel = 1;
            dma_filter = ll_cam_dma_filter_jpeg;
            sampling_mode = i2s::SamplingMode::SM_0A00_0B00;
        } else {
            ESP_LOGE(TAG, "Requested format is not supported");
            return ESP_ERR_NOT_SUPPORTED;
        }
        //I2S0.fifo_conf.rx_fifo_mod = sampling_mode; // TODO: correct reg access / set sampling mode
        return ESP_OK;
    }
}

#[cfg(all(esp32, esp_idf_version_major = "4"))]
fn run() {
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
