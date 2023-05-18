pub mod cam;
pub mod cam_hal;
pub mod ll_cam;
pub mod sensor;

use crate::cam::*;

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

    let cam_slave = setup(
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
       Ov2640Pid = 0x26,
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
