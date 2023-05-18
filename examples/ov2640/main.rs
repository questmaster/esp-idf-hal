pub mod cam;
pub mod cam_hal;
pub mod ll_cam;
pub mod sensor;

#[cfg(all(esp32, esp_idf_version_major = "4"))]
use crate::cam::*;
use crate::sensor::{FrameSize, PixFormat};

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

    let cam_slave = EspCamera::init(
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
        Hertz::from(16.MHz()),
        PixFormat::PixformatJpeg,
        FrameSize::FramesizeVga,
        50,
        1,
    )
    .unwrap();

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
