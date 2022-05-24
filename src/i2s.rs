//! # I2S peripheral control
//!
//! Current implementation respects ESP32 only and only camera slave mode.
//!
//! ## TODO
//! - Camera mode support for HREF with HSYNC, SD8-SD15
//! - I2S for audio
//! - ADC mode
//! - LCD TX mode
//!

use core::convert::TryInto;
use core::marker::PhantomData;
use core::ptr::{read_volatile, write_volatile};
use esp_idf_sys::i2s_port_t;
use esp_idf_sys::*;

use crate::{delay::*, gpio::*, units::*};

crate::embedded_hal_error!(
    I2sError,
    embedded_hal::spi::Error,
    embedded_hal::spi::ErrorKind
);

/// Pins used by the I2S camera slave interface
pub struct CameraPins<
    VSYNC: InputPin,
    HREF: InputPin,
    PCLK: InputPin,
    SD0: InputPin,
    SD1: InputPin,
    SD2: InputPin,
    SD3: InputPin,
    SD4: InputPin,
    SD5: InputPin,
    SD6: InputPin,
    SD7: InputPin,
> {
    pub vsync: VSYNC,
    pub href: HREF,
    pub pclk: PCLK,
    pub sd0: SD0,
    pub sd1: SD1,
    pub sd2: SD2,
    pub sd3: SD3,
    pub sd4: SD4,
    pub sd5: SD5,
    pub sd6: SD6,
    pub sd7: SD7,
}

/* TODO: config currently not used. Is it needed at all?
/// I2S configuration
pub mod config {
    use crate::units::*;

    /// I2S configuration
    #[derive(Copy, Clone)]
    pub struct Config {
        pub baudrate: Hertz,
        pub data_mode: embedded_hal::spi::Mode,
    }

    impl Config {
        pub fn new() -> Self {
            Default::default()
        }
        /*
                #[must_use]
                pub fn baudrate(mut self, baudrate: Hertz) -> Self {
                    self.baudrate = baudrate;
                    self
                }

                #[must_use]
                pub fn data_mode(mut self, data_mode: embedded_hal::spi::Mode) -> Self {
                    self.data_mode = data_mode;
                    self
                }*/
    }

    impl Default for Config {
        fn default() -> Self {
            Self {
                baudrate: Hertz(1_000_000),
                data_mode: embedded_hal::spi::MODE_0,
            }
        }
    }
}*/

/// This enum defines the I2S peripheral registers available
#[allow(non_camel_case_types, dead_code)]
pub enum I2sRegisterBank {
    I2S_FIFO_WR_REG,
    I2S_FIFO_RD_REG,
    I2S_CONF_REG,
    I2S_CONF1_REG,
    I2S_CONF2_REG,
    I2S_TIMING_REG,
    I2S_FIFO_CONF_REG,
    I2S_CONF_SINGLE_DATA_REG,
    I2S_CONF_CHAN_REG,
    I2S_LC_HUNG_CONF_REG,
    I2S_CLKM_CONF_REG,
    I2S_SAMPLE_RATE_CONF_REG,
    I2S_PD_CONF_REG,
    I2S_STATE_REG,
    // ... more
}

/// The Register struct allows access to low level registers for I2S configuration.
pub struct Register {
    port: i2s_port_t,
    reg: I2sRegisterBank,
}

impl Register {
    pub fn new(port: i2s_port_t, reg: I2sRegisterBank) -> Self {
        Self { port, reg }
    }

    const fn reg_address(port: &i2s_port_t, reg: &I2sRegisterBank) -> u32 {
        #[allow(non_snake_case)]
        let mut address = match *port {
            esp_idf_sys::i2s_port_t_I2S_NUM_0 => 0x3FF4_F000,
            #[cfg(I2S1_enabled)]
            esp_idf_sys::i2s_port_t_I2S_NUM_1 => 0x3FF6_D000,
            _ => unreachable!(),
        };

        address += match reg {
            I2sRegisterBank::I2S_FIFO_WR_REG => 0x00,
            I2sRegisterBank::I2S_FIFO_RD_REG => 0x04,
            I2sRegisterBank::I2S_CONF_REG => 0x08,
            I2sRegisterBank::I2S_CONF1_REG => 0xA0,
            I2sRegisterBank::I2S_CONF2_REG => 0xA8,
            I2sRegisterBank::I2S_TIMING_REG => 0x1C,
            I2sRegisterBank::I2S_FIFO_CONF_REG => 0x20,
            I2sRegisterBank::I2S_CONF_SINGLE_DATA_REG => 0x28,
            I2sRegisterBank::I2S_CONF_CHAN_REG => 0x2C,
            I2sRegisterBank::I2S_LC_HUNG_CONF_REG => 0x74,
            I2sRegisterBank::I2S_CLKM_CONF_REG => 0xAC,
            I2sRegisterBank::I2S_SAMPLE_RATE_CONF_REG => 0xB0,
            I2sRegisterBank::I2S_PD_CONF_REG => 0xA4,
            I2sRegisterBank::I2S_STATE_REG => 0xBC,
            // ... more
        };
        if address % 4 != 0 {
            panic!("Non-aligned register address for I2S peripheral.")
        }

        address
    }

    pub fn read(&self) -> u32 {
        unsafe { read_volatile(Register::reg_address(&self.port, &self.reg) as *mut u32) }
    }

    pub fn write(&self, value: u32) -> &Self {
        unsafe {
            write_volatile(
                Register::reg_address(&self.port, &self.reg) as *mut u32,
                value,
            );
        }
        self
    }

    pub fn set_mask(&self, mask: u32) -> &Self {
        let mut value = self.read();
        value |= mask;
        self.write(value)
    }

    pub fn unset_mask(&self, mask: u32) -> &Self {
        let mut value = self.read();
        value &= !mask;
        self.write(value)
    }
}

/// I2S camera slave abstraction
pub struct CameraSlave<
    I2S: I2s,
    VSYNC: InputPin,
    HREF: InputPin,
    PCLK: InputPin,
    SD0: InputPin,
    SD1: InputPin,
    SD2: InputPin,
    SD3: InputPin,
    SD4: InputPin,
    SD5: InputPin,
    SD6: InputPin,
    SD7: InputPin,
> {
    i2s: I2S,
    pins: CameraPins<VSYNC, HREF, PCLK, SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7>,
    port: i2s_port_t,
}

/// Implementation for I2S0
impl<
        VSYNC: InputPin,
        HREF: InputPin,
        PCLK: InputPin,
        SD0: InputPin,
        SD1: InputPin,
        SD2: InputPin,
        SD3: InputPin,
        SD4: InputPin,
        SD5: InputPin,
        SD6: InputPin,
        SD7: InputPin,
    > CameraSlave<I2S0, VSYNC, HREF, PCLK, SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7>
{
    /// Create new instance of I2S controller for I2S0
    ///
    /// This ia an initial set of vars.
    pub fn new(
        i2s: I2S0,
        pins: CameraPins<VSYNC, HREF, PCLK, SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7>,
        //        config: config::Config,
    ) -> Result<Self, EspError> {
        CameraSlave::new_internal(i2s, pins /*, config*/)
    }
}

/// Implementation for I2S1
#[cfg(I2S1_enabled)]
impl<
        VSYNC: InputPin,
        HREF: InputPin,
        PCLK: InputPin,
        SD0: InputPin,
        SD1: InputPin,
        SD2: InputPin,
        SD3: InputPin,
        SD4: InputPin,
        SD5: InputPin,
        SD6: InputPin,
        SD7: InputPin,
    > CameraSlave<I2S1, VSYNC, HREF, PCLK, SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7>
{
    /// Create new instance of I2S controller for I2S1
    ///
    /// This ia an initial set of vars.
    pub fn new(
        i2s: I2S1,
        pins: CameraPins<VSYNC, HREF, PCLK, SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7>,
        //        config: config::Config,
    ) -> Result<Self, EspError> {
        CameraSlave::new_internal(i2s, pins /*, config*/)
    }
}

/// General implementation
impl<
        I2S: I2s,
        VSYNC: InputPin,
        HREF: InputPin,
        PCLK: InputPin,
        SD0: InputPin,
        SD1: InputPin,
        SD2: InputPin,
        SD3: InputPin,
        SD4: InputPin,
        SD5: InputPin,
        SD6: InputPin,
        SD7: InputPin,
    > CameraSlave<I2S, VSYNC, HREF, PCLK, SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7>
{
    /// Internal implementation of new() shared by all I2S peripherals
    fn new_internal(
        i2s: I2S,
        pins: CameraPins<VSYNC, HREF, PCLK, SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7>,
        //        config: config::Config,
    ) -> Result<Self, EspError> {
        let port = I2S::port();
        let vsync = pins.vsync.pin();

        // TODO: vsync INT
        esp!(unsafe {
            gpio_install_isr_service(
                (ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM)
                    .try_into()
                    .unwrap(),
            )
        })?;
        //gpio_isr_handler_add(vsync, ll_cam_vsync_isr, cam); // TODO: pass isr handler func
        esp!(unsafe { gpio_intr_disable(vsync) })?;

        if port == I2S0::port() {
            unsafe {
                // I2S0 special functions
                gpio_matrix_in(pins.pclk.pin().try_into().unwrap(), I2S0I_WS_IN_IDX, false);
                gpio_matrix_in(vsync.try_into().unwrap(), I2S0I_V_SYNC_IDX, false);
                gpio_matrix_in(pins.href.pin().try_into().unwrap(), I2S0I_H_SYNC_IDX, false);
                gpio_matrix_in(
                    pins.sd0.pin().try_into().unwrap(),
                    I2S0I_DATA_IN0_IDX,
                    false,
                );
                gpio_matrix_in(
                    pins.sd1.pin().try_into().unwrap(),
                    I2S0I_DATA_IN1_IDX,
                    false,
                );
                gpio_matrix_in(
                    pins.sd2.pin().try_into().unwrap(),
                    I2S0I_DATA_IN2_IDX,
                    false,
                );
                gpio_matrix_in(
                    pins.sd3.pin().try_into().unwrap(),
                    I2S0I_DATA_IN3_IDX,
                    false,
                );
                gpio_matrix_in(
                    pins.sd4.pin().try_into().unwrap(),
                    I2S0I_DATA_IN4_IDX,
                    false,
                );
                gpio_matrix_in(
                    pins.sd5.pin().try_into().unwrap(),
                    I2S0I_DATA_IN5_IDX,
                    false,
                );
                gpio_matrix_in(
                    pins.sd6.pin().try_into().unwrap(),
                    I2S0I_DATA_IN6_IDX,
                    false,
                );
                gpio_matrix_in(
                    pins.sd7.pin().try_into().unwrap(),
                    I2S0I_DATA_IN7_IDX,
                    false,
                );
                gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false); // TODO: What does this do?

                periph_module_enable(esp_idf_sys::periph_module_t_PERIPH_I2S0_MODULE);
            }
        } else {
            unsafe {
                // I2S1 special functions
                gpio_matrix_in(pins.pclk.pin().try_into().unwrap(), I2S1I_WS_IN_IDX, false);
                gpio_matrix_in(vsync.try_into().unwrap(), I2S1I_V_SYNC_IDX, false);
                gpio_matrix_in(pins.href.pin().try_into().unwrap(), I2S1I_H_SYNC_IDX, false);
                gpio_matrix_in(
                    pins.sd0.pin().try_into().unwrap(),
                    I2S1I_DATA_IN0_IDX,
                    false,
                );
                gpio_matrix_in(
                    pins.sd1.pin().try_into().unwrap(),
                    I2S1I_DATA_IN1_IDX,
                    false,
                );
                gpio_matrix_in(
                    pins.sd2.pin().try_into().unwrap(),
                    I2S1I_DATA_IN2_IDX,
                    false,
                );
                gpio_matrix_in(
                    pins.sd3.pin().try_into().unwrap(),
                    I2S1I_DATA_IN3_IDX,
                    false,
                );
                gpio_matrix_in(
                    pins.sd4.pin().try_into().unwrap(),
                    I2S1I_DATA_IN4_IDX,
                    false,
                );
                gpio_matrix_in(
                    pins.sd5.pin().try_into().unwrap(),
                    I2S1I_DATA_IN5_IDX,
                    false,
                );
                gpio_matrix_in(
                    pins.sd6.pin().try_into().unwrap(),
                    I2S1I_DATA_IN6_IDX,
                    false,
                );
                gpio_matrix_in(
                    pins.sd7.pin().try_into().unwrap(),
                    I2S1I_DATA_IN7_IDX,
                    false,
                );
                gpio_matrix_in(0x38, I2S1I_H_ENABLE_IDX, false); // TODO: What does this do?

                periph_module_enable(esp_idf_sys::periph_module_t_PERIPH_I2S1_MODULE);
            }
        };

        // setup i2s registers for camera slave mode
        let reg_conf2 = Register::new(port, I2sRegisterBank::I2S_CONF2_REG);
        let reg_conf = Register::new(port, I2sRegisterBank::I2S_CONF_REG);
        let reg_conf_chan = Register::new(port, I2sRegisterBank::I2S_CONF_CHAN_REG);
        let reg_fifo_conf = Register::new(port, I2sRegisterBank::I2S_FIFO_CONF_REG);

        reg_conf2.write(0x0000_0021); // Reset & I2S_LCD_EN & I2S_CAMERA_EN
        reg_conf.write(0x0000_F040); // Reset & I2S_RX_SLAVE_MOD & !I2S_RX_MSB_RIGHT & !I2S_RX_RIGHT_FIRST
        reg_conf_chan.write(0x0000_0008); // Reset & I2S_RX_CHAN_MOD[2:0]=1
        reg_fifo_conf.write(0x0001_1820); // Reset & I2S_RX_FIFO_MOD[2:0]=1

        Ok(Self { i2s, pins, port })
    }

    /* TODO: add dma data transfer */
}

impl<
        I2S: I2s,
        VSYNC: InputPin,
        HREF: InputPin,
        PCLK: InputPin,
        SD0: InputPin,
        SD1: InputPin,
        SD2: InputPin,
        SD3: InputPin,
        SD4: InputPin,
        SD5: InputPin,
        SD6: InputPin,
        SD7: InputPin,
    > Drop for CameraSlave<I2S, VSYNC, HREF, PCLK, SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7>
{
    fn drop(&mut self) {
        unsafe {
            gpio_intr_disable(self.pins.vsync.pin());
        }
        if self.port == I2S0::port() {
            unsafe {
                periph_module_disable(esp_idf_sys::periph_module_t_PERIPH_I2S0_MODULE);
            }
        } else {
            unsafe {
                periph_module_disable(esp_idf_sys::periph_module_t_PERIPH_I2S1_MODULE);
            }
        }
    }
}

pub trait I2s: Send {
    fn port() -> i2s_port_t;
}

macro_rules! impl_i2s {
    ($i2s:ident: $port:expr) => {
        pub struct $i2s(::core::marker::PhantomData<*const ()>);

        impl $i2s {
            /// # Safety
            ///
            /// Care should be taken not to instantiate this I2S instance, if it is already instantiated and used elsewhere
            pub unsafe fn new() -> Self {
                $i2s(::core::marker::PhantomData)
            }
        }

        unsafe impl Send for $i2s {}

        impl I2s for $i2s {
            #[inline(always)]
            fn port() -> i2s_port_t {
                $port
            }
        }
    };
}

impl_i2s!(I2S0: esp_idf_sys::i2s_port_t_I2S_NUM_0);
#[cfg(I2S1_enabled)]
impl_i2s!(I2S1: esp_idf_sys::i2s_port_t_I2S_NUM_1); // It seems by default ESP_IDF is build with only one I2S enabled.
