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

use crate::peripheral::Peripheral;
use core::convert::TryInto;
use core::marker::PhantomData;
use esp_idf_sys::i2s_port_t;
use esp_idf_sys::*;

use crate::gpio::*;

crate::embedded_hal_error!(
    I2sError,
    embedded_hal::spi::Error,
    embedded_hal::spi::ErrorKind
);

/// I2S configuration
/*pub mod config {

    /// I2S configuration
    #[derive(Copy, Clone)]
    pub struct Config {
        pub vsync_invert: bool,
        //pub data_mode: embedded_hal::spi::Mode,
    }

    impl Config {
        pub fn new() -> Self {
            Default::default()
        }

        #[must_use]
        pub fn vsync_invert(mut self, vsync_invert: bool) -> Self {
            self.vsync_invert = vsync_invert;
            self
        }
        /*
        #[must_use]
        pub fn data_mode(mut self, data_mode: embedded_hal::spi::Mode) -> Self {
            self.data_mode = data_mode;
            self
        }*/
    }

    impl Default for Config {
        fn default() -> Self {
            Self {
                vsync_invert: false,
                //data_mode: embedded_hal::spi::MODE_0,
            }
        }
    }
}*/

pub mod regs {
    use core::ptr::{read_volatile, write_volatile};
    use esp_idf_sys::i2s_port_t;

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
}

/// I2S camera slave abstraction
pub struct CameraDriver<'d> {
    i2s: i2s_port_t,
    vsync_pin: i32,
    _p: PhantomData<&'d mut ()>,
}

/// Implementation for I2S0
impl<'d> CameraDriver<'d> {
    /// Create new instance of I2S controller for I2S0
    ///
    /// This ia an initial set of vars.
    pub fn new(
        _i2s: impl Peripheral<P = I2S0> + 'd,
        vsync: impl Peripheral<P = impl InputPin> + 'd,
        href: impl Peripheral<P = impl InputPin> + 'd,
        pclk: impl Peripheral<P = impl InputPin> + 'd,
        sd0: impl Peripheral<P = impl InputPin> + 'd,
        sd1: impl Peripheral<P = impl InputPin> + 'd,
        sd2: impl Peripheral<P = impl InputPin> + 'd,
        sd3: impl Peripheral<P = impl InputPin> + 'd,
        sd4: impl Peripheral<P = impl InputPin> + 'd,
        sd5: impl Peripheral<P = impl InputPin> + 'd,
        sd6: impl Peripheral<P = impl InputPin> + 'd,
        sd7: impl Peripheral<P = impl InputPin> + 'd,
        //        config: config::Config,
    ) -> Result<Self, EspError> {
        let port = <I2S0 as I2s>::port();
        crate::into_ref!(pclk, vsync, href, sd0, sd1, sd2, sd3, sd4, sd5, sd6, sd7);

        /* ll_cam_set_pins */

        /* gpio_config_t io_conf = {0};
        io_conf.intr_type = cam->vsync_invert ? GPIO_PIN_INTR_NEGEDGE : GPIO_PIN_INTR_POSEDGE;
        io_conf.pin_bit_mask = 1ULL << config->pin_vsync;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = 1;
        io_conf.pull_down_en = 0;
        gpio_config(&io_conf);
        gpio_install_isr_service(ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM);
        gpio_isr_handler_add(config->pin_vsync, ll_cam_vsync_isr, cam);
        gpio_intr_disable(config->pin_vsync);*/

        // TODO: vsync INT
        esp!(unsafe {
            gpio_install_isr_service(
                (ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM)
                    .try_into()
                    .unwrap(),
            );
            //gpio_isr_handler_add(vsync.pin().try_into().unwrap(), ll_cam_vsync_isr, cam);
            gpio_intr_disable(vsync.pin())
        })?;

        unsafe {
            // I2S0 special functions
            gpio_matrix_in(pclk.pin().try_into().unwrap(), I2S0I_WS_IN_IDX, false);
            gpio_matrix_in(vsync.pin().try_into().unwrap(), I2S0I_V_SYNC_IDX, false);
            gpio_matrix_in(href.pin().try_into().unwrap(), I2S0I_H_SYNC_IDX, false);
            gpio_matrix_in(sd0.pin().try_into().unwrap(), I2S0I_DATA_IN0_IDX, false);
            gpio_matrix_in(sd1.pin().try_into().unwrap(), I2S0I_DATA_IN1_IDX, false);
            gpio_matrix_in(sd2.pin().try_into().unwrap(), I2S0I_DATA_IN2_IDX, false);
            gpio_matrix_in(sd3.pin().try_into().unwrap(), I2S0I_DATA_IN3_IDX, false);
            gpio_matrix_in(sd4.pin().try_into().unwrap(), I2S0I_DATA_IN4_IDX, false);
            gpio_matrix_in(sd5.pin().try_into().unwrap(), I2S0I_DATA_IN5_IDX, false);
            gpio_matrix_in(sd6.pin().try_into().unwrap(), I2S0I_DATA_IN6_IDX, false);
            gpio_matrix_in(sd7.pin().try_into().unwrap(), I2S0I_DATA_IN7_IDX, false);
            gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false); // TODO: What does this do?

            /* ll_cam_config */

            periph_module_enable(esp_idf_sys::periph_module_t_PERIPH_I2S0_MODULE);
        }

        // setup i2s registers for camera slave mode
        let reg_conf2 = regs::Register::new(port, regs::I2sRegisterBank::I2S_CONF2_REG);
        let reg_conf = regs::Register::new(port, regs::I2sRegisterBank::I2S_CONF_REG);
        let reg_conf_chan = regs::Register::new(port, regs::I2sRegisterBank::I2S_CONF_CHAN_REG);
        let reg_fifo_conf = regs::Register::new(port, regs::I2sRegisterBank::I2S_FIFO_CONF_REG);

        reg_conf2.write(0x0000_0021); // Reset & I2S_LCD_EN & I2S_CAMERA_EN
        reg_conf.write(0x0000_F040); // Reset & I2S_RX_SLAVE_MOD & !I2S_RX_MSB_RIGHT & !I2S_RX_RIGHT_FIRST
        reg_conf_chan.write(0x0000_0008); // Reset & I2S_RX_CHAN_MOD[2:0]=1
        reg_fifo_conf.write(0x0001_1820); // Reset & I2S_RX_FIFO_MOD[2:0]=1

        /* default reg values missing here */

        Ok(Self {
            i2s: <I2S0 as I2s>::port() as _,
            vsync_pin: vsync.pin(),
            _p: PhantomData,
        })
    }

    /* TODO: add dma data transfer */
}

impl<'d> Drop for CameraDriver<'d> {
    fn drop(&mut self) {
        unsafe {
            gpio_intr_disable(self.vsync_pin);
        }
        if self.i2s == I2S0::port() {
            unsafe {
                periph_module_disable(esp_idf_sys::periph_module_t_PERIPH_I2S0_MODULE);
            }
        } else {
            unimplemented!();
        }
    }
}

pub trait I2s: Send {
    fn port() -> i2s_port_t;
}

macro_rules! impl_i2s {
    ($i2s:ident: $port:expr) => {
        crate::impl_peripheral!($i2s);

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
