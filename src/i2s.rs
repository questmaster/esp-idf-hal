///
/// Current implementation respects ESP32 only.
///

use core::marker::PhantomData;
use esp_idf_sys::*;
use core::ptr::{read_volatile, write_volatile};
use esp_idf_sys::i2s_port_t;

use crate::{delay::*, gpio::*, units::*};


/// Pins used by the I2S interface
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
}

/// This enum defines the I2S peripheral registers available
#[allow(non_camel_case_types, dead_code)]
pub enum I2sBankRegisters {
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

/// The register struct allows access to low level registers for I2S configuration.
pub struct Register {
    port: i2s_port_t,
    reg: I2sBankRegisters,
}

impl Register {
    pub fn new(port: i2s_port_t,
               reg: I2sBankRegisters) -> Self {
        Self { port, reg }
    }

    #[inline]
    fn reg_address(port: &i2s_port_t,
                   reg: &I2sBankRegisters) -> u32 {
        #[allow(non_snake_case)] let mut address = match *port {
            esp_idf_sys::i2s_port_t_I2S_NUM_0 => 0x3FF4_F000,
            esp_idf_sys::i2s_port_t_I2S_NUM_MAX => 0x3FF6_D000,
            _ => unreachable!(),
        };

        address += match reg {
            I2sBankRegisters::I2S_FIFO_WR_REG => 0x00,
            I2sBankRegisters::I2S_FIFO_RD_REG => 0x04,
            I2sBankRegisters::I2S_CONF_REG => 0x08,
            I2sBankRegisters::I2S_CONF1_REG => 0xA0,
            I2sBankRegisters::I2S_CONF2_REG => 0xA8,
            I2sBankRegisters::I2S_TIMING_REG => 0x1C,
            I2sBankRegisters::I2S_FIFO_CONF_REG => 0x20,
            I2sBankRegisters::I2S_CONF_SINGLE_DATA_REG => 0x28,
            I2sBankRegisters::I2S_CONF_CHAN_REG => 0x2C,
            I2sBankRegisters::I2S_LC_HUNG_CONF_REG => 0x74,
            I2sBankRegisters::I2S_CLKM_CONF_REG => 0xAC,
            I2sBankRegisters::I2S_SAMPLE_RATE_CONF_REG => 0xB0,
            I2sBankRegisters::I2S_PD_CONF_REG => 0xA4,
            I2sBankRegisters::I2S_STATE_REG => 0xBC,
            // ... more
        };
        address
    }

    pub fn read(&self) -> u32 {
        unsafe {
            read_volatile(Register::reg_address(&self.port, &self.reg) as *mut u32)
        }
    }

    pub fn write(&self, value: u32) -> &Self {
        unsafe {
            write_volatile(Register::reg_address(&self.port, &self.reg) as *mut u32, value);
        }
        self
    }

    pub fn set_mask(&self, mask: u32) -> &Self {
        let mut val = self.read();
        val |= mask;
        self.write(val);

        self
    }

    pub fn unset_mask(&self, mask: u32) -> &Self {
        let mut val = self.read();
        val &= !mask;
        self.write(val);

        self
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
impl<VSYNC: InputPin, HREF: InputPin, PCLK: InputPin,
    SD0: InputPin, SD1: InputPin, SD2: InputPin, SD3: InputPin, SD4: InputPin, SD5: InputPin, SD6: InputPin, SD7: InputPin>
CameraSlave<I2S0, VSYNC, HREF, PCLK, SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7>
{
    /// Create new instance of I2S controller for I2S0
    ///
    /// This ia an initial set of vars.
    pub fn new(
        i2s: I2S0,
        pins: CameraPins<VSYNC, HREF, PCLK, SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7>,
        config: config::Config,
    ) -> Result<Self, EspError> {
        CameraSlave::new_internal(i2s, pins, config)
    }
}

/// General implementation
impl<I2S: I2s, VSYNC: InputPin, HREF: InputPin, PCLK: InputPin,
    SD0: InputPin, SD1: InputPin, SD2: InputPin, SD3: InputPin, SD4: InputPin, SD5: InputPin, SD6: InputPin, SD7: InputPin>
CameraSlave<I2S, VSYNC, HREF, PCLK, SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7>
{
    /// Internal implementation of new shared by all SPI controllers
    fn new_internal(
        i2s: I2S,
        pins: CameraPins<VSYNC, HREF, PCLK, SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7>,
        config: config::Config,
    ) -> Result<Self, EspError> {
        let port = I2S::port();

        // setup i2s registers for camera slave mode
        let reg_conf2 = Register::new(port, I2sBankRegisters::I2S_CONF2_REG);
        let reg_conf = Register::new(port, I2sBankRegisters::I2S_CONF_REG);
        let reg_conf_chan = Register::new(port, I2sBankRegisters::I2S_CONF_CHAN_REG);
        let reg_fifo_conf = Register::new(port, I2sBankRegisters::I2S_FIFO_CONF_REG);

        reg_conf2.write(0x0000_0021);       // Reset & I2S_LCD_EN & I2S_CAMERA_EN
        reg_conf.write(0x0000_F040);        // Reset & I2S_RX_SLAVE_MOD & !I2S_RX_MSB_RIGHT & !I2S_RX_RIGHT_FIRST
        reg_conf_chan.write(0x0000_0008);   // Reset & I2S_RX_CHAN_MOD[2:0]=1
        reg_fifo_conf.write(0x0001_1820);   // Reset & I2S_RX_FIFO_MOD[2:0]=1

        Ok(Self {
            i2s,
            pins,
            port,
        })
    }

    /* TODO: add data transfer */
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

impl_i2s!(I2S0: esp_idf_sys::i2s_port_t_I2S_NUM_0); // TODO: naming in esp-idf-sys needs fixing
impl_i2s!(I2S1: esp_idf_sys::i2s_port_t_I2S_NUM_MAX);
