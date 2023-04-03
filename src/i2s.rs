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

use crate::gpio::*;
use crate::peripheral::Peripheral;
use core::convert::TryInto;
use core::marker::PhantomData;
use esp_idf_sys::i2s_port_t;
use esp_idf_sys::*;
use std::os::raw::c_void;

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

        vsync_invert: bool,

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

        #[must_use]
        pub fn dma_half_buffer_size(mut self, size: u32) -> Self {
            self.dma_half_buffer_size = size;
            self
        }
    }

    impl Default for Config {
        fn default() -> Self {
            Self {
                dma_half_buffer_size: 0,
                dma: [],
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
        /* I2S FIFO registers */
        I2S_FIFO_WR_REG,
        I2S_FIFO_RD_REG,
        /* Configuration registers */
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
        /* DMA registers */
        I2S_LC_CONF_REG,
        I2S_RXEOF_NUM_REG,
        I2S_OUT_LINK_REG,
        I2S_IN_LINK_REG,
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

        /// Get requested registers address for given I2S port.
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
                I2sRegisterBank::I2S_LC_CONF_REG => 0x08,
                I2sRegisterBank::I2S_RXEOF_NUM_REG => 0x24,
                I2sRegisterBank::I2S_OUT_LINK_REG => 0x30,
                I2sRegisterBank::I2S_IN_LINK_REG => 0x34,
                // ... more
            };
            if address % 4 != 0 {
                panic!("Non-aligned register address for I2S peripheral.")
            }

            address
        }

        /// Read full register value.
        pub fn read(&self) -> u32 {
            unsafe { read_volatile(Register::reg_address(&self.port, &self.reg) as *mut u32) }
        }

        /// Write full register <i>value</i>.
        pub fn write(&self, value: u32) -> &Self {
            unsafe {
                write_volatile(
                    Register::reg_address(&self.port, &self.reg) as *mut u32,
                    value,
                );
            }
            self
        }

        /// Write only bits from value defined in mask into register.
        pub fn write_masked(&self, mask: u32, value: u32) -> &Self {
            let mut result = self.maskout_bits(mask);
            result |= value & mask;
            self.write(result)
        }

        /// Set all bits in register found in mask and write back.
        pub fn set_bits(&self, mask: u32) -> &Self {
            let mut value = self.read();
            value |= mask;
            self.write(value)
        }

        /// Unset all bits in register found in mask and write back to register.
        pub fn unset_bits(&self, mask: u32) -> &Self {
            let value = self.maskout_bits(mask);
            self.write(value)
        }

        /// Unset bits of register and return value.
        fn maskout_bits(&self, mask: u32) -> u32 {
            let mut value = self.read();
            value &= !mask;
            value
        }
    }
}

//struct DmaSample {
struct DmaElement {
    _sample2: u8,
    _unused2: u8,
    _sample1: u8,
    _unused1: u8,
}

//union DmaElement {
//    sampled: DmaSample,
//    value: u32,
//}

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

/// I2S camera slave abstraction
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

/// Implementation for I2S0
impl<'d> CameraDriver<'d> {
    /// Create new instance of I2S controller for I2S0
    ///
    /// This ia an initial set of vars.
    #[allow(clippy::too_many_arguments)]
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
        //config: config::Config,
    ) -> Result<Self, EspError> {
        let port = <I2S0 as I2s>::port();
        crate::into_ref!(pclk, vsync, href, sd0, sd1, sd2, sd3, sd4, sd5, sd6, sd7);
        let result = Self {
            i2s: <I2S0 as I2s>::port() as _,
            dma_half_buffer_size: 0,
            _dma: Vec::with_capacity(0),
            vsync_pin: vsync.pin(),
            vsync_invert: true,
            _swap_data: false,
            _p: PhantomData,
        };

        // vsync INT
        /* ll_cam_set_pin */
        let io_conf = gpio_config_t {
            intr_type: if result.vsync_invert {
                GPIO_INT_TYPE_GPIO_PIN_INTR_NEGEDGE
            } else {
                GPIO_INT_TYPE_GPIO_PIN_INTR_POSEDGE
            },
            pin_bit_mask: 1_u64 << vsync.pin(),
            mode: gpio_mode_t_GPIO_MODE_INPUT,
            pull_up_en: 1,
            pull_down_en: 0,
        };
        esp!(unsafe {
            gpio_config(&io_conf);
            gpio_install_isr_service(
                (ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM)
                    .try_into()
                    .unwrap(),
            );
            gpio_isr_handler_add(
                vsync.pin(),
                Some(Self::cam_vsync_isr),
                /*0 as *mut c_void*/ std::ptr::null_mut::<c_void>(),
            ); // todo: correct parameter
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
            gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false); // What does this do?

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

        /* other default reg values missing/not implemented here */

        Ok(result)
    }

    pub fn config(&self, _frame_size: FrameSize, _sensor_pid: u16) -> Result<(), EspError> {
        // Todo: missing impl of cam_config in cam_hal.c; start() needs this to be done before exec!
        unimplemented!()
    }

    pub fn start(&self, _frame_pos: usize) -> bool {
        let port = self.i2s;

        let reg_conf = regs::Register::new(port, regs::I2sRegisterBank::I2S_CONF_REG);
        let reg_lc_conf = regs::Register::new(port, regs::I2sRegisterBank::I2S_LC_CONF_REG); // lc_conf?
        let reg_in_link = regs::Register::new(port, regs::I2sRegisterBank::I2S_IN_LINK_REG);
        let reg_rxeof_num = regs::Register::new(port, regs::I2sRegisterBank::I2S_RXEOF_NUM_REG);

        reg_conf.unset_bits(0x0000_0020); //I2S0.conf.rx_start = 0;

        //unsafe {}; // todo: I2S_ISR_ENABLE(in_suc_eof);

        reg_conf.set_bits(0x0000_0002); //I2S0.conf.rx_reset = 1;
        reg_conf.unset_bits(0x0000_0002); //I2S0.conf.rx_reset = 0;
        reg_conf.set_bits(0x0000_0008); //I2S0.conf.rx_fifo_reset = 1;
        reg_conf.unset_bits(0x0000_0008); //I2S0.conf.rx_fifo_reset = 0;
        reg_lc_conf.set_bits(0x0000_0001); //I2S0.lc_conf.in_rst = 1;
        reg_lc_conf.unset_bits(0x0000_0001); //I2S0.lc_conf.in_rst = 0;
        reg_lc_conf.set_bits(0x0000_0004); //I2S0.lc_conf.ahbm_fifo_rst = 1;
        reg_lc_conf.unset_bits(0x0000_0004); //I2S0.lc_conf.ahbm_fifo_rst = 0;
        reg_lc_conf.set_bits(0x0000_0008); //I2S0.lc_conf.ahbm_rst = 1;
        reg_lc_conf.unset_bits(0x0000_0008); //I2S0.lc_conf.ahbm_rst = 0;

        // todo: config items and dma buffer needed! -> see fn config()
        reg_rxeof_num.write(self.dma_half_buffer_size / std::mem::size_of::<DmaElement>() as u32); //I2S0.rx_eof_num = cam->dma_half_buffer_size / sizeof(dma_elem_t);

        // todo:reg_in_link.write_masked(0x000f_ffff, std::ptr::addr_of!(self.dma[0]) as u32); //I2S0.in_link.addr = ((uint32_t)&cam->dma[0]) & 0xfffff;

        reg_in_link.set_bits(0x2000_0000); //I2S0.in_link.start = 1;

        reg_conf.set_bits(0x0000_0020); // I2S0.conf.rx_start = 1;

        true
    }

    pub fn stop(&self) -> bool {
        let port = <I2S0 as I2s>::port();

        let reg_conf = regs::Register::new(port, regs::I2sRegisterBank::I2S_CONF_REG);
        let reg_in_link = regs::Register::new(port, regs::I2sRegisterBank::I2S_IN_LINK_REG);

        reg_conf.unset_bits(0x0000_0020); //I2S0.conf.rx_start = 0;

        //unsafe {}; // todo: I2S_ISR_DISABLE(in_suc_eof);
        reg_in_link.set_bits(0x1000_0000); // I2S0.in_link.stop = 1;

        true
    }

    /* --- ISR Handler --- */

    unsafe extern "C" fn cam_vsync_isr(_cam_driver: *mut c_void) {}

    /* --- DMA --- */
    /* TODO: add dma data transfer */
}

impl<'d> Drop for CameraDriver<'d> {
    fn drop(&mut self) {
        unsafe {
            gpio_isr_handler_remove(self.vsync_pin);
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
