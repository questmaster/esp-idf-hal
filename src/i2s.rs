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

use crate::gpio::InputPin;
use crate::peripheral::Peripheral;
use core::ffi::c_void;
use core::marker::PhantomData;
use esp_idf_sys::*;

crate::embedded_hal_error!(
    I2sError,
    embedded_hal::spi::Error,
    embedded_hal::spi::ErrorKind
);

// definition missing in esp_idf_sys for esp32s2
#[cfg(esp32s2)]
static ETS_I2S0_INTR_SOURCE: u8 = 35;

pub mod config {
    /// I2S configuration
    #[derive(Copy, Clone, Default)]
    pub struct Config<'a> {
        pub dma_buffer: Option<&'a [u8]>,
        pub vsync_invert: bool,
        pub vsync_isr: esp_idf_sys::gpio_isr_t,
        pub dma_isr: esp_idf_sys::intr_handler_t,
    }

    impl Config<'_> {
        pub fn new() -> Self {
            Default::default()
        }

        #[must_use]
        pub fn dma_buffer(mut self, dma_buffer: Option<&'static [u8]>) -> Self {
            self.dma_buffer = dma_buffer;
            self
        }

        #[must_use]
        pub fn vsync_invert(mut self, vsync_invert: bool) -> Self {
            self.vsync_invert = vsync_invert;
            self
        }

        #[must_use]
        pub fn vsync_isr(mut self, vsync_isr: esp_idf_sys::gpio_isr_t) -> Self {
            self.vsync_isr = vsync_isr;
            self
        }

        #[must_use]
        pub fn dma_isr(mut self, dma_isr: esp_idf_sys::intr_handler_t) -> Self {
            self.dma_isr = dma_isr;
            self
        }
    }
}

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

        /* Interrupt registers */
        I2S_INT_RAW_REG,
        I2S_INT_ST_REG,
        I2S_INT_ENA_REG,
        I2S_INT_CLR_REG,
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
        #[cfg(esp32)]
        const fn reg_address(port: &i2s_port_t, reg: &I2sRegisterBank) -> u32 {
            #[allow(non_snake_case)]
            let mut address = match *port {
                esp_idf_sys::i2s_port_t_I2S_NUM_0 => 0x3FF4_F000,
                #[cfg(any(esp32, esp32s2))]
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
                I2sRegisterBank::I2S_INT_RAW_REG => 0x0C,
                I2sRegisterBank::I2S_INT_ST_REG => 0x10,
                I2sRegisterBank::I2S_INT_ENA_REG => 0x14,
                I2sRegisterBank::I2S_INT_CLR_REG => 0x18,
            };
            if address % 4 != 0 {
                panic!("Non-aligned register address for I2S peripheral.")
            }

            address
        }
        #[cfg(not(esp32))]
        const fn reg_address(_port: &i2s_port_t, _reg: &I2sRegisterBank) -> u32 {
            panic!("Register address calculation not implemented.");
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
pub struct DmaElement {
    _sample2: u8,
    _unused2: u8,
    _sample1: u8,
    _unused1: u8,
}

//union DmaElement {
//    sampled: DmaSample,
//    value: u32,
//}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
pub enum SamplingMode {
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s2 00 s3, 00 s3 00 s4, ...
     */
    SM_0A0B_0B0C = 0,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s3 00 s4, ...
     */
    SM_0A0B_0C0D = 1,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 00, 00 s2 00 00, 00 s3 00 00, ...
     */
    SM_0A00_0B00 = 3,
}

/// DmaFilter functions need to be placed in IRAM! (same as ISRs)
type DmaFilter = fn(dst: &mut [u8], src: &[u8], len: usize) -> usize; // todo: how to describe arrays here?

/// I2S camera slave abstraction
pub struct CameraDriver<'d> {
    i2s: i2s_port_t,
    sampling_mode: SamplingMode,

    // uint32_t dma_bytes_per_item;
    // uint32_t dma_buffer_size;
    dma_half_buffer_size: u32,
    // uint32_t dma_half_buffer_cnt;
    // uint32_t dma_node_buffer_size;
    // uint32_t dma_node_cnt;
    // uint32_t frame_copy_cnt;
    //
    // //for JPEG mode
    _dma: &'d [lldesc_t], //Vec<lldesc_t>,
    // uint8_t  *dma_buffer;
    //
    // intr_handle_t cam_intr_handle;
    //
    vsync_pin: i32, // was uin8_t

    _swap_data: bool,

    dma_filter: Option<DmaFilter>,

    config: config::Config<'d>,
    _p: PhantomData<&'d mut ()>,
}

/// Implementation for I2S0
impl<'d> CameraDriver<'d> {
    /// Create new instance of I2S controller for I2S0
    ///
    /// This ia an initial set of vars.
    #[allow(clippy::too_many_arguments)]
    #[cfg(any(esp32, esp32s2))]
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
        config: config::Config<'d>,
    ) -> Result<Self, EspError> {
        let port = <I2S0 as I2s>::port();
        crate::into_ref!(pclk, vsync, href, sd0, sd1, sd2, sd3, sd4, sd5, sd6, sd7);
        let result = Self {
            i2s: <I2S0 as I2s>::port() as _,
            sampling_mode: SamplingMode::SM_0A00_0B00,
            dma_half_buffer_size: 0,
            _dma: Default::default(), //Vec::with_capacity(0),
            vsync_pin: vsync.pin(),
            _swap_data: false,
            dma_filter: None,
            config, // todo: cleanup struct with config values
            _p: PhantomData,
        };

        // vsync INT
        /* ll_cam_set_pin */
        let io_conf = gpio_config_t {
            intr_type: if result.config.vsync_invert {
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
            gpio_install_isr_service((ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM) as i32);
            gpio_isr_handler_add(
                vsync.pin(),
                Some(result.config.vsync_isr.expect("Missing vsync isr handler.")),
                /*0 as *mut c_void*/
                core::ptr::null_mut::<c_void>(),
                //core::ptr::addr_of_mut!(&result) as *mut c_void,
            ); // todo: correct parameter
            gpio_intr_disable(vsync.pin())
        })?;

        unsafe {
            // I2S0 special functions
            gpio_matrix_in(pclk.pin() as u32, I2S0I_WS_IN_IDX, false);
            gpio_matrix_in(vsync.pin() as u32, I2S0I_V_SYNC_IDX, false);
            gpio_matrix_in(href.pin() as u32, I2S0I_H_SYNC_IDX, false);
            gpio_matrix_in(sd0.pin() as u32, I2S0I_DATA_IN0_IDX, false);
            gpio_matrix_in(sd1.pin() as u32, I2S0I_DATA_IN1_IDX, false);
            gpio_matrix_in(sd2.pin() as u32, I2S0I_DATA_IN2_IDX, false);
            gpio_matrix_in(sd3.pin() as u32, I2S0I_DATA_IN3_IDX, false);
            gpio_matrix_in(sd4.pin() as u32, I2S0I_DATA_IN4_IDX, false);
            gpio_matrix_in(sd5.pin() as u32, I2S0I_DATA_IN5_IDX, false);
            gpio_matrix_in(sd6.pin() as u32, I2S0I_DATA_IN6_IDX, false);
            gpio_matrix_in(sd7.pin() as u32, I2S0I_DATA_IN7_IDX, false);
            gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false); // What does this do?

            /* ll_cam_config */

            periph_module_enable(esp_idf_sys::periph_module_t_PERIPH_I2S0_MODULE);
        }

        // setup i2s registers for camera slave mode
        let reg_conf2 = regs::Register::new(port, regs::I2sRegisterBank::I2S_CONF2_REG);
        let reg_conf = regs::Register::new(port, regs::I2sRegisterBank::I2S_CONF_REG);
        let reg_conf_chan = regs::Register::new(port, regs::I2sRegisterBank::I2S_CONF_CHAN_REG);
        let reg_fifo_conf = regs::Register::new(port, regs::I2sRegisterBank::I2S_FIFO_CONF_REG);
        let reg_lc_conf = regs::Register::new(port, regs::I2sRegisterBank::I2S_LC_CONF_REG);

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

        // todo: ll_cam_config NOT complete; some configs missing here, please review
        reg_conf2.write(0x0000_0021); // Reset & I2S_LCD_EN & I2S_CAMERA_EN
        reg_conf.write(0x0000_F040); // Reset & I2S_RX_SLAVE_MOD & !I2S_RX_MSB_RIGHT & !I2S_RX_RIGHT_FIRST
        reg_conf_chan.write(0x0000_0008); // Reset & I2S_RX_CHAN_MOD[2:0]=1
        reg_fifo_conf.write(0x0010_1820 & ((result.sampling_mode as u32) << 16)); // Reset & I2S_RX_FIFO_MOD[2:0]=SamplingMode & I2S_RX_FIFO_MOD_FORCE_EN=1

        /* other default reg values missing/not implemented here */

        Ok(result)
    }

    ///
    /// Enables I2S0 as camera slave and enables ISRs and DMA.
    ///
    pub fn start(&self, _frame_pos: usize) -> bool {
        let port = self.i2s;

        let reg_conf = regs::Register::new(port, regs::I2sRegisterBank::I2S_CONF_REG);
        let reg_lc_conf = regs::Register::new(port, regs::I2sRegisterBank::I2S_LC_CONF_REG); // lc_conf?
        let reg_in_link = regs::Register::new(port, regs::I2sRegisterBank::I2S_IN_LINK_REG);
        let reg_rxeof_num = regs::Register::new(port, regs::I2sRegisterBank::I2S_RXEOF_NUM_REG);

        reg_conf.unset_bits(0x0000_0020); //I2S0.conf.rx_start = 0;

        self.insuceof_interrupt_enable(true); // I2S_ISR_ENABLE(in_suc_eof);

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
        reg_rxeof_num.write(self.dma_half_buffer_size / core::mem::size_of::<DmaElement>() as u32); //I2S0.rx_eof_num = cam->dma_half_buffer_size / sizeof(dma_elem_t);

        // todo:reg_in_link.write_masked(0x000f_ffff, core::ptr::addr_of!(self.dma[0]) as u32); //I2S0.in_link.addr = ((uint32_t)&cam->dma[0]) & 0xfffff;

        reg_in_link.set_bits(0x2000_0000); //I2S0.in_link.start = 1;

        reg_conf.set_bits(0x0000_0020); // I2S0.conf.rx_start = 1;

        true
    }

    ///
    /// Disables I2S0 as camera slave and disables ISRs and DMA.
    ///
    #[link_section = ".iram1"] /* IRAM_ATTR */
    pub fn stop(&self) -> bool {
        let port = <I2S0 as I2s>::port();

        let reg_conf = regs::Register::new(port, regs::I2sRegisterBank::I2S_CONF_REG);
        let reg_in_link = regs::Register::new(port, regs::I2sRegisterBank::I2S_IN_LINK_REG);

        reg_conf.unset_bits(0x0000_0020); //I2S0.conf.rx_start = 0;

        self.insuceof_interrupt_enable(false); // I2S_ISR_DISABLE(in_suc_eof);
        reg_in_link.set_bits(0x1000_0000); // I2S0.in_link.stop = 1;

        true
    }

    pub fn set_sampling_mode(&mut self, sampling_mode: SamplingMode) {
        let port = <I2S0 as I2s>::port();
        let fifo_conf = regs::Register::new(port, regs::I2sRegisterBank::I2S_FIFO_CONF_REG);

        self.sampling_mode = sampling_mode;

        let coded_sampling_mode = (sampling_mode as u32) << 16_u8; // move value to bit 16 position
        fifo_conf.write_masked(coded_sampling_mode, 0x0007_0000);
    }

    pub fn get_sampling_mode(&self) -> SamplingMode {
        self.sampling_mode
    }

    pub fn vsync_interrupt_enable(&self, enable: bool) {
        match enable {
            true => unsafe { gpio_intr_enable(self.vsync_pin) },
            _ => unsafe { gpio_intr_disable(self.vsync_pin) },
        };
    }

    fn insuceof_interrupt_enable(&self, enable: bool) {
        let port = self.i2s;
        let int_clr = regs::Register::new(port, regs::I2sRegisterBank::I2S_INT_CLR_REG);
        let int_ena = regs::Register::new(port, regs::I2sRegisterBank::I2S_INT_ENA_REG);

        match enable {
            true => {
                int_clr.set_bits(0x0000_0100); // I2S_IN_SUC_EOF_INT
                int_ena.set_bits(0x0000_0100);
            }
            _ => {
                int_ena.unset_bits(0x0000_0100);
                int_clr.set_bits(0x0000_0100);
            }
        };
    }

    pub fn cam_init_dma_isr(&self) -> Result<(), EspError> {
        esp! {
            unsafe{
                esp_intr_alloc(ETS_I2S0_INTR_SOURCE as i32, (ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM) as i32, self.config.dma_isr, /*cam*/ core::ptr::null_mut::<c_void>(), core::ptr::null_mut::<*mut esp_idf_sys::intr_handle_data_t>() /*&cam->cam_intr_handle*/)
            }
        }
    }

    /* --- non instance functions --- */

    pub fn bytes_per_sample(mode: SamplingMode) -> usize {
        match mode {
            SamplingMode::SM_0A00_0B00 => 4,
            SamplingMode::SM_0A0B_0B0C => 4,
            SamplingMode::SM_0A0B_0C0D => 2,
        }
    }

    /* --- DMA --- */
    /* TODO: add dma data transfer */

    #[link_section = ".iram1"] /* IRAM_ATTR */
    pub fn dma_memcpy(&self, output: &mut [u8], input: &[u8], len: usize) -> usize {
        //DBG_PIN_SET(1);
        let dma_filter = self.dma_filter.expect("No DMA filter function set");
        dma_filter(output, input, len)
        //DBG_PIN_SET(0);
    }
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
#[cfg(any(esp32s3, esp32))]
impl_i2s!(I2S1: esp_idf_sys::i2s_port_t_I2S_NUM_1); // It seems by default ESP_IDF is build with only one I2S enabled.
