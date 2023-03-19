use core::ptr;
///
/// TODO:
///     - Support for optional signals (hsync, rst)
///     - DMA support
///     - more than camera mode
///     - (removed) Both I2S modules 0 + 1 (currently just 0); only port0 usable for cam
///     - Clean-up Config from SPI placeholder to I2S
///
use core::{borrow::Borrow, marker::PhantomData};
use esp_idf_hal::{gpio, mutex};

use esp_idf_sys::*;

use crate::gpio::InputPin;

/// Pins used by the I2S interface
pub struct Pins<
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
    use esp_idf_hal::units::*;

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

struct Lock(i2s_port_t);

impl Lock {
    fn new(port: i2s_port_t) -> Result<Self, EspError> {
        //esp!(unsafe { spi_device_acquire_bus(port, portMAX_DELAY) })?; // TODO fix i2s

        Ok(Self(port))
    }
}

impl Drop for Lock {
    fn drop(&mut self) {
        unsafe {
            //spi_device_release_bus(self.0); // TODO fix i2s
        }
    }
}

/// Master SPI abstraction
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
    pins: Pins<VSYNC, HREF, PCLK, SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7>,
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
        pins: Pins<VSYNC, HREF, PCLK, SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7>,
        config: config::Config,
    ) -> Result<Self, EspError> {
        CameraSlave::new_internal(i2s, pins, config)
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
    /// Internal implementation of new shared by all SPI controllers
    fn new_internal(
        i2s: I2S,
        pins: Pins<VSYNC, HREF, PCLK, SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7>,
        config: config::Config,
    ) -> Result<Self, EspError> {
        let i2s_config = i2s_config_t {
            mode: i2s_mode_t_I2S_MODE_MASTER | i2s_mode_t_I2S_MODE_TX,
            sample_rate: 44100,
            bits_per_sample: 16,
            channel_format: i2s_channel_fmt_t_I2S_CHANNEL_FMT_RIGHT_LEFT,
            communication_format: i2s_comm_format_t_I2S_COMM_FORMAT_STAND_I2S,
            intr_alloc_flags: 0, // default interrupt priority
            dma_buf_count: 8,
            dma_buf_len: 64,
            use_apll: false,

            ..Default::default()
        };

        esp!(unsafe {
            i2s_driver_install(I2S::port(), &i2s_config, 0, ptr::null_mut()) /*TODO: DMA support*/
        })?;

        let pin_config = i2s_pin_config_t {
            bck_io_num: 26,
            ws_io_num: 25,
            data_out_num: 22,
            data_in_num: 23,

            ..Default::default()
        };

        esp!(unsafe { i2s_set_pin(I2S::port(), &pin_config) })?;

        /*
        // TODO: DMA and samplerate config? i2s_set_dac_mode
        esp!(unsafe {
            i2s_set_dac_mode(I2S::port(), ...)
        })?;

        */

        Ok(Self {
            i2s,
            pins,
            port: I2S::port(),
        })
    }
    /*
    /// Release and return the raw interface to the underlying SPI peripheral
    #[allow(clippy::type_complexity)]
    pub fn release(self) -> Result<(SPI, Pins<SCLK, SDO, SDI, CS>), EspError> {
        esp!(unsafe { spi_bus_remove_device(self.device) })?;
        esp!(unsafe { spi_bus_free(SPI::device()) })?;

        Ok((self.spi, self.pins))
    }

    fn lock_bus(&mut self) -> Result<Lock, SpiError> {
        Lock::new(self.device).map_err(SpiError::other)
    }

    fn lock_bus_for(&mut self, lock_bus: bool, size: usize) -> Result<Option<Lock>, SpiError> {
        if lock_bus && size > TRANS_LEN {
            Ok(Some(self.lock_bus()?))
        } else {
            Ok(None)
        }
    }

    fn transfer_internal(
        &mut self,
        read: &mut [u8],
        write: &[u8],
        lock_bus: bool,
    ) -> Result<(), SpiError> {
        let _lock = self.lock_bus_for(lock_bus, max(read.len(), write.len()))?;

        let len = max(read.len(), write.len());
        for offset in (0..len).step_by(TRANS_LEN) {
            let read_chunk_end = min(offset + TRANS_LEN, read.len());
            let write_chunk_end = min(offset + TRANS_LEN, write.len());

            if read_chunk_end != write_chunk_end {
                let mut buf = [0_u8; TRANS_LEN];

                let write_ptr = if write_chunk_end < offset + TRANS_LEN {
                    if write_chunk_end > offset {
                        buf[0..write_chunk_end - offset]
                            .copy_from_slice(&write[offset..write_chunk_end]);
                    }

                    buf.as_ptr()
                } else {
                    let chunk = &write[offset..write_chunk_end];

                    chunk.as_ptr()
                };

                let read_ptr = if read_chunk_end < offset + TRANS_LEN {
                    buf.as_mut_ptr()
                } else {
                    let chunk = &mut read[offset..read_chunk_end];

                    chunk.as_mut_ptr()
                };

                let transfer_len = max(read_chunk_end, write_chunk_end) - offset;

                self.transfer_internal_raw(read_ptr, transfer_len, write_ptr, transfer_len)?;

                if read_chunk_end > offset && read_chunk_end < offset + TRANS_LEN {
                    read[offset..read_chunk_end].copy_from_slice(&buf[0..read_chunk_end - offset]);
                }
            } else {
                let read_chunk = &mut read[offset..read_chunk_end];
                let write_chunk = &write[offset..write_chunk_end];

                self.transfer_internal_raw(
                    read_chunk.as_mut_ptr(),
                    read_chunk.len(),
                    write_chunk.as_ptr(),
                    write_chunk.len(),
                )?;
            }
        }

        Ok(())
    }

    fn transfer_inplace_internal(
        &mut self,
        data: &mut [u8],
        lock_bus: bool,
    ) -> Result<(), SpiError> {
        let _lock = self.lock_bus_for(lock_bus, data.len())?;

        let total_len = data.len();
        for offset in (0..data.len()).step_by(TRANS_LEN) {
            let chunk = &mut data[offset..min(offset + TRANS_LEN, total_len)];
            let len = chunk.len();
            let ptr = chunk.as_mut_ptr();

            self.transfer_internal_raw(ptr, len, ptr, len)?;
        }

        Ok(())
    }

    fn write_iter_internal<WI>(&mut self, words: WI) -> Result<(), SpiError>
    where
        WI: IntoIterator<Item = u8>,
    {
        let mut words = words.into_iter();

        let mut buf = [0_u8; TRANS_LEN];

        let mut lock = None;

        loop {
            let mut offset = 0_usize;

            while offset < buf.len() {
                if let Some(word) = words.next() {
                    buf[offset] = word;
                    offset += 1;
                } else {
                    break;
                }
            }

            if offset == 0 {
                break;
            }

            if offset == buf.len() && lock.is_none() {
                lock = Some(self.lock_bus()?);
            }

            let chunk = &mut buf[..offset];
            let ptr = chunk.as_mut_ptr();

            self.transfer_internal_raw(ptr, chunk.len(), ptr, chunk.len())?;
        }

        Ok(())
    }

    fn transfer_internal_raw(
        &mut self,
        read: *mut u8,
        read_len: usize,
        write: *const u8,
        write_len: usize,
    ) -> Result<(), SpiError> {
        let mut transaction = spi_transaction_t {
            flags: 0,
            __bindgen_anon_1: spi_transaction_t__bindgen_ty_1 {
                tx_buffer: write as *const _,
            },
            __bindgen_anon_2: spi_transaction_t__bindgen_ty_2 {
                rx_buffer: read as *mut _,
            },
            length: (write_len * 8) as _,
            rxlength: (read_len * 8) as _,
            ..Default::default()
        };

        esp!(unsafe { spi_device_polling_transmit(self.device, &mut transaction as *mut _) })
            .map_err(SpiError::other)?;

        Ok(())
    }*/
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

impl_i2s!(I2S0: i2s_port_t_I2S_NUM_0);
impl_i2s!(I2S1: i2s_port_t_I2S_NUM_1);

pub struct I2sPeripherals {
    pub i2s0: I2S0,
    pub i2s1: I2S1,
}

static TAKEN: mutex::Mutex<bool> = mutex::Mutex::new(false);

impl I2sPeripherals {
    pub fn take() -> Option<Self> {
        let mut taken = TAKEN.lock();

        if *taken {
            None
        } else {
            *taken = true;
            Some(unsafe { I2sPeripherals::new() })
        }
    }

    /// # Safety
    ///
    /// Care should be taken not to instantiate the Peripherals structure, if it is already instantiated and used elsewhere
    pub unsafe fn new() -> Self {
        Self {
            i2s0: I2S0::new(),
            i2s1: I2S1::new(),
        }
    }
}
