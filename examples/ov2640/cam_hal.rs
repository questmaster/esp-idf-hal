use crate::sensor::FrameSize;
use esp_idf_sys::TickType_t;

struct Camera {}

impl Camera {
    pub fn new() -> Self {
        Self {}
    }

    pub fn config(&mut self, frame_size: FrameSize, sensor_pid: u16) {}

    pub fn start(&self) {}
    pub fn stop(&self) {}
    pub fn take(&self, timeout: TickType_t) {}
    pub fn give(&self /*, camera_fb_t dma_buffer */) {}

    /*
    Missing func:
    - give_all
     */
}

impl Drop for Camera {
    fn drop(&mut self) {
        todo!()
    }
}
