use esp_idf_hal::i2s;

use esp_idf_sys::EspError;

use crate::sensor::*;

static LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE: u32 = 4092;
static CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX: u32 = 4092;

// This needs to be abstracted for the usage with the driver
pub struct LlCamera {
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

pub fn calc_rgb_dma(cam: &mut LlCamera) -> bool {
    let dma_half_buffer_max: u32 =
        (CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX / 2 / cam.dma_bytes_per_item) as u32;
    let dma_buffer_max = 2 * dma_half_buffer_max;
    let node_max: u32 = (LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE / cam.dma_bytes_per_item) as u32;

    let line_width: u32 = (cam.width * cam.in_bytes_per_pixel as u16) as u32;
    let image_size: u32 = (cam.height as u32 * line_width) as u32;
    if image_size > (4 * 1024 * 1024) || (line_width > dma_half_buffer_max) {
        //ESP_LOGE(TAG, "Resolution too high");
        return false;
    }

    let mut node_size = node_max;
    let mut nodes_per_line = 1;
    let mut lines_per_node = 1;
    let mut lines_per_half_buffer = 1;
    let mut dma_half_buffer_min = node_max;
    let mut dma_half_buffer = dma_half_buffer_max;
    let mut dma_buffer_size = dma_buffer_max;

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
                while (cam.height % lines_per_node as u16) != 0 {
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
    while (cam.height % lines_per_half_buffer as u16) != 0 {
        dma_half_buffer = dma_half_buffer - dma_half_buffer_min;
        lines_per_half_buffer = dma_half_buffer / line_width;
    }
    // Calculate DMA size
    dma_buffer_size = (dma_buffer_max / dma_half_buffer) * dma_half_buffer;

    //        ESP_LOGI(TAG, "node_size: %4u, nodes_per_line: %u, lines_per_node: %u, dma_half_buffer_min: %5u, dma_half_buffer: %5u, lines_per_half_buffer: %2u, dma_buffer_size: %5u, image_size: %u",
    //                 node_size * cam.dma_bytes_per_item, nodes_per_line, lines_per_node,
    //                 dma_half_buffer_min * cam.dma_bytes_per_item, dma_half_buffer * cam.dma_bytes_per_item,
    //                 lines_per_half_buffer, dma_buffer_size * cam.dma_bytes_per_item, image_size);

    cam.dma_buffer_size = (dma_buffer_size * cam.dma_bytes_per_item) as u32;
    cam.dma_half_buffer_size = (dma_half_buffer * cam.dma_bytes_per_item) as u32;
    cam.dma_node_buffer_size = (node_size * cam.dma_bytes_per_item) as u32;
    cam.dma_half_buffer_cnt = cam.dma_buffer_size / cam.dma_half_buffer_size;
    return true;
}

fn dma_sizes(cam: &mut LlCamera) -> bool {
    //cam.dma_bytes_per_item = i2s::CameraDriver::bytes_per_sample(sampling_mode); // todo: get from camera driver
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

/*    fn ll_cam_dma_filter_jpeg(dst: &mut [u8], src: &[u8], len: usize) -> usize {
        let dma_el: &i2s::DmaElement = src;
        let elements = len / core::mem::size_of::<i2s::DmaElement>();
        let end = (elements / 4) * 4;
        // manually unrolling 4 iterations of the loop here
        for i in (0..end).step_by(4) {
            dst[i + 0] = dma_el[i + 0].sample1;
            dst[i + 1] = dma_el[i + 1].sample1;
            dst[i + 2] = dma_el[i + 2].sample1;
            dst[i + 3] = dma_el[i + 3].sample1;
        }
        elements
    }
*/
fn set_sample_mode(
    cam: &mut LlCamera,
    pix_format: PixFormat,
    xclk_freq_hz: u32,
    sensor_pid: u16,
) -> Result<i2s::SamplingMode, EspError> {
    /*        if pix_format == PixformatGrayscale {
        if sensor_pid == Ov3660Pid
            || sensor_pid == Ov5640Pid
            || sensor_pid == Nt99141Pid
            || sensor_pid == Sc031gsPid
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
            if xclk_freq_hz > 10000000 && sensor_pid != Ov7725Pid {
                sampling_mode = i2s::SamplingMode::SM_0A00_0B00;
                dma_filter = ll_cam_dma_filter_grayscale_highspeed;
            } else {
                sampling_mode = i2s::SamplingMode::SM_0A0B_0C0D;
                dma_filter = ll_cam_dma_filter_grayscale;
            }
            cam.in_bytes_per_pixel = 2; // camera sends YU/YV
        }
        cam.fb_bytes_per_pixel = 1; // frame buffer stores Y8
    } else if pix_format == PixformatYuv422 || pix_format == PixformatRgb565 {
        if xclk_freq_hz > 10000000 && sensor_pid != Ov7725Pid {
            if sensor_pid == Ov7670Pid {
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
    } else*/
    match pix_format {
        PixFormat::PixformatJpeg => {
            cam.in_bytes_per_pixel = 1;
            cam.fb_bytes_per_pixel = 1;
            //dma_filter = ll_cam_dma_filter_jpeg;
            //sampling_mode = i2s::SamplingMode::SM_0A00_0B00;
        }
        _ => {
            //ESP_LOGE(TAG, "Requested format is not supported");
            //return Err(ESP_ERR_NOT_SUPPORTED);
        }
    }
    //I2S0.fifo_conf.rx_fifo_mod = sampling_mode; // TODO: correct reg access / set sampling mode
    Ok(i2s::SamplingMode::SM_0A00_0B00)
}
