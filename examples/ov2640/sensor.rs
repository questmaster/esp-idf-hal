pub enum CameraPid {
    Ov9650Pid = 0x96,
    Ov7725Pid = 0x77,
    Ov2640Pid = 0x26,
    Ov3660Pid = 0x3660,
    Ov5640Pid = 0x5640,
    Ov7670Pid = 0x76,
    Nt99141Pid = 0x1410,
    Gc2145Pid = 0x2145,
    Gc032aPid = 0x232a,
    Gc0308Pid = 0x9b,
    Bf3005Pid = 0x30,
    Bf20a6Pid = 0x20a6,
    Sc101iotPid = 0xda4a,
    Sc030iotPid = 0x9a46,
    Sc031gsPid = 0x0031,
}

pub enum PixFormat {
    PixformatRgb565,    // 2BPP/RGB565
    PixformatYuv422,    // 2BPP/YUV422
    PixformatYuv420,    // 1.5BPP/YUV420
    PixformatGrayscale, // 1BPP/GRAYSCALE
    PixformatJpeg,      // JPEG/COMPRESSED
    PixformatRgb888,    // 3BPP/RGB888
    PixformatRaw,       // RAW
    PixformatRgb444,    // 3BP2P/RGB444
    PixformatRgb555,    // 3BP2P/RGB555
}

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
