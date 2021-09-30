// HDMI output & MIPI Sample
//
// Requirements:
//   1.CAMERA_RASPBERRY_PI
//   2.Screen
//
// Please set the value of "camera-type" in "mbed_app.json" to null or "CAMERA_RASPBERRY_PI".
// Please set the value of "lcd-type" in "mved_app.json" to "RGB_TO_HDMI"

#include "mbed.h"
#include "SdUsbConnect.h"
#include "EasyAttach_CameraAndLCD.h"
#include "AsciiFont.h"
#include "r_dk2_if.h"
#include "r_drp_simple_isp.h"
#include "face_detector.hpp"
#include "r_cache_lld_rza2m.h"
#include "FATFileSystem.h"

#if !defined(TARGET_RZ_A2XX)
#error "DRP and MIPI are not supported."
#endif
#if MBED_CONF_APP_CAMERA_TYPE != CAMERA_RASPBERRY_PI_832X480
#error Please set the value of "camera-type" in "mbed_app.json" to "CAMERA_RASPBERRY_PI" and build.
#endif

// Frame buffer stride should be set to a multiple of 32 or 128
#define VIDEO_PIXEL_HW         (832u)
#define VIDEO_PIXEL_VW         (480u)

#define FRAME_BUFFER_STRIDE    (((VIDEO_PIXEL_HW * 1) + 31u) & ~31u)
#define FRAME_BUFFER_STRIDE_2  (((VIDEO_PIXEL_HW * 2) + 31u) & ~31u)
#define FRAME_BUFFER_HEIGHT    (VIDEO_PIXEL_VW)

// #define OVL_WIDTH              (AsciiFont::CHAR_PIX_WIDTH * 2 * 11)
// #define OVL_STRIDE             (((OVL_WIDTH * 1) + 31u) & ~31u)
// #define OVL_HEIGHT             (AsciiFont::CHAR_PIX_HEIGHT * 2)

#define DRP_FLG_CAMER_IN       (0x00000100)
#define DRP_NOT_FINISH         (0)
#define DRP_FINISH             (1)
#define TILE_0                 (0)

#define RESULT_BUFFER_BYTE_PER_PIXEL  (2u)

#define FACE_DETECTOR_MODEL     "/storage/lbpcascade_frontalface.xml"
#define MOUNT_NAME              "storage"
#define FILE_NAME_LEN           (64)

#define WB_DAYLIGHT            (0)
#define WB_CLOUDY              (1)
#define WB_FLUORESCENT         (2)
#define WB_TUNGSTEN            (3)

using namespace cv;

static DisplayBase Display;
static uint8_t fbuf_bayer[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((aligned(128)));
static uint8_t fbuf_yuv[FRAME_BUFFER_STRIDE_2 * FRAME_BUFFER_HEIGHT]__attribute((aligned(32)));
// static uint8_t fbuf_overlay[OVL_STRIDE * OVL_HEIGHT]__attribute((section("NC_BSS"),aligned(32)));
static uint8_t fbuf_grayscale[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((aligned(32)));

static r_drp_simple_isp_t param_isp __attribute((section("NC_BSS")));
static void cb_drp_finish(uint8_t id);
static uint8_t drp_lib_id[R_DK2_TILE_NUM] = {0};
static volatile uint8_t drp_lib_status[R_DK2_TILE_NUM] = {DRP_NOT_FINISH};
static Thread drpTask(osPriorityHigh,(1024*33));
static Timer g_detect_timer;
static Rect copysquare;
static bool draw_square = false;

static uint32_t isp_wb_mode_req;
static uint32_t isp_wb_mode;

static const uint32_t clut_data_resut[] = {0x00000000, 0xff00ff00};  // ARGB8888


static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type) {
    drpTask.flags_set(DRP_FLG_CAMER_IN);
}

static void cb_drp_finish(uint8_t id){
    uint32_t tile_no;
    /* Change the operation state of the DRP library notified by the argument to finish */
    for (tile_no = 0; tile_no < R_DK2_TILE_NUM; tile_no++ )
    {
        if (drp_lib_id[tile_no] == id)
        {
            drp_lib_status[tile_no] = DRP_FINISH;
            break;
        }
        else
        {
            /* DO NOTHING */
        }
    }
    return;
}

static void Start_Video_Camera(void) {
    // Video capture setting (progressive form fixed)
    Display.Video_Write_Setting(
        DisplayBase::VIDEO_INPUT_CHANNEL_0,
        DisplayBase::COL_SYS_NTSC_358,
        (void *)fbuf_bayer,
        FRAME_BUFFER_STRIDE,
        DisplayBase::VIDEO_FORMAT_RAW8,
        DisplayBase::WR_RD_WRSWA_NON,
        VIDEO_PIXEL_VW,
        VIDEO_PIXEL_HW
    );
    EasyAttach_CameraStart(Display, DisplayBase::VIDEO_INPUT_CHANNEL_0);
}

static void Start_LCD_Display(void) {
    DisplayBase::rect_t rect;
    // DisplayBase::clut_t clut_param; // 濾鏡名稱區塊

    rect.vs = 100;
    rect.vw = 480;
    rect.hs = 50;
    rect.hw = 800;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_0,
        (void *)fbuf_yuv,
        FRAME_BUFFER_STRIDE_2,
        DisplayBase::GRAPHICS_FORMAT_YCBCR422,
        DisplayBase::WR_RD_WRSWA_32_16_8BIT,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_0);

    // memset(fbuf_overlay, 0, sizeof(fbuf_overlay));
    // clut_param.color_num = sizeof(clut_data_resut) / sizeof(uint32_t);
    // clut_param.clut = clut_data_resut;

    // rect.vs = 5;
    // rect.vw = OVL_HEIGHT;
    // rect.hs = 5;
    // rect.hw = OVL_WIDTH;
    // Display.Graphics_Read_Setting(
    //     DisplayBase::GRAPHICS_LAYER_2,
    //     (void *)fbuf_overlay,
    //     OVL_STRIDE,
    //     DisplayBase::GRAPHICS_FORMAT_CLUT8,
    //     DisplayBase::WR_RD_WRSWA_32_16_8BIT,
    //     &rect,
    //     &clut_param
    // );
    // Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_2);

    ThisThread::sleep_for(chrono::milliseconds(50));
    EasyAttach_LcdBacklight(true);
}

static void drawsquare(int x, int y, int w, int h, uint32_t const colour){
    uint32_t idx_base;
    uint32_t wk_idx;
    uint32_t i;
    uint32_t coller_pix[RESULT_BUFFER_BYTE_PER_PIXEL];  /* YCbCr 422 */

    idx_base = ((x & 0xfffffffe) + (VIDEO_PIXEL_HW * y)) * RESULT_BUFFER_BYTE_PER_PIXEL;

    /* Select color */
    coller_pix[0] = (colour >> 0) & 0xff;   
    coller_pix[1] = (colour >> 8) & 0xff;   
    coller_pix[2] = (colour >> 16) & 0xff;  
    coller_pix[3] = (colour >> 24) & 0xff;  

    /* top */
    wk_idx = idx_base;
    for (i = 0; (int)i < w; i += 2)
    {
        fbuf_yuv[wk_idx++ ] = coller_pix[0];
        fbuf_yuv[wk_idx++ ] = coller_pix[1];
        fbuf_yuv[wk_idx++ ] = coller_pix[2];
        fbuf_yuv[wk_idx++ ] = coller_pix[3];
    }

    /* middle */
    for (i = 1; (int)i < (h - 1); i++ )
    {
        wk_idx = idx_base + (VIDEO_PIXEL_HW * RESULT_BUFFER_BYTE_PER_PIXEL * i);
        fbuf_yuv[wk_idx + 0] = coller_pix[0];
        fbuf_yuv[wk_idx + 1] = coller_pix[1];
        fbuf_yuv[wk_idx + 2] = coller_pix[2];
        fbuf_yuv[wk_idx + 3] = coller_pix[3];
        wk_idx += (((w & 0xfffffffe) - 2) * RESULT_BUFFER_BYTE_PER_PIXEL);
        fbuf_yuv[wk_idx + 0] = coller_pix[0];
        fbuf_yuv[wk_idx + 1] = coller_pix[1];
        fbuf_yuv[wk_idx + 2] = coller_pix[2];
        fbuf_yuv[wk_idx + 3] = coller_pix[3];
    }

    /* bottom */
    wk_idx = idx_base + (VIDEO_PIXEL_HW * RESULT_BUFFER_BYTE_PER_PIXEL * (h - 1));
    for (i = 0; (int)i < w; i += 2 )
    {
        fbuf_yuv[wk_idx++ ] = coller_pix[0];
        fbuf_yuv[wk_idx++ ] = coller_pix[1];
        fbuf_yuv[wk_idx++ ] = coller_pix[2];
        fbuf_yuv[wk_idx++ ] = coller_pix[3];
    }
    draw_square = true;
}

// static void button_fall(void) {
//     if (isp_wb_mode_req < WB_TUNGSTEN) {
//         isp_wb_mode_req++;
//     } else {
//         isp_wb_mode_req = WB_DAYLIGHT;
//     }
// }

static void drp_task(void) {
    uint32_t counter = 0;
    
    // Display initial
    EasyAttach_Init(Display);

    // DRP initial
    R_DK2_Initialize();

    // Waiting for SD & USB insertion
    printf("Finding a storage...");
    SdUsbConnect storage("storage");
    storage.wait_connect();
    printf("done\n");

    // Face detection initial
    detectFaceInit(FACE_DETECTOR_MODEL);
    g_detect_timer.reset();
    g_detect_timer.start();

    // Display output start
    Start_LCD_Display();

    // Camera activation process
    Display.Graphics_Irq_Handler_Set(DisplayBase::INT_TYPE_S0_VFIELD, 0, IntCallbackFunc_Vfield);
    Start_Video_Camera();

    copysquare.width = 0;
    copysquare.height = 0;
    while (true)
    {
        /* Camera image acquisition process */
        ThisThread::flags_wait_all(DRP_FLG_CAMER_IN);
        /************************************/
        /* Load DRP Library                 */
        /*        +-----------------------+ */
        /* tile 0 |                       | */
        /*        +                       + */
        /* tile 1 |                       | */
        /*        +                       + */
        /* tile 2 |                       | */
        /*        + SimpleIsp bayer2yuv_6 + */
        /* tile 3 |                       | */
        /*        +                       + */
        /* tile 4 |                       | */
        /*        +                       + */
        /* tile 5 |                       | */
        /*        +-----------------------+ */
        R_DK2_Load(g_drp_lib_simple_isp_bayer2yuv_6,
                R_DK2_TILE_0,
                R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);

        /************************/
        /* Activate DRP Library */
        /************************/
        R_DK2_Activate(0, 0);

        /* Set ISP parameters */
        memset(&param_isp, 0, sizeof(param_isp));
        /* ISP source: camera image buffer */
        param_isp.src    = (uint32_t)fbuf_bayer; 
        /* ISP destination: output image buffer on monitor */
        param_isp.dst    = (uint32_t)fbuf_yuv;
        param_isp.width  = VIDEO_PIXEL_HW;
        param_isp.height = VIDEO_PIXEL_VW;
        param_isp.gain_r = 0x1266;
        param_isp.gain_g = 0x0CB0;
        param_isp.gain_b = 0x1359;

        drp_lib_status[TILE_0] = DRP_NOT_FINISH;

        /*********************/
        /* Start DRP Library */
        /*********************/
        R_DK2_Start(drp_lib_id[0], (void *)&param_isp, sizeof(r_drp_simple_isp_t));

        /***************************************/
        /* Wait until DRP processing is finish */
        /***************************************/
        while (drp_lib_status[TILE_0] == DRP_NOT_FINISH)
        {
            /* Spin here forever.. */
        }

        /**********************/
        /* Unload DRP Library */
        /**********************/
        R_DK2_Unload(drp_lib_id[TILE_0], &drp_lib_id[0]);

        if ((copysquare.width > 0) && (copysquare.height > 0) && (counter > 0) )
            {
                counter--;
                R_CACHE_L1DataInvalidLine(fbuf_yuv,sizeof(fbuf_yuv));
                drawsquare(copysquare.x, copysquare.y, copysquare.width, copysquare.height, 0x4CF04C55);
                R_CACHE_L1DataCleanInvalidLine(fbuf_yuv,sizeof(fbuf_yuv)); 
            }
        /************************************/
        /* Load DRP Library                 */
        /*        +-----------------------+ */
        /* tile 0 |                       | */
        /*        +                       + */
        /* tile 1 |                       | */
        /*        +                       + */
        /* tile 2 |                       | */
        /*        + SimpleIsp bayer2grayscale_6 + */
        /* tile 3 |                       | */
        /*        +                       + */
        /* tile 4 |                       | */
        /*        +                       + */
        /* tile 5 |                       | */
        /*        +-----------------------+ */
        R_DK2_Load(g_drp_lib_simple_isp_bayer2grayscale_6,
                R_DK2_TILE_0,
                R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);

        /************************/
        /* Activate DRP Library */
        /************************/
        R_DK2_Activate(0, 0);

        /* Set ISP parameters */
        memset(&param_isp, 0, sizeof(param_isp));
        /* ISP source: camera image buffer */
        param_isp.src    = (uint32_t)fbuf_bayer;
        /* ISP destination: Grayscale image buffer for face detecting */
        param_isp.dst    = (uint32_t)fbuf_grayscale;
        param_isp.width  = VIDEO_PIXEL_HW;
        param_isp.height = VIDEO_PIXEL_VW;
        param_isp.gain_r = 0x1266;
        param_isp.gain_g = 0x0CB0;
        param_isp.gain_b = 0x1359;

        /* Initialize variables to be used in termination judgment of the DRP library */
        drp_lib_status[TILE_0] = DRP_NOT_FINISH;

        /*********************/
        /* Start DRP Library */
        /*********************/
        R_DK2_Start(drp_lib_id[0], (void *)&param_isp, sizeof(r_drp_simple_isp_t));

        /***************************************/
        /* Wait until DRP processing is finish */
        /***************************************/
        while (drp_lib_status[TILE_0] == DRP_NOT_FINISH)
        {
            /* DO NOTHING */
        }

        /**********************/
        /* Unload DRP Library */
        /**********************/
        R_DK2_Unload(drp_lib_id[TILE_0], &drp_lib_id[0]);

        
        /* Cache invalidate operation */
        R_CACHE_L1DataInvalidLine(fbuf_grayscale,sizeof(fbuf_grayscale));
        /* Call OpenCV face detection library */
        /* Detect a face in the frame */
        Mat img_gray(VIDEO_PIXEL_VW, VIDEO_PIXEL_HW, CV_8U, fbuf_grayscale);
        Rect face_roi;
        detectFace(img_gray, face_roi);

        /* A face is detected */
        if ((face_roi.width > 0) && (face_roi.height > 0))
        {
            counter = 5;
            printf("Detected a face X:%d Y:%d W:%d H:%d\n",face_roi.x, face_roi.y, face_roi.width, face_roi.height);
            copysquare = face_roi; 
        }
        else
        {
            printf("No face is detected\n");
        } 
    }

    // ====================================

    // AsciiFont ascii_font(fbuf_overlay, OVL_WIDTH, OVL_HEIGHT, OVL_STRIDE, 1);
    // char str[64];

    // /* Load DRP Library                 */
    // /*        +-----------------------+ */
    // /* tile 0 |                       | */
    // /*        +                       + */
    // /* tile 1 |                       | */
    // /*        +                       + */
    // /* tile 2 |                       | */
    // /*        + SimpleIsp bayer2yuv_6 + */
    // /* tile 3 |                       | */
    // /*        +                       + */
    // /* tile 4 |                       | */
    // /*        +                       + */
    // /* tile 5 |                       | */
    // /*        +-----------------------+ */
    // R_DK2_Load(g_drp_lib_simple_isp_bayer2yuv_6,
    //            R_DK2_TILE_0,
    //            R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);
    // R_DK2_Activate(0, 0);

    // memset(&param_isp, 0, sizeof(param_isp));
    // param_isp.src    = (uint32_t)fbuf_bayer;
    // param_isp.dst    = (uint32_t)fbuf_yuv;
    // param_isp.width  = VIDEO_PIXEL_HW;
    // param_isp.height = VIDEO_PIXEL_VW;
    // param_isp.gain_r = 0x1266;
    // param_isp.gain_g = 0x0CB0;
    // param_isp.gain_b = 0x1359;

    // isp_wb_mode_req = WB_DAYLIGHT;
    // isp_wb_mode = WB_DAYLIGHT;
    // sprintf(str, "Daylight");
    // ascii_font.DrawStr(str, 0, 0 + (AsciiFont::CHAR_PIX_HEIGHT + 1) * 0, 1, 2);

    // InterruptIn button(USER_BUTTON0);
    // button.fall(&button_fall);

    // while (true) {
    //     ThisThread::flags_wait_all(DRP_FLG_CAMER_IN);
    //     if (isp_wb_mode_req != isp_wb_mode) {
    //         isp_wb_mode = isp_wb_mode_req;
    //         switch (isp_wb_mode) {
    //             case WB_DAYLIGHT:
    //                 param_isp.gain_r = 0x1266;
    //                 param_isp.gain_g = 0x0CB0;
    //                 param_isp.gain_b = 0x1359;
    //                 sprintf(str, "Daylight");
    //                 break;
    //             case WB_CLOUDY:
    //                 param_isp.gain_r = 0x1400;
    //                 param_isp.gain_g = 0x0CB0;
    //                 param_isp.gain_b = 0x11CA;
    //                 sprintf(str, "Cloudy");
    //                 break;
    //             case WB_FLUORESCENT:
    //                 param_isp.gain_r = 0x1000;
    //                 param_isp.gain_g = 0x0CB0;
    //                 param_isp.gain_b = 0x163D;
    //                 sprintf(str, "Fluorescent");
    //                 break;
    //             case WB_TUNGSTEN:
    //                 param_isp.gain_r = 0x0E66;
    //                 param_isp.gain_g = 0x0CB0;
    //                 param_isp.gain_b = 0x1876;
    //                 sprintf(str, "Tungsten");
    //                 break;
    //         }
    //         memset(fbuf_overlay, 0, sizeof(fbuf_overlay));
    //         ascii_font.DrawStr(str, 0, 0 + (AsciiFont::CHAR_PIX_HEIGHT + 1) * 0, 1, 2);
    //     }
    //     R_DK2_Start(drp_lib_id[0], (void *)&param_isp, sizeof(r_drp_simple_isp_t));
    // }
}

int main(void) {
    // Start DRP task
    drpTask.start(callback(drp_task));

    // ThisThread::sleep_for(osWaitForever);
    ThisThread::sleep_for(chrono::seconds(10));
}