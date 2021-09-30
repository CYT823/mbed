/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized. This
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
* Copyright (C) 2012 - 2015 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
#include <string.h>
#include "DisplayBace.h"
#include "gr_board_vdc5.h"
#if defined(TARGET_RZ_A2XX)
#include "r_spea.h"
#endif

/**************************************************************************//**
 * @brief       Constructor of the DisplayBase class
 * @param[in]   None
 * @retval      None
******************************************************************************/
DisplayBase::DisplayBase( void )
{
    /* Lcd setting (default) */
    _lcd_config.lcd_type             = LCD_TYPE_PARALLEL_RGB;       /* LVDS or Pararel RGB                      */
    _lcd_config.intputClock          = 66.67f;                      /* P1  clk [MHz] ex. 66.67                  */
    _lcd_config.outputClock          = 40.00f;                      /* LCD clk [MHz] ex. 33.33                  */

    _lcd_config.lcd_outformat        = LCD_OUTFORMAT_RGB888;        /* Output format select */
    _lcd_config.lcd_edge             = EDGE_FALLING;                /* Output phase control of LCD_DATA23 to LCD_DATA0 pin */

    _lcd_config.h_toatal_period      = (800u + 40u + 128u+ 88u);    /* Free-running Hsync period                */
    _lcd_config.v_toatal_period      = (600u + 1u + 4u + 23u);      /* Free-running Vsync period                */
    _lcd_config.h_disp_widht         = 800u;                        /* LCD display area size, horizontal width  */
    _lcd_config.v_disp_widht         = 600u;                        /* LCD display area size, vertical width    */
    _lcd_config.h_back_porch         = (128u+ 88u);                 /* LCD display horizontal back porch period */
    _lcd_config.v_back_porch         = (4u + 23u);                  /* LCD display vertical back porch period   */

    _lcd_config.h_sync_port          = LCD_TCON_PIN_0;              /* TCONn or Not use(-1)                     */
    _lcd_config.h_sync_port_polarity = SIG_POL_NOT_INVERTED;        /* Polarity inversion control of signal     */
    _lcd_config.h_sync_width         = 128u;                        /* Hsync width                              */

    _lcd_config.v_sync_port          = LCD_TCON_PIN_1;              /* TCONn or Not use(-1)                     */
    _lcd_config.v_sync_port_polarity = SIG_POL_NOT_INVERTED;        /* Polarity inversion control of signal     */
    _lcd_config.v_sync_width         = 4u;                          /* Vsync width                              */

    _lcd_config.de_port              = LCD_TCON_PIN_NON;            /* TCONn or Not use(-1)                     */
    _lcd_config.de_port_polarity     = SIG_POL_NOT_INVERTED;        /* Polarity inversion control of signal     */

    /* Digital video input setting (default) */
    _video_input_sel                    = INPUT_SEL_VDEC;           /* Video decoder output signals */
    _video_ext_in_config.inp_format     = VIDEO_EXTIN_FORMAT_BT601;
    _video_ext_in_config.inp_pxd_edge   = EDGE_RISING;              /* Clock edge select for capturing data          */
    _video_ext_in_config.inp_vs_edge    = EDGE_RISING;              /* Clock edge select for capturing Vsync signals */
    _video_ext_in_config.inp_hs_edge    = EDGE_RISING;              /* Clock edge select for capturing Hsync signals */
    _video_ext_in_config.inp_endian_on  = OFF;                      /* External input bit endian change on/off       */
    _video_ext_in_config.inp_swap_on    = OFF;                      /* External input B/R signal swap on/off         */
    _video_ext_in_config.inp_vs_inv     = SIG_POL_NOT_INVERTED;     /* External input DV_VSYNC inversion control     */
    _video_ext_in_config.inp_hs_inv     = SIG_POL_INVERTED;         /* External input DV_HSYNC inversion control     */
    _video_ext_in_config.inp_f525_625   = EXTIN_LINE_525;           /* Number of lines for BT.656 external input     */
    _video_ext_in_config.inp_h_pos      = EXTIN_H_POS_CRYCBY;       /* Y/Cb/Y/Cr data string start timing to Hsync reference */
    _video_ext_in_config.cap_vs_pos     = 6u;                       /* Capture start position from Vsync             */
    _video_ext_in_config.cap_hs_pos     = 302u;                     /* Capture start position form Hsync             */
    _video_ext_in_config.cap_width      = 640u;                     /* Capture width                                 */
    _video_ext_in_config.cap_height     = 468u;                     /* Capture height should be a multiple of 4      */

#if defined(TARGET_RZ_A2XX)
    /* mipi */
    _video_mipi_config.mipi_lanenum    = 2;
    _video_mipi_config.mipi_vc         = 0;
    _video_mipi_config.mipi_interlace  = 0;
    _video_mipi_config.mipi_laneswap   = 0; /* Progressive */
    _video_mipi_config.mipi_frametop   = 0;
    _video_mipi_config.mipi_outputrate = 80;

    _video_mipi_config.mipi_phy_timing.mipi_ths_prepare  = 0x00000012u;
    _video_mipi_config.mipi_phy_timing.mipi_ths_settle   = 0x00000019u;
    _video_mipi_config.mipi_phy_timing.mipi_tclk_prepare = 0x0000000Fu;
    _video_mipi_config.mipi_phy_timing.mipi_tclk_settle  = 0x0000001Eu;
    _video_mipi_config.mipi_phy_timing.mipi_tclk_miss    = 0x00000008u;
    _video_mipi_config.mipi_phy_timing.mipi_t_init_slave = 0x0000338Fu;

    memset(&_video_vin_setup, 0, sizeof(_video_vin_setup));
    
    /* SPEA */
    memset(&_video_spea_config, 0, sizeof(_video_spea_config));
#endif
} /* End of constructor method () */

/**************************************************************************//**
 * @brief       Graphics initialization processing
 * @param[in]   lcd_config                : LCD configuration
 * @retval      error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Graphics_init( const lcd_config_t * lcd_config )
{
    if( lcd_config != NULL ) {
        _lcd_config.lcd_type             = lcd_config->lcd_type;            /* LVDS or Pararel RGB                      */
        _lcd_config.intputClock          = lcd_config->intputClock;         /* P1  clk [MHz] ex. 66.67                  */
        _lcd_config.outputClock          = lcd_config->outputClock;         /* LCD clk [MHz] ex. 33.33                  */

        _lcd_config.lcd_outformat        = lcd_config->lcd_outformat;       /* Output format select */
        _lcd_config.lcd_edge             = lcd_config->lcd_edge;            /* Output phase control of LCD_DATA23 to LCD_DATA0 pin */

        _lcd_config.h_toatal_period      = lcd_config->h_toatal_period;     /* Free-running Hsync period                */
        _lcd_config.v_toatal_period      = lcd_config->v_toatal_period;     /* Free-running Vsync period                */
        _lcd_config.h_disp_widht         = lcd_config->h_disp_widht;        /* LCD display area size, horizontal width  */
        _lcd_config.v_disp_widht         = lcd_config->v_disp_widht;        /* LCD display area size, vertical width    */
        _lcd_config.h_back_porch         = lcd_config->h_back_porch;        /* LCD display horizontal back porch period */
        _lcd_config.v_back_porch         = lcd_config->v_back_porch;        /* LCD display vertical back porch period   */

        _lcd_config.h_sync_port          = lcd_config->h_sync_port;         /* TCONn or Not use(-1)                     */
        _lcd_config.h_sync_port_polarity = lcd_config->h_sync_port_polarity;/* Polarity inversion control of signal     */
        _lcd_config.h_sync_width         = lcd_config->h_sync_width;        /* Hsync width                              */

        _lcd_config.v_sync_port          = lcd_config->v_sync_port;         /* TCONn or Not use(-1)                     */
        _lcd_config.v_sync_port_polarity = lcd_config->v_sync_port_polarity;/* Polarity inversion control of signal     */
        _lcd_config.v_sync_width         = lcd_config->v_sync_width;        /* Vsync width  */

        _lcd_config.de_port              = lcd_config->de_port;             /* TCONn or Not use(-1)                     */
        _lcd_config.de_port_polarity     = lcd_config->de_port_polarity;    /* Polarity inversion control of signal     */
    }

    return (graphics_error_t)DRV_Graphics_Init( (drv_lcd_config_t *)&_lcd_config );
} /* End of method Graphics_init() */

/**************************************************************************//**
 * @brief       Graphics Video initialization processing
 * @param[in]   video_input_sel                : Input select
 * @param[in]   video_ext_in_config            : Digtal video input configuration
 * @retval      error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Graphics_Video_init( video_input_sel_t video_input_sel, video_ext_in_config_t * video_ext_in_config )
{
    graphics_error_t error;

    if ((video_input_sel == INPUT_SEL_EXT)
#if defined(TARGET_RZ_A1H)
     || (video_input_sel == INPUT_SEL_VDEC)
#endif /* TARGET_RZ_A1H */
     || (video_input_sel == INPUT_SEL_CEU)) {
        _video_input_sel = video_input_sel;

        if (video_ext_in_config != NULL) {
            /* Signals supplied via the external input pins        */
            /* if using Video decoder output signals, not using value. */
            _video_ext_in_config.inp_format    = video_ext_in_config->inp_format;
            _video_ext_in_config.inp_pxd_edge  = video_ext_in_config->inp_pxd_edge;
            _video_ext_in_config.inp_vs_edge   = video_ext_in_config->inp_vs_edge;
            _video_ext_in_config.inp_hs_edge   = video_ext_in_config->inp_hs_edge;
            _video_ext_in_config.inp_endian_on = video_ext_in_config->inp_endian_on;
            _video_ext_in_config.inp_swap_on   = video_ext_in_config->inp_swap_on;
            _video_ext_in_config.inp_vs_inv    = video_ext_in_config->inp_vs_inv;
            _video_ext_in_config.inp_hs_inv    = video_ext_in_config->inp_hs_inv;
            _video_ext_in_config.inp_f525_625  = video_ext_in_config->inp_f525_625;
            _video_ext_in_config.inp_h_pos     = video_ext_in_config->inp_h_pos;
            _video_ext_in_config.cap_vs_pos    = video_ext_in_config->cap_vs_pos;
            _video_ext_in_config.cap_hs_pos    = video_ext_in_config->cap_hs_pos;
            _video_ext_in_config.cap_width     = video_ext_in_config->cap_width;
            _video_ext_in_config.cap_height    = video_ext_in_config->cap_height;
        }
        error = (graphics_error_t)DRV_Graphics_Video_init( (drv_video_input_sel_t)video_input_sel,
                (drv_video_ext_in_config_t *)&_video_ext_in_config );
    } else {
        error = GRAPHICS_PARAM_RANGE_ERR;
    }

    return error;
} /* End of method Graphics_Video_init() */

/**************************************************************************//**
 * @brief       Graphics Video initialization processing
 * @param[in]   video_input_sel                : Input select
 * @param[in]   video_mipi_config              : MIPI configuration
 * @param[in]   video_vin_setup                : MIPI configuration
 * @retval      error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Graphics_Video_init( video_input_sel_t video_input_sel, video_mipi_param_t * video_mipi_config, video_vin_setup_t * video_vin_setup )
{
    graphics_error_t error = GRAPHICS_VDC5_ERR;

#if defined(TARGET_RZ_A2XX)
    if (video_input_sel == INPUT_SEL_MIPI) {
        _video_input_sel = video_input_sel;

        if (video_mipi_config != NULL) {
            memcpy(&_video_mipi_config, video_mipi_config, sizeof(_video_mipi_config));
        }
        if (video_vin_setup != NULL) {
            memcpy(&_video_vin_setup, video_vin_setup, sizeof(_video_vin_setup));
        }
        error = (graphics_error_t)DRV_Graphics_Video_init( (drv_video_input_sel_t)video_input_sel,
                (drv_video_ext_in_config_t *)&_video_ext_in_config );
    } else {
        error = GRAPHICS_PARAM_RANGE_ERR;
    }
#endif

    return error;
} /* End of method Graphics_Video_init() */

/**************************************************************************//**
 * @brief       LCD I/O initialization processing
 * @param[in]   pin                       : Pointer of the pin assignment
 * @param[in]   pin_count                 : Total number of the pin assignment
 * @retval      error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Graphics_Lcd_Port_Init( PinName *pin, unsigned int pin_count )
{
    return (graphics_error_t)DRV_Graphics_Lcd_Port_Init( pin, pin_count );
} /* End of method Graphics_Lcd_Port_Init() */

/**************************************************************************//**
 * @brief       LVDS I/O port initialization processing
 * @param[in]   pin                       : Pointer of the pin assignment
 * @param[in]   pin_count                 : Total number of the pin assignment
 * @retval      error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Graphics_Lvds_Port_Init( PinName *pin, unsigned int pin_count )
{
    return (graphics_error_t)DRV_Graphics_Lvds_Port_Init( pin, pin_count );
} /* End of method Graphics_Lvds_Port_Init() */

/**************************************************************************//**
 * @brief       Digital video input I/O port initialization processing
 * @param[in]   pin                       : Pointer of the pin assignment
 * @param[in]   pin_count                 : Total number of the pin assignment
 * @retval      error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Graphics_Dvinput_Port_Init( PinName *pin, unsigned int pin_count )
{
    return (graphics_error_t)DRV_Graphics_Dvinput_Port_Init( pin, pin_count );
} /* End of method Graphics_Dvinput_Port_Init() */

/**************************************************************************//**
 * @brief       CEU input I/O port initialization processing
 * @param[in]   pin                       : Pointer of the pin assignment
 * @param[in]   pin_count                 : Total number of the pin assignment
 * @retval      error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Graphics_Ceu_Port_Init( PinName *pin, unsigned int pin_count )
{
    return (graphics_error_t)DRV_Graphics_CEU_Port_Init( pin, pin_count );
} /* End of method Graphics_Ceu_Port_Init() */

/**************************************************************************//**
 * @brief       IRQ interrupt handler setting
 * @param[in]   irq                       : VDC5 interrupt type
 * @param[in]   num                       : Interrupt line number
 * @param[in]   * callback                : Interrupt callback function pointer
 * @retval      error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Graphics_Irq_Handler_Set( int_type_t irq, unsigned short num, void (* callback)(int_type_t)  )
{
    return (graphics_error_t)DRV_Graphics_Irq_Handler_Set( (vdc5_int_type_t)irq, num, (void (*)(vdc5_int_type_t))callback );
} /* End of method Graphics_Irq_Handler_Set() */

/**************************************************************************//**
 * @brief       Graphics surface read start processing
 * @param[in]   layer_id                  : Graphics layer ID
 * @retval      error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Graphics_Start( graphics_layer_t layer_id )
{
    return (graphics_error_t)DRV_Graphics_Start( (drv_graphics_layer_t)layer_id );
} /* End of method Graphics_Start() */

/**************************************************************************//**
 * @brief       Graphics surface read stop processing
 * @param[in]   layer_id                  : Graphics layer ID
 * @retval      error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Graphics_Stop( graphics_layer_t layer_id )
{
    return (graphics_error_t)DRV_Graphics_Stop( (drv_graphics_layer_t)layer_id );
} /* End of method Graphics_Stop() */

/**************************************************************************//**
 * @brief       Video surface write start processing
 * @param[in]   video_input_channel        : Video input channel
 * @retval      error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Video_Start( video_input_channel_t video_input_channel )
{
    graphics_error_t error = GRAPHICS_OK;

    /*  Digital video inputs : supporting video_input_channel 0 only. */
    if( _video_input_sel == INPUT_SEL_EXT && video_input_channel == VIDEO_INPUT_CHANNEL_1 ) {
        error = GRAPHICS_PARAM_RANGE_ERR;
    }

    if( error == GRAPHICS_OK ) {
        error = (graphics_error_t)DRV_Video_Start( (drv_video_input_channel_t)video_input_channel );
    }
    return error;
} /* End of method Video_Start() */

/**************************************************************************//**
 * @brief       Video surface write stop processing
 * @param[in]   video_input_channel        : Video input channel
 * @retval      error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Video_Stop( video_input_channel_t video_input_channel )
{
    graphics_error_t error = GRAPHICS_OK;

    /*  Digital video inputs : supporting video_input_channel 0 only. */
    if( _video_input_sel == INPUT_SEL_EXT && video_input_channel == VIDEO_INPUT_CHANNEL_1 ) {
        error = GRAPHICS_PARAM_RANGE_ERR;
    }

    if( error == GRAPHICS_OK ) {
        error = (graphics_error_t)DRV_Video_Stop(
                    (drv_video_input_channel_t)video_input_channel );
    }
    return error;
} /* End of method Video_Stop() */

/**************************************************************************//**
 * @brief       Graphics surface read process setting
 *
 *              Description:<br>
 *              This function supports the following 4 image format.
 *                  YCbCr422, RGB565, RGB888, ARGB8888
 * @param[in]   layer_id                   : Graphics layer ID
 * @param[in]   framebuff                  : Base address of the frame buffer
 * @param[in]   fb_stride                  : Line offset address of the frame buffer
 * @param[in]   gr_format                  : Format of the frame buffer read signal
 * @param[in]   wr_rd_swa                  : frame buffer swap setting
 *      - WR_RD_WRSWA_NON        : Not swapped: 1-2-3-4-5-6-7-8
 *      - WR_RD_WRSWA_8BIT       : Swapped in 8-bit units: 2-1-4-3-6-5-8-7
 *      - WR_RD_WRSWA_16BIT      : Swapped in 16-bit units: 3-4-1-2-7-8-5-6
 *      - WR_RD_WRSWA_16_8BIT    : Swapped in 16-bit units + 8-bit units: 4-3-2-1-8-7-6-5
 *      - WR_RD_WRSWA_32BIT      : Swapped in 32-bit units: 5-6-7-8-1-2-3-4
 *      - WR_RD_WRSWA_32_8BIT    : Swapped in 32-bit units + 8-bit units: 6-5-8-7-2-1-4-3
 *      - WR_RD_WRSWA_32_16BIT   : Swapped in 32-bit units + 16-bit units: 7-8-5-6-3-4-1-2
 *      - WR_RD_WRSWA_32_16_8BIT : Swapped in 32-bit units + 16-bit units + 8-bit units: 8-7-6-5-4-3-2-1
 * @param[in]   gr_rect                    : Graphics display area
 * @param[in]   gr_clut                    : CLUT setup parameter
 * @retval      Error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Graphics_Read_Setting(
    graphics_layer_t    layer_id,
    void              * framebuff,
    unsigned int        fb_stride,
    graphics_format_t   gr_format,
    wr_rd_swa_t         wr_rd_swa,
    rect_t            * gr_rect,
    clut_t            * gr_clut )
{
    rect_t rect;

    rect.hs = gr_rect->hs + _lcd_config.h_back_porch;
    rect.vs = gr_rect->vs + _lcd_config.v_back_porch;
    rect.hw = gr_rect->hw;
    rect.vw = gr_rect->vw;

#if defined(TARGET_RZ_A2XX)
    full_screen = rect;
#endif

    return (graphics_error_t)DRV_Graphics_Read_Setting(
               (drv_graphics_layer_t)layer_id,
               framebuff,
               fb_stride,
               (drv_graphics_format_t)gr_format,
               (drv_wr_rd_swa_t)wr_rd_swa,
               (drv_rect_t *)&rect,
               (drv_clut_t *)gr_clut);

} /* End of method Graphics_Read_Setting() */

/**************************************************************************//**
 * @brief       Graphics surface read process changing
 *
 *              Description:<br>
 *              This function is used to swap buffers.
 *
 * @param[in]   layer_id                : Graphics layer ID
 * @param[in]   framebuff               : Base address of the frame buffer
 * @retval      Error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Graphics_Read_Change ( graphics_layer_t layer_id, void *  framebuff)
{
    return (graphics_error_t)DRV_Graphics_Read_Change(
               (drv_graphics_layer_t)layer_id, framebuff );
} /* End of method Graphics_Read_Change() */

#if defined(TARGET_RZ_A2XX)
/**************************************************************************//**
 * @brief       Graphics create surface processing
 *
 *              Description:<br>
 *              Setup Sprite
 *
 * @param[in]   gr_disp_cnf   : Graphics surface read config
 * @retval      VDC driver error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Graphics_Read_Setting_SPEA(
    graphics_layer_t    layer_id,
    rect_t            * gr_rect)
{
    vdc_read_t spea_cnf;
    vdc_error_t error;
    vdc_start_t start;
    vdc_gr_disp_sel_t disp_mode;


    /* Read data parameter */
    spea_cnf.gr_ln_off_dir
        = VDC_GR_LN_OFF_DIR_INC;       /* Line offset address direction of the frame buffer */
    spea_cnf.gr_flm_sel     = VDC_GR_FLM_SEL_FLM_NUM;          /* Selects a frame buffer address setting signal */
    spea_cnf.gr_imr_flm_inv = VDC_OFF;                         /* Frame buffer number for distortion correction */
    spea_cnf.gr_bst_md      = VDC_BST_MD_32BYTE;               /* Frame buffer burst transfer mode */
    spea_cnf.gr_base        = (void *) VIRTUAL_FRAME_BASE_ADD; /* Frame buffer base address */
    spea_cnf.gr_ln_off      = VIRTUAL_FRAME_STRAID;            /* Frame buffer line offset address */
    spea_cnf.width_read_fb  = NULL;                            /* Width of the image read from frame buffer */
    spea_cnf.adj_sel
        = VDC_ON;                      /* Measures to decrease the influence by folding pixels/lines (on/off) */
    spea_cnf.gr_format      = VDC_GR_FORMAT_ARGB8888;          /* Format of the frame buffer read signal */
    spea_cnf.gr_ycc_swap
        = VDC_GR_YCCSWAP_CBY0CRY1;         /* Controls swapping of data read from buffer in the YCbCr422 format */
    spea_cnf.gr_rdswa       = VDC_WR_RD_WRSWA_32BIT;           /* Frame buffer swap setting */

    /* Display area */
    spea_cnf.gr_grc.vs = (uint16_t) full_screen.vs;            /* vertical start position */
    spea_cnf.gr_grc.vw = (uint16_t) gr_rect->vw;               /* vertical display size */
    spea_cnf.gr_grc.hs = (uint16_t) full_screen.hs;            /*horizontal start position*/
    spea_cnf.gr_grc.hw = (uint16_t) gr_rect->hw;               /* horizontal display size */

    disp_mode = VDC_DISPSEL_BLEND;
    start.gr_disp_sel = &disp_mode;

    error = R_VDC_ReadDataControl(VDC_CHANNEL_0, (vdc_layer_id_t)layer_id, &spea_cnf);
    if (error == VDC_OK) {
        error = R_VDC_StartProcess(VDC_CHANNEL_0, (vdc_layer_id_t)layer_id, &start);
    }
    if (error == VDC_OK) {
        error = (vdc_error_t)R_SPEA_WindowOffset((vdc_layer_id_t)layer_id, 0, 0);
    }
    return (graphics_error_t)error;
} /* End of method Graphics_Read_Setting_SPEA() */

/**************************************************************************//**
 * @brief       Graphics surface read process changing and updating
 *
 *              Description:<br>
 *              Update Sprite settings.
 *
 * @param[in]   layer_id     : VDC Layer ID (2 or 3)
 * @param[in]   window_id    : SPEA window ID
 *                              - WINDOW_00 - WINDOW_15
 * @param[in]   sken         : Window ON/OFF.
 * @param[in]   size         : Window size.
 * @param[in]   pos          : Window start coordinates.
 * @param[in]   buffer       : Window read buffer address.
 * @retval      SPEA driver error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Graphics_Update_Window_SPEA ( const graphics_layer_t layer_id, 
                const video_spea_window_id_t window_id,
                const video_spea_onoff_t sken,
                const video_spea_sklym_t * size,
                const video_spea_skpsm_t * pos,
                const void * buffer)
{
    spea_error_t spea_error;

    spea_error = R_SPEA_SetWindow((vdc_layer_id_t)layer_id, 
                (spea_window_id_t)window_id,
                (spea_onoff_t)sken,
                (spea_sklym_t*)size,
                (spea_skpsm_t*)pos,
                buffer);
    
    if (spea_error == SPEA_OK) {
        spea_error = R_SPEA_WindowUpdate((vdc_layer_id_t)layer_id);
    }
    return (graphics_error_t)spea_error;
} /* End of method Graphics_Update_Window_SPEA() */
#endif

/**************************************************************************//**
 * @brief       Video surface write process setting
 *              This function set the video write process. Input form is weave
 *              (progressive) mode fixed.
 *              This function supports the following 3 image format.
 *                  YCbCr422, RGB565, RGB888
 * @param[in]   video_input_ch          : Video input channel
 * @param[in]   col_sys                 : Analog video signal color system
 * @param[in]   adc_vinsel              : Video input pin
 * @param[in]   framebuff               : Base address of the frame buffer
 * @param[in]   fb_stride [byte]        : Line offset address of the frame buffer
 * @param[in]   video_format            : Frame buffer video-signal writing format
 *      - VIDEO_FORMAT_YCBCR422 : YCBCR422 (2byte/px)
 *      - VIDEO_FORMAT_RGB565   : RGB565 (2byte/px)
 *      - VIDEO_FORMAT_RGB888   : RGB888 (4byte/px)
 * @param[in]   wr_rd_swa               : frame buffer swap setting
 *      - WR_RD_WRSWA_NON        : Not swapped: 1-2-3-4-5-6-7-8
 *      - WR_RD_WRSWA_8BIT       : Swapped in 8-bit units: 2-1-4-3-6-5-8-7
 *      - WR_RD_WRSWA_16BIT      : Swapped in 16-bit units: 3-4-1-2-7-8-5-6
 *      - WR_RD_WRSWA_16_8BIT    : Swapped in 16-bit units + 8-bit units: 4-3-2-1-8-7-6-5
 *      - WR_RD_WRSWA_32BIT      : Swapped in 32-bit units: 5-6-7-8-1-2-3-4
 *      - WR_RD_WRSWA_32_8BIT    : Swapped in 32-bit units + 8-bit units: 6-5-8-7-2-1-4-3
 *      - WR_RD_WRSWA_32_16BIT   : Swapped in 32-bit units + 16-bit units: 7-8-5-6-3-4-1-2
 *      - WR_RD_WRSWA_32_16_8BIT : Swapped in 32-bit units + 16-bit units + 8-bit units: 8-7-6-5-4-3-2-1
 * @param[in]   video_write_size_vw [px]: output v width
 * @param[in]   video_write_size_hw [px]: output h width
 * @param[in]   video_adc_vinsel        : Input pin control
 * @retval      Error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Video_Write_Setting(
    video_input_channel_t       video_input_channel,
    graphics_video_col_sys_t    col_sys,
    void                      * framebuff,
    unsigned int                fb_stride,
    video_format_t              video_format,
    wr_rd_swa_t                 wr_rd_swa,
    unsigned short              write_buff_vw,
    unsigned short              write_buff_hw,
    video_adc_vinsel_t          video_adc_vinsel )

{
    graphics_error_t error = GRAPHICS_OK;

    if( _video_input_sel == INPUT_SEL_VDEC ) {
        if( col_sys == COL_SYS_NTSC_358 || col_sys == COL_SYS_NTSC_443 || col_sys == COL_SYS_NTSC_443_60 ) {
            if( (write_buff_vw / 2u) > 240u ) {
                error = GRAPHICS_VIDEO_NTSC_SIZE_ERR;
            }
        } else {
            if( (write_buff_vw / 2u) > 280u ) {
                error = GRAPHICS_VIDEO_PAL_SIZE_ERR;
            }
        }

        if( write_buff_hw > 800u ) {
            error = GRAPHICS_PARAM_RANGE_ERR;
        }
        if( error == GRAPHICS_OK ) {
            error = (graphics_error_t)DRV_Video_Write_Setting(
                        (drv_video_input_channel_t)video_input_channel,
                        (drv_graphics_video_col_sys_t)col_sys,
                        framebuff,
                        fb_stride,
                        (drv_video_format_t)video_format,
                        (drv_wr_rd_swa_t)wr_rd_swa,
                        write_buff_vw,
                        write_buff_hw,
                        (drv_video_adc_vinsel_t)video_adc_vinsel);
        }
    } else if( _video_input_sel == INPUT_SEL_EXT ) {
        rect_t cap_area;

        cap_area.hs = _video_ext_in_config.cap_hs_pos * 2;
        cap_area.hw = _video_ext_in_config.cap_width  * 2;
        cap_area.vs = _video_ext_in_config.cap_vs_pos;
        cap_area.vw = _video_ext_in_config.cap_height;

        error = (graphics_error_t) DRV_Video_Write_Setting_Digital(
                    framebuff,
                    fb_stride,
                    (drv_video_format_t)video_format,
                    (drv_wr_rd_swa_t)wr_rd_swa,
                    write_buff_vw,
                    write_buff_hw,
                    (drv_rect_t *)&cap_area );
    } else if( _video_input_sel == INPUT_SEL_CEU ) {
        error = (graphics_error_t) DRV_Video_Write_Setting_Ceu(
                    framebuff,
                    fb_stride,
                    (drv_video_format_t)video_format,
                    (drv_wr_rd_swa_t)wr_rd_swa,
                    write_buff_vw,
                    write_buff_hw,
                    (drv_video_ext_in_config_t *)&_video_ext_in_config);
#if defined(TARGET_RZ_A2XX)
    } else if( _video_input_sel == INPUT_SEL_MIPI ) {
        error = (graphics_error_t) DRV_Video_Write_Setting_Mipi(
                    framebuff,
                    fb_stride,
                    (drv_video_format_t)video_format,
                    (drv_wr_rd_swa_t)wr_rd_swa,
                    write_buff_vw,
                    write_buff_hw,
                    (drv_mipi_param_t *)&_video_mipi_config,
                    (drv_vin_setup_t *)&_video_vin_setup);
#endif
    } else {
        error = GRAPHICS_PARAM_RANGE_ERR;
    }
    return error;
} /* End of method Video_Write_Setting() */

/**************************************************************************//**
 * @brief       Graphics surface write process changing
 *              This function is used to swap buffers of the weave write processing.
 * @param[in]   video_input_ch          : Video input channle
 * @param[in]   framebuff               : Base address of the frame buffer
 * @param[in]   fb_stride               : Line offset address of the frame buffer
 * @retval      Error code
******************************************************************************/
DisplayBase::graphics_error_t
DisplayBase::Video_Write_Change (
    video_input_channel_t video_input_channel, void * framebuff, uint32_t fb_stride )
{
    return (graphics_error_t)DRV_Video_Write_Change(
               (drv_video_input_channel_t)video_input_channel, framebuff, fb_stride );
} /* End of method Video_Write_Change() */

/* End of file */
