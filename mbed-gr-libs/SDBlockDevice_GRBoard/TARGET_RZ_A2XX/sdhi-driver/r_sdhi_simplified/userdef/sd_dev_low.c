/**********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO
 * THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2019 Renesas Electronics Corporation. All rights reserved.
 *********************************************************************************************************************/
/******************************************************************************
* System Name  : SDHI Driver
* File Name    : sd_dev_low.c
* Version      : 1.30
* Device(s)    : RZ/A2M
* Tool-Chain   : e2 studio (GCC ARM Embedded)
* OS           : None
* H/W Platform : RZ/A2M Evaluation Board
* Description  : RZ/A2M SD Driver Sample Program
* Operation    :
* Limitations  : Ch0 and Ch1 can't be used at the same time, because the timers
*              : used on Ch0 and Ch1 are common.
******************************************************************************
* History : DD.MM.YYYY Version Description
*         : 16.03.2018 1.00    First Release
*         : 14.12.2018 1.01    Changed the DMAC soft reset procedure.
*         : 28.12.2018 1.02    Support for OS
*         : 29.05.2019 1.20    Correspond to internal coding rules
*         : 12.11.2019 1.30    Support for SDIO
******************************************************************************/


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "r_typedefs.h"
#include "iodefine.h"
#include "iobitmask.h"
#include "cmsis.h"
#include "cmsis_os.h"
#include "irq_ctrl.h"
#include "mbed_wait_api.h"
#include "platform/mbed_critical.h"
#include "r_sdif.h"
#include "r_sd_cfg.h"
#include "sd_dev_dmacdrv.h"
#include "r_sdhi_simplified_drv_sc_cfg.h"

/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Macro definitions
******************************************************************************/
#define INT_PRV_LEVEL_SDHI      (26u)       /* SDHI interrupt level  */

#define SDHI_PRV_CH_0           (0)
#define SDHI_PRV_CH_1           (1)
#define SDHI_PRV_CH_NUM         (2)

#define SDHI_PRV_PPOC_POC_3_3V  (1u)

#define SDHI_PRV_CLK_TDSEL_3_3V (3u)

#define SDHI_PRV_CARDDET_TIME   (1000uL)
#define SDHI_PRV_POWERON_TIME   (100uL)

#ifdef SD_CFG_HWINT
#define SDHI_PRV_MAX_WAITTIME   (0xFFFFuL)

#else /* #ifdef SD_CFG_HWINT */
#define SDHI_PRV_1MSEC          (1uL)

#endif /* #ifdef SD_CFG_HWINT */

#define SDHI_PRV_TO_SPEED       (512uL)
#define SDHI_PRV_1024BYTE       (1024uL)
#define SDHI_PRV_1000MSEC       (1000uL)

#define UNUSED_PARAM(param)     (void)(param)
#define SDHI0_IRQ               SDHI0_0_IRQn
#define SDHI1_IRQ               SDHI1_0_IRQn
#define SDHI0_STB               0
#define SDHI1_STB               0
/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/

/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/

/******************************************************************************
Private global variables and functions
******************************************************************************/
static void sddev_sd_int_handler_0(void);
static void sddev_sd_int_handler_1(void);
osSemaphoreDef(sdint_sem_sync);
osSemaphoreDef(sdint_sem_dma);

static st_sdhi_info_dev_ch_t s_sdhi_dev_ch[SDHI_PRV_CH_NUM] =
{
    /*--- ch 0 ---*/
    {
        { false,                 SDHI0_STB                                  }, /* stb  */
        { SDHI0_IRQ,             INT_PRV_LEVEL_SDHI, sddev_sd_int_handler_0 }, /* intc */
        { false,                 false                                      }, /* gpio */
#ifdef SD_CFG_HWINT
        { 0uL,                   0uL                                        }  /* semaphore */
#endif /* #ifdef SD_CFG_HWINT */
    },
    /*--- ch 1 ---*/
    {
        { false,                 SDHI1_STB                                  }, /* stb  */
        { SDHI1_IRQ,             INT_PRV_LEVEL_SDHI, sddev_sd_int_handler_1 }, /* intc */
        { false,                 false                                      }, /* gpio */
#ifdef SD_CFG_HWINT
        { 0uL,                   0uL                                        }  /* semaphore */
#endif /* #ifdef SD_CFG_HWINT */
    }
};

/******************************************************************************
 * Function Name: sddev_cmd0_sdio_mount
 * Description  : Select to issue CMD0 before SDIO Mount
 * Arguments    : int32_t sd_port : channel no (0 or 1)
 * Return Value : SD_OK  : issue CMD0
 *                SD_ERR : not issue CMD0
 *****************************************************************************/
int32_t sddev_cmd0_sdio_mount(int32_t sd_port)
{
    /* Cast to an appropriate type */
    (void)sd_port;

#ifdef SD_CFG_IO
    return SD_ERR;
#else
    return SD_ERR;
#endif
}
/******************************************************************************
 End of function sddev_cmd0_sdio_mount
 *****************************************************************************/

/******************************************************************************
 * Function Name: sddev_cmd8_sdio_mount
 * Description  : Select to issue CMD8 before SDIO Mount
 * Arguments    : int32_t sd_port : channel no (0 or 1)
 * Return Value : SD_OK  : issue CMD8
 *                SD_ERR : not issue CMD8
 *****************************************************************************/
int32_t sddev_cmd8_sdio_mount(int32_t sd_port)
{
    /* Cast to an appropriate type */
    (void)sd_port;

#ifdef SD_CFG_IO
    return SD_OK;
#else
    return SD_ERR;
#endif
}
/******************************************************************************
 End of function sddev_cmd8_sdio_mount
 *****************************************************************************/

/******************************************************************************
* Function Name: sddev_init
* Description  : Initialize H/W to use SDHI
* Arguments    : int32_t sd_port : channel no (0 or 1)
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int32_t sddev_init(int32_t sd_port)
{
    int32_t               ret;
    st_sdhi_info_dev_ch_t *p_ch;

    ret  = SD_OK;
    p_ch = sddev_get_dev_ch_instance(sd_port);

    if (p_ch != NULL) {
        core_util_critical_section_enter();

        /* --- SDHI, clear module standby start ---*/
        /* [Canceling Module Standby Function] */
        /* a. Release from the module standby state after the activation of the
              module by a power-on reset while the MSTP bit is set to 1
           1. Clear the MSTP bit to 0.
           2. After that, dummy-read the same register. */

        /* b. Release from the module standby state after a transition to standby
              following activation of the module
           1. Clear the MSTP bit to 0, then dummy-read the same register.   */
        volatile uint8_t reg_read_8;

        CPG.STBCR10.BYTE &= ~(0x0C >> (sd_port * 2));
        reg_read_8 = CPG.STBCR10.BYTE; /* dummy read */
        (void)reg_read_8;

        if (p_ch->stb.stb_pon_init == false) {
            /* case a */
            p_ch->stb.stb_pon_init = true;
        } else {
            /* case b */
            while (1) {
                CPG.STBREQ1.BYTE &= ~(0x04 >> sd_port);
                reg_read_8 = CPG.STBREQ1.BYTE; /* dummy read */
                reg_read_8 = CPG.STBACK1.BYTE;
                if ((reg_read_8 & (0x04 >> sd_port)) == 0) {
                    break;
                }
            }
        }
        /* --- SDHI, clear module standby end ---*/

        core_util_critical_section_exit();

#ifdef SD_CFG_HWINT
        if (ret == SD_OK) {
            /* ---- Register SDHI interrupt handler ---- */
            (void)IRQ_SetHandler(p_ch->intc.int_id, p_ch->intc.p_func);
            /* ---- Set priority of SDHI interrupt handler to INT_LEVEL_SDHI ---- */
            GIC_SetConfiguration(p_ch->intc.int_id, 1);
            (void)IRQ_SetPriority(p_ch->intc.int_id, p_ch->intc.int_priority);
            /* ---- Validate SDHI interrupt ---- */
            (void)IRQ_Enable(p_ch->intc.int_id);

            if (p_ch->semaphore.sem_sync == NULL) {
                p_ch->semaphore.sem_sync = osSemaphoreNew(0xffff, 0, osSemaphore(sdint_sem_sync));
            }
            if (p_ch->semaphore.sem_sync == NULL) {
                ret = SD_ERR;
            }
            if (p_ch->semaphore.sem_dma == NULL) {
                p_ch->semaphore.sem_dma = osSemaphoreNew(0xffff, 0, osSemaphore(sdint_sem_dma));
            }
            if (p_ch->semaphore.sem_dma == NULL) {
                ret = SD_ERR;
            }
            if (ret == SD_ERR) {
                if (p_ch->semaphore.sem_sync != NULL) {
                    osSemaphoreDelete(p_ch->semaphore.sem_sync);
                    p_ch->semaphore.sem_sync = NULL;
                }
                if (p_ch->semaphore.sem_dma != NULL) {
                    osSemaphoreDelete(p_ch->semaphore.sem_dma);
                    p_ch->semaphore.sem_dma = NULL;
                }
            }
        }
#endif /* #ifdef SD_CFG_HWINT */
    } else { /* if (p_ch != NULL) */
        ret = SD_ERR;
    }
    return ret;
}
/*******************************************************************************
 End of function sddev_init
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_power_on
* Description  : Power-on H/W to use SDHI
* Arguments    : int32_t sd_port : channel no (0 or 1)
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int32_t sddev_power_on(int32_t sd_port)
{
    /* Cast to an appropriate type */
    UNUSED_PARAM(sd_port);

    /* ---Power On SD ---- */

    /* ---- Wait for  SD Wake up ---- */
    osDelay(SDHI_PRV_POWERON_TIME);

    return SD_OK;
}
/*******************************************************************************
 End of function sddev_power_on
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_power_off
* Description  : Power-off H/W to use SDHI
* Arguments    : int32_t sd_port : channel no (0 or 1)
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int32_t sddev_power_off(int32_t sd_port)
{
    /* Cast to an appropriate type */
    UNUSED_PARAM(sd_port);

    return SD_OK;
}
/*******************************************************************************
 End of function sddev_power_off
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_read_data
* Description  : read from SDHI buffer FIFO
* Arguments    : int32_t sd_port   : channel no (0 or 1)
*              : uint8_t *buff     : buffer addrees to store reading datas
*              : uint32_t reg_addr : SDIP FIFO address
*              : int32_t num       : counts to read(unit:byte)
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int32_t sddev_read_data(int32_t sd_port, uint8_t *buff, uint32_t reg_addr, int32_t num)
{
    int32_t  i;
    int32_t  cnt;
    uint64_t *p_reg;
    uint64_t *p_l;
    uint8_t  *p_c;
    volatile uint64_t tmp;

    /* Cast to an appropriate type */
    UNUSED_PARAM(sd_port);

    /* Cast to an appropriate type */
    p_reg = (uint64_t *)(reg_addr);

    cnt = (num / 8);

    /* Cast to an appropriate type */
    if (0uL != ((uint32_t)buff & 0x7uL))
    {
        /* Cast to an appropriate type */
        p_c = (uint8_t *)buff;
        for (i = cnt; i > 0 ; i--)
        {
            tmp = *p_reg;

            /* Cast to an appropriate type */
            *p_c++ = (uint8_t)(tmp);

            /* Cast to an appropriate type */
            *p_c++ = (uint8_t)(tmp >> 8);

            /* Cast to an appropriate type */
            *p_c++ = (uint8_t)(tmp >> 16);

            /* Cast to an appropriate type */
            *p_c++ = (uint8_t)(tmp >> 24);

            /* Cast to an appropriate type */
            *p_c++ = (uint8_t)(tmp >> 32);

            /* Cast to an appropriate type */
            *p_c++ = (uint8_t)(tmp >> 40);

            /* Cast to an appropriate type */
            *p_c++ = (uint8_t)(tmp >> 48);

            /* Cast to an appropriate type */
            *p_c++ = (uint8_t)(tmp >> 56);
        }

        cnt = (num % 8);
        if (0 != cnt)
        {
            tmp = *p_reg;
            for (i = cnt; i > 0 ; i--)
            {
                /* Cast to an appropriate type */
                *p_c++ = (uint8_t)(tmp);
                tmp >>= 8;
            }
        }
    }
    else
    {
        /* Cast to an appropriate type */
        p_l = (uint64_t *)buff;
        for (i = cnt; i > 0 ; i--)
        {
            *p_l++ = *p_reg;
        }

        cnt = (num % 8);
        if (0 != cnt)
        {
            /* Cast to an appropriate type */
            p_c = (uint8_t *)p_l;
            tmp = *p_reg;
            for (i = cnt; i > 0 ; i--)
            {
                /* Cast to an appropriate type */
                *p_c++ = (uint8_t)(tmp);
                tmp >>= 8;
            }
        }
    }

    return SD_OK;
}
/*******************************************************************************
 End of function sddev_read_data
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_write_data
* Description  : write to SDHI buffer FIFO
* Arguments    : int32_t sd_port   : channel no (0 or 1)
*              : uint8_t *buff     : buffer addrees to store writting datas
*              : uint32_t reg_addr : SDIP FIFO address
*              : int32_t num       : counts to write(unit:byte)
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int32_t sddev_write_data(int32_t sd_port, uint8_t *buff, uint32_t reg_addr, int32_t num)
{
    int32_t  i;

    /* Cast to an appropriate type */
    UNUSED_PARAM(sd_port);

    /* Cast to an appropriate type */
    uint64_t *p_reg = (uint64_t *)(reg_addr);

    /* Cast to an appropriate type */
    uint64_t *p_buff = (uint64_t *)buff;
    uint64_t tmp;

    /* dont care non 8byte alignment data */
    num += 7;
    num /= 8;

    /* Cast to an appropriate type */
    if (((uint32_t)buff & 0x7uL) != 0uL)
    {
        for (i = num; i > 0 ; i--)
        {
            /* Cast to an appropriate type */
            tmp  = (uint64_t)(*buff++);

            /* Cast to an appropriate type */
            tmp |= ((uint64_t)(*buff++) << 8);

            /* Cast to an appropriate type */
            tmp |= ((uint64_t)(*buff++) << 16);

            /* Cast to an appropriate type */
            tmp |= ((uint64_t)(*buff++) << 24);

            /* Cast to an appropriate type */
            tmp |= ((uint64_t)(*buff++) << 32);

            /* Cast to an appropriate type */
            tmp |= ((uint64_t)(*buff++) << 40);

            /* Cast to an appropriate type */
            tmp |= ((uint64_t)(*buff++) << 48);

            /* Cast to an appropriate type */
            tmp |= ((uint64_t)(*buff++) << 56);
            *p_reg = tmp;
        }
    }
    else
    {
        for (i = num; i > 0 ; i--)
        {
            *p_reg = *p_buff++;
        }
    }

    return SD_OK;
}
/*******************************************************************************
 End of function sddev_write_data
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_get_clockdiv
* Description  : Get clock div value.
* Arguments    : int32_t sd_port : channel no (0 or 1)
*              : int32_t clock   : request clock frequency
*              :   SD_CLK_50MHZ
*              :   SD_CLK_25MHZ
*              :   SD_CLK_20MHZ
*              :   SD_CLK_10MHZ
*              :   SD_CLK_5MHZ
*              :   SD_CLK_1MHZ
*              :   SD_CLK_400KHZ
* Return Value : clock div value
*              :   SD_DIV_4   : 1/4   clock
*              :   SD_DIV_8   : 1/8   clock
*              :   SD_DIV_16  : 1/16  clock
*              :   SD_DIV_32  : 1/32  clock
*              :   SD_DIV_256 : 1/256 clock
*              :   SD_DIV_512 : 1/512 clock
******************************************************************************/
uint32_t sddev_get_clockdiv(int32_t sd_port, int32_t clock)
{
    uint32_t div;

    /* Cast to an appropriate type */
    UNUSED_PARAM(sd_port);

    switch (clock)
    {
        case SD_CLK_50MHZ:
            div = SD_DIV_4;        /* 132MHz/4 = 33MHz      */
            break;
        case SD_CLK_25MHZ:
            div = SD_DIV_8;        /* 132MHz/8 = 16.5MHz    */
            break;
        case SD_CLK_20MHZ:
            div = SD_DIV_8;        /* 132MHz/8 = 16.5MHz    */
            break;
        case SD_CLK_10MHZ:
            div = SD_DIV_16;       /* 132MHz/16 = 8.3MHz    */
            break;
        case SD_CLK_5MHZ:
            div = SD_DIV_32;       /* 132MHz/32 = 4.1MHz    */
            break;
        case SD_CLK_1MHZ:
            div = SD_DIV_256;      /* 132MHz/256 = 515.6kHz */
            break;
        case SD_CLK_400KHZ:
            div = SD_DIV_512;      /* 132MHz/512 = 257.8kHz */
            break;
        default:
            div = SD_DIV_512;      /* 132MHz/512 = 257.8kHz */
            break;
    }

    return div;
}
/*******************************************************************************
 End of function sddev_get_clockdiv
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_set_port
* Description  : setting ports to use SDHI
* Arguments    : int32_t sd_port : channel no (0 or 1)
*              : int32_t mode    : SD_PORT_PARALLEL : 4bit mode
*                                : SD_PORT_SERIAL   : 1bit mode
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int32_t sddev_set_port(int32_t sd_port, int32_t mode)
{
    /* Cast to an appropriate type */
    UNUSED_PARAM(sd_port);

    /* Cast to an appropriate type */
    UNUSED_PARAM(mode);

    return SD_OK;
}
/*******************************************************************************
 End of function sddev_set_port
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_int_wait
* Description  : Waitting for SDHI Interrupt
* Arguments    : int32_t sd_port : channel no (0 or 1)
*              : int32_t time    : time out value to wait interrupt
* Return Value : get interrupt : SD_OK
*              : time out      : SD_ERR
******************************************************************************/
int32_t sddev_int_wait(int32_t sd_port, int32_t time)
{
    int32_t             ret;
#ifdef SD_CFG_HWINT
    st_sdhi_info_dev_ch_t *p_ch;

    ret  = SD_ERR;
    p_ch = sddev_get_dev_ch_instance(sd_port);

    if (p_ch != NULL) {
        if (osSemaphoreAcquire(p_ch->semaphore.sem_sync, time) == osOK) {
            ret = SD_OK;
        }
    }
#else /* #ifdef SD_CFG_HWINT */
    uint32_t waittime = (uint32_t)time;

    /* interrupt generated? */
    ret = sd_check_int(sd_port);
    while ((ret == SD_ERR) && (waittime > 0uL)) {
        osDelay(SDHI_PRV_1MSEC);
        waittime--;

        /* interrupt generated? */
        ret = sd_check_int(sd_port);
    }

#endif /* #ifdef SD_CFG_HWINT */

    return ret;
}
/*******************************************************************************
 End of function sddev_int_wait
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_init_dma
* Description  : Initialize DMAC to transfer data from SDHI FIFO
* Arguments    : int32_t sd_port : channel no (0 or 1)
*              : uint32_t buff   : buffer addrees to transfer datas
*              : int32_t dir     : direction to transfer
*              :                 :   0 : FIFO -> buffer
*              :                 :   1 : buffer -> FIFO
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int32_t sddev_init_dma(int32_t sd_port, uint32_t buff, int32_t dir)
{
#ifdef    SD_CFG_TRNS_DMA
    int32_t ret = SD_ERR;

    ret = sd_DMAC_PeriReqInit(sd_port, buff, dir);
    if (SD_OK == ret)
    {
        ret = sd_DMAC_Open(sd_port, dir);
    }
    return ret;

#else

    /* Cast to an appropriate type */
    UNUSED_PARAM(sd_port);

    /* Cast to an appropriate type */
    UNUSED_PARAM(buff);

    /* Cast to an appropriate type */
    UNUSED_PARAM(dir);

    return SD_OK;

#endif
}
/*******************************************************************************
 End of function sddev_init_dma
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_wait_dma_end
* Description  : Wait to complete DMAC transfer
* Arguments    : int32_t sd_port : channel no (0 or 1)
*              : int32_t cnt     : counts to transfer(unit:byte)
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int32_t sddev_wait_dma_end(int32_t sd_port, int32_t cnt)
{
#ifdef SD_CFG_TRNS_DMA
    int32_t             ret;
    uint32_t            waittime;

#ifdef SD_CFG_HWINT
    st_sdhi_info_dev_ch_t *p_ch;

    ret  = SD_ERR;
    p_ch = sddev_get_dev_ch_instance(sd_port);
    
    if (p_ch != NULL)
    {
        /* Caluculate timeout time (as 512kByte/sec)    */
        waittime = ((((uint32_t)cnt / SDHI_PRV_TO_SPEED) / SDHI_PRV_1024BYTE) * SDHI_PRV_1000MSEC);
        if (waittime < SDHI_PRV_1000MSEC)
        {
            waittime = SDHI_PRV_1000MSEC;
        }

        if (osSemaphoreAcquire(p_ch->semaphore.sem_dma, waittime) == osOK) {
            ret = SD_OK;
        }
    }

#else /* #ifdef SD_CFG_HWINT */
    /* Caluculate timeout time (as 512kByte/sec)    */
    waittime = ((((uint32_t)cnt / SDHI_PRV_TO_SPEED) / SDHI_PRV_1024BYTE) * SDHI_PRV_1000MSEC);
    if (waittime < SDHI_PRV_1000MSEC)
    {
        waittime = SDHI_PRV_1000MSEC;
    }

    /* interrupt generated? */
    ret = sd_check_int_dm(sd_port);
    while ((ret == SD_ERR) && (waittime > 0uL))
    {
        osDelay(SDHI_PRV_1MSEC);
        waittime--;

        /* interrupt generated? */
        ret = sd_check_int_dm(sd_port);
    }

#endif /* #ifdef SD_CFG_HWINT */

    return ret;

#else /* #ifdef SD_CFG_TRNS_DMA */

    /* Cast to an appropriate type */
    UNUSED_PARAM(sd_port);

    /* Cast to an appropriate type */
    UNUSED_PARAM(cnt);

    return SD_OK;

#endif /* #ifdef SD_CFG_TRNS_DMA */
}
/*******************************************************************************
 End of function sddev_wait_dma_end
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_disable_dma
* Description  : Disable DMAC transfer
* Arguments    : int32_t sd_port : channel no (0 or 1)
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int32_t sddev_disable_dma(int32_t sd_port)
{
#ifdef SD_CFG_TRNS_DMA
    int32_t ret;

    ret = sd_DMAC_Close(sd_port);

    return ret;

#else

    /* Cast to an appropriate type */
    UNUSED_PARAM(sd_port);

    return SD_OK;

#endif
}
/*******************************************************************************
 End of function sddev_disable_dma
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_reset_dma
* Description  : Reset of the SDHI module built-in DMAC.
* Arguments    : int32_t sd_port : channel no (0 or 1)
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int32_t sddev_reset_dma(int32_t sd_port)
{
    int32_t ret;

    ret = sd_DMAC_Reset(sd_port);
    if (SD_OK == ret)
    {
        ret = sd_DMAC_Released(sd_port);
    }
    return ret;
}
/*******************************************************************************
 End of function sddev_reset_dma
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_finalize_dma
* Description  : Finalize of the SDHI module built-in DMAC.
* Arguments    : int32_t sd_port : channel no (0 or 1)
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int32_t sddev_finalize_dma(int32_t sd_port)
{
    int32_t ret;

    ret = sd_DMAC_Reset(sd_port);

    return ret;
}
/*******************************************************************************
 End of function sddev_finalize_dma
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_loc_cpu
* Description  : lock cpu to disable interrupt
* Arguments    : int32_t sd_port : channel no (0 or 1)
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int32_t sddev_loc_cpu(int32_t sd_port)
{
    if ( (0 == sd_port) || (1 == sd_port) )
    {
        GIC_DisableIRQ(SDHI0_0_IRQn + (sd_port * 2));
    }
    return SD_OK;
}
/*******************************************************************************
 End of function sddev_loc_cpu
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_unl_cpu
* Description  : unlock cpu to enable interrupt
* Arguments    : int32_t sd_port : channel no (0 or 1)
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int32_t sddev_unl_cpu(int32_t sd_port)
{
    if ( (0 == sd_port) || (1 == sd_port) )
    {
        GIC_EnableIRQ(SDHI0_0_IRQn + (sd_port * 2));
    }
    return SD_OK;
}
/*******************************************************************************
 End of function sddev_unl_cpu
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_finalize
* Description  : finalize SDHI
* Arguments    : int32_t sd_port : channel no (0 or 1)
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int32_t sddev_finalize(int32_t sd_port)
{
    int32_t               ret;
    st_sdhi_info_dev_ch_t *p_ch;
    volatile uint8_t      reg_read_8;

    ret  = SD_OK;
    p_ch = sddev_get_dev_ch_instance(sd_port);

    if (p_ch != NULL) {
        core_util_critical_section_enter();
        while (1) {
            CPG.STBREQ1.BYTE |= (0x04 >> sd_port);
            reg_read_8 = CPG.STBREQ1.BYTE; /* dummy read */
            reg_read_8 = CPG.STBACK1.BYTE;
            if ((reg_read_8 & (0x04 >> sd_port)) != 0) {
                break;
            }
        }
        CPG.STBCR10.BYTE |= (0x0C >> (sd_port * 2));
        reg_read_8 = CPG.STBCR10.BYTE; /* dummy read */
        core_util_critical_section_exit();

#ifdef SD_CFG_HWINT
        /* --- Invalidate SDHI interrupt --- */
        (void)IRQ_Disable(p_ch->intc.int_id);

        /* --- Unregister SDHI interrupt handler --- */
        (void)IRQ_SetHandler(p_ch->intc.int_id, NULL);

        /* --- Delete semaphore --- */
        if (p_ch->semaphore.sem_sync != NULL) {
            osSemaphoreDelete(p_ch->semaphore.sem_sync);
            p_ch->semaphore.sem_sync = NULL;
        }
        if (p_ch->semaphore.sem_dma != NULL) {
            osSemaphoreDelete(p_ch->semaphore.sem_dma);
            p_ch->semaphore.sem_dma = NULL;
        }
#endif
    }
    else /* if (p_ch != NULL) */
    {
        ret = SD_ERR;
    }
    return ret;
}
/*******************************************************************************
 End of function sddev_finalize
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_sd_int_handler_0
* Description  : Setting Interrupt function for SDHI(INTC_ID_SDMMC_SDHI0_0)
* Arguments    : uint32_t int_sense : Interrupt mode
* Return Value : none
******************************************************************************/
static void sddev_sd_int_handler_0(void)
{
    sd_int_handler(0);
    sd_int_dm_handler(0);
    sdio_int_handler(0);
}
/*******************************************************************************
 End of function sddev_sd_int_handler_0
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_sd_int_handler_1
* Description  : Setting Interrupt function for SDHI(INTC_ID_SDMMC_SDHI1_0)
* Arguments    : uint32_t int_sense : Interrupt mode
* Return Value : none
******************************************************************************/
static void sddev_sd_int_handler_1(void)
{
    sd_int_handler(1);
    sd_int_dm_handler(1);
    sdio_int_handler(1);
}
/*******************************************************************************
 End of function sddev_sd_int_handler_1
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_cd_layout
* Description  : CD Terminal Support Confirmation
* Arguments    : int32_t sd_port : channel no (0 or 1)
* Return Value : Support     : SD_OK
*              : Not Support : SD_ERR
******************************************************************************/
int32_t sddev_cd_layout(int32_t sd_port)
{
    int32_t ret;
    int32_t sc_port;

    ret = sddev_get_sc_table_config_ch(sd_port, &sc_port);
    if (SD_OK == ret)
    {
        if (SD_CD_ENABLED != SDHI_SC_TABLE[sc_port].config.cd)
        {
            ret = SD_ERR;
        }
    }
    return ret;
}
/*******************************************************************************
 End of function sddev_cd_layout
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_wp_layout
* Description  : WP Terminal Support Confirmation
* Arguments    : int32_t sd_port : channel no (0 or 1)
* Return Value : Support     : SD_OK
*              : Not Support : SD_ERR
******************************************************************************/
int32_t sddev_wp_layout(int32_t sd_port)
{
    int32_t ret;
    int32_t sc_port;

    ret = sddev_get_sc_table_config_ch(sd_port, &sc_port);
    if (SD_OK == ret)
    {
        if (SD_WP_ENABLED != SDHI_SC_TABLE[sc_port].config.wp)
        {
            ret = SD_ERR;
        }
    }
    return ret;
}
/*******************************************************************************
 End of function sddev_wp_layout
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_get_sc_table_config_ch
* Description  : Get the channel number of the table generated by the smart
*              : configurator.
* Arguments    : int32_t sd_port    : channel no (0 or 1)
*              : int32_t *p_sc_port : pointer to sc channel no
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int32_t sddev_get_sc_table_config_ch(int32_t sd_port, int32_t *p_sc_port)
{
    int32_t ret;
    int32_t ch_num;
    int32_t cnt;

    ret = SD_ERR;

    /* Cast to an appropriate type */
    if (NULL != p_sc_port)
    {
        ch_num = (sizeof(SDHI_SC_TABLE)) / sizeof(st_r_drv_sdhi_sc_config_t);

        for (cnt = 0; cnt < ch_num; cnt++)
        {
            if (SDHI_SC_TABLE[cnt].channel == sd_port)
            {
                ret = SD_OK;
                *p_sc_port = cnt;
                break;
            }
        }
    }
    return ret;
}
/*******************************************************************************
 End of function sddev_get_sc_table_config_ch
 ******************************************************************************/

/******************************************************************************
* Function Name: sddev_get_dev_ch_instance
* Description  : Get SDHI driver target CPU interface information object
* Arguments    : int32_t sd_port : channel no (0 or 1)
* Return Value : success : pointer to SDHI driver target CPU interface information
*              : fail    : NULL
******************************************************************************/
st_sdhi_info_dev_ch_t *sddev_get_dev_ch_instance(int32_t sd_port)
{
    st_sdhi_info_dev_ch_t *p_info;

    if ((sd_port >= SDHI_PRV_CH_0) && (sd_port < SDHI_PRV_CH_NUM))
    {
        p_info = &s_sdhi_dev_ch[sd_port];
    }
    else
    {
        /* Cast to an appropriate type */
        p_info = NULL;

        /* NON_NOTICE_ASSERT: channel range over */
    }
    return p_info;
}
/*******************************************************************************
 End of function sddev_get_dev_ch_instance
 ******************************************************************************/

/******************************************************************************
* Function Name: SD_status_callback_function
* Description  : Callback interrupt function for SDHI protocol status control
* Arguments    : int32_t sd_port : channel no (0 or 1)
*              : int32_t cd      : no used
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int32_t SD_status_callback_function(int32_t sd_port, int32_t cd)
{
    /* Cast to an appropriate type */
    UNUSED_PARAM(cd);

#ifdef SD_CFG_HWINT
    int32_t               ret;
    st_sdhi_info_dev_ch_t *p_ch;

    ret  = SD_ERR;
    p_ch = sddev_get_dev_ch_instance(sd_port);

    /* Cast to an appropriate type */
    if (NULL != p_ch)
    {
        (void)osSemaphoreRelease(p_ch->semaphore.sem_sync);
        ret = SD_OK;
    }
    return ret;

#else /* #ifdef SD_CFG_HWINT */

    /* Cast to an appropriate type */
    UNUSED_PARAM(sd_port);

    return SD_OK;

#endif /* #ifdef SD_CFG_HWINT */
}
/*******************************************************************************
 End of function SD_status_callback_function
 ******************************************************************************/

/******************************************************************************
* Function Name: SD_dma_end_callback_function
* Description  : Callback interrupt function for complete DMA transfer control
* Arguments    : int32_t sd_port : channel no (0 or 1)
*              : int32_t cd      : no used
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int32_t SD_dma_end_callback_function(int32_t sd_port, int32_t cd)
{
    /* Cast to an appropriate type */
    UNUSED_PARAM(cd);

#ifdef SD_CFG_HWINT
    int32_t               ret;
    st_sdhi_info_dev_ch_t *p_ch;

    ret  = SD_ERR;
    p_ch = sddev_get_dev_ch_instance(sd_port);

    /* Cast to an appropriate type */
    if (NULL != p_ch)
    {
        (void)osSemaphoreRelease(p_ch->semaphore.sem_dma);
        ret = SD_OK;
    }
    return ret;

#else /* #ifdef SD_CFG_HWINT */

    /* Cast to an appropriate type */
    UNUSED_PARAM(sd_port);

    return SD_OK;

#endif /* #ifdef SD_CFG_HWINT */
}
/*******************************************************************************
 End of function SD_dma_end_callback_function
 ******************************************************************************/

/******************************************************************************
 * Function Name: SD_confirm_semaphore
 * Description  : Confirm semaphore status
 * Arguments    : int32_t sd_port : channel no (0 or 1)
 * Return Value : none
 *****************************************************************************/
void SD_confirm_semaphore(int32_t sd_port)
{
#ifdef SD_CFG_HWINT
    st_sdhi_info_dev_ch_t  *p_ch;

    p_ch = sddev_get_dev_ch_instance(sd_port);

    if (p_ch != NULL) {
        while (osSemaphoreAcquire(p_ch->semaphore.sem_sync, 0) == osOK);
    }

#else /* #ifdef SD_CFG_HWINT */

    /* Cast to an appropriate type */
    UNUSED_PARAM(sd_port);

#endif /* #ifdef SD_CFG_HWINT */
}
/*******************************************************************************
 End of function SD_confirm_semaphore
 ******************************************************************************/

/* End of File */
