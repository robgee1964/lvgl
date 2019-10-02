/*****************************************************************************
 * Simple 4 wire resistive touch screen driver
 *****************************************************************************
 * FileName:        TouchScreenResistive.h
 * Processor:       PIC24F, PIC24H, dsPIC, PIC32
 * Compiler:       	MPLAB C30, MPLAB C32
 * Company:         Microchip Technology Incorporated
 *
 * Software License Agreement
 *
 * Copyright � 2011 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
 *
 * Date        	Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 01/19/11		Ported from TouchScreen.h.
 *****************************************************************************/

/*****************************************************************************
 Description:  This is a resistive touch screen driver that is using the
			   Microchip Graphics Library. The calibration values are
			   automatically checked (by reading a specific memory location
			   on the non-volatile memory) when initializing the module if the
			   function pointers to the read and write callback functions
			   are initialized. If the read value is invalid calibration
			   will automatically be executed. Otherwise, the calibration
			   values will be loaded and used.
			   The driver assumes that the application side provides the
			   read and write routines to a non-volatile memory.
			   If the callback functions are not initialized, the calibration
			   routine will always be called at startup to initialize the
			   global calibration values.
			   This driver assumes that the Graphics Library is initialized
			   and will be using the default font of the library.
 *****************************************************************************/

#ifndef _AD_TOUCH_H
#define _AD_TOUCH_H

#include "GenericTypeDefs.h"
#include "lv_drv_conf.h"
#include <stdint.h>
#include <stdbool.h>
#include "lvgl.h"



#define SAMPLE_POINTS   4

// Default calibration points
#define TOUCHCAL_ULX       107
#define TOUCHCAL_ULY       224
#define TOUCHCAL_URX       924
#define TOUCHCAL_URY       243
#define TOUCHCAL_LRX       913
#define TOUCHCAL_LRY       804
#define TOUCHCAL_LLX       106
#define TOUCHCAL_LLY       787
#define TOUCHCAL_DEF_OFST  20

typedef WORD (*t_NVM_Read)(DWORD);           // typedef for read function pointer
typedef void (*t_NVM_Write)(WORD, DWORD);    // typedef for write function pointer
typedef void (*t_NVM_SectorErase)(DWORD);    // typedef for sector erase function pointer#

typedef struct __attribute__ ((packed))
{
   lv_point_t  points[SAMPLE_POINTS];
   WORD        scn_ofst;      // location of calibration circles from corner of screen
   WORD        crc;
} t_Tpcal;




bool ADtouchInit(void);
bool 	ADtouchCheckForCalibration(void);
SHORT ADtouchDetectPosition(void);
bool ADtouchReadInput(lv_indev_drv_t * drv, lv_indev_data_t*data);
bool 	ADtouchStoreCalibration(t_Tpcal* pCal);
bool 	ADtouchLoadCalibration(t_Tpcal* pCal);
bool ADtouchRead(lv_indev_drv_t * drv, lv_indev_data_t*data);

// use this macro to debug the touch screen panel
// this will enable the use of debugging functions in the C file.
// It assumes that the graphics portion is working.
//#define ENABLE_DEBUG_TOUCHSCREEN


#endif //_TOUCHSCREEN_RESISTIVE_H
