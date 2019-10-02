/*
 * touch.c
 *
 *  Created on: 29 Sep 2019
 *      Author: Rob
 */

/*********************
 *      INCLUDES
 *********************/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "../../lvgl/lvgl.h"
#include "../../lv_drv_conf.h"
// TODO change the following #include so that we can work with any touchscreen device

#include "touch.h"

#ifdef DEBUG
#define DEBUG_PRINT(fmt, args...)    printf(fmt, ## args)
#else
#define DEBUG_PRINT(fmt, args...)    /* Don't do anything in release builds */
#endif


/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void touchCalculateCalpoints(t_Tpcal* pCal);
static bool  touchLoadCalibration(void);


/**********************
 *  STATIC VARIABLES
 **********************/

static int32_t ymt, xmt;
static int32_t xmd, ymd;
static int32_t xc, yc;

static const t_Tpcal defCal = {
{
   {TOUCHCAL_ULX, TOUCHCAL_ULY},
   {TOUCHCAL_URX, TOUCHCAL_URY},
   {TOUCHCAL_LRX, TOUCHCAL_LRY},
   {TOUCHCAL_LLX, TOUCHCAL_LLY},
}, TOUCHCAL_DEF_OFST, 0};

static t_Tpcal tpCal = {0};

static bool (*pdev_init)(void) = TOUCH_INIT;
static bool (*pdev_read)(lv_indev_drv_t * drv, lv_indev_data_t * data) = TOUCH_READ;
static bool (*dev_store_calibration)(t_Tpcal* pCal) = TOUCH_STORE_CAL;
static bool (*dev_load_calibration)(t_Tpcal* pCal) = TOUCH_LOAD_CAL;
static bool (*dev_check_cal_req)(void) = TOUCH_CHECK_CAL;


/**********************
 *   GLOBAL FUNCTIONS
 **********************/
int32_t touchInit(void)
{
   bool calRequired = false;

   if(!pdev_init())
      return TOUCH_DRV_FAIL;

   if(!touchLoadCalibration())
   {
      calRequired = true;
   }
   else if(dev_check_cal_req != NULL)
   {
      if(dev_check_cal_req())
         calRequired = true;
   }

   if(calRequired)
      return TOUCH_CAL_REQ;
   else
      return TOUCH_INIT_OK;

}


bool touchRead(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
   bool bStatus = pdev_read(drv, data);

   if(data->state == LV_INDEV_STATE_PR)
   {
      DEBUG_PRINT("raw x:%d. raw y: %d\n", data->point.x, data->point.y);
   }

   // scaling
   int32_t x = xc + (((int32_t)data->point.x * xmt)/xmd);
   data->point.x = (lv_coord_t)x;

   int32_t y = yc + (((int32_t)data->point.y * ymt)/ymd);
   data->point.y = (lv_coord_t)y;

   if(data->point.x < 0)
     data->point.x = 0;
   if(data->point.y < 0)
     data->point.y = 0;
   if(data->point.x >= lv_disp_get_hor_res(drv->disp))
     data->point.x = lv_disp_get_hor_res(drv->disp) - 1;
   if(data->point.y >= lv_disp_get_ver_res(drv->disp))
     data->point.y = lv_disp_get_ver_res(drv->disp) - 1;

   if(data->state == LV_INDEV_STATE_PR)
   {
      DEBUG_PRINT("x: %d, y: %d\n", data->point.x, data->point.y);
   }

   return bStatus;
}


bool touchReadRaw(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
   bool bStatus = pdev_read(drv, data);

   return bStatus;
}



bool touchDoCalibration(lv_point_t* pPoints, uint16_t ofst)
{
   bool bSuccess = false;
   t_Tpcal cal;
   uint32_t i;
   
   for(i = 0; i < SAMPLE_POINTS; i++)
   {
      DEBUG_PRINT("x: %d, y: %d\r\n", pPoints[i].x, pPoints[i].y);
   }

   memcpy(cal.points, pPoints, SAMPLE_POINTS*sizeof(lv_point_t));
   cal.scn_ofst = ofst;

   DEBUG_PRINT("ofst:%d\n", ofst);
   DEBUG_PRINT("cal.scn_ofst:%d\n", cal.scn_ofst);

   touchCalculateCalpoints(&cal);

   if(dev_store_calibration(&cal))
      bSuccess = true;

   return bSuccess;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/


static void touchCalculateCalpoints(t_Tpcal* pCal)  //lv_point_t* pPoints, WORD ofst)
{
   // ofst is the location of the calibation circle from screen edge
   int32_t trA, trB, trC, trD;                    // variables for the coefficients
   int32_t trAhold, trBhold, trChold, trDhold;
   int32_t test1, test2;                          // temp variables (must be signed type)

   DEBUG_PRINT("scn_ofst:%d\n", pCal->scn_ofst);


   lv_point_t scrPoints[SAMPLE_POINTS] =
   {  {pCal->scn_ofst, pCal->scn_ofst},                             // Top left
      {LV_HOR_RES_MAX - 1 - pCal->scn_ofst, pCal->scn_ofst},       // Top right
      {LV_HOR_RES_MAX - 1 - pCal->scn_ofst, LV_VER_RES_MAX  - 1 - pCal->scn_ofst},  // Bottom right
      {pCal->scn_ofst, LV_VER_RES_MAX  - 1 - pCal->scn_ofst},       // bottom left
   };

   // xslope1
   int32_t xmd_1 = pCal->points[1].x - pCal->points[0].x;
   int32_t xmd_2 = pCal->points[2].x - pCal->points[3].x;

   DEBUG_PRINT("xmd1:%d, xmd2:%d\n", xmd_1, xmd_2);

   xmd = (xmd_1 + xmd_2)/2;
   xmt = LV_HOR_RES_MAX - (2 * pCal->scn_ofst);

   // yslope
   int32_t ymd_1 = pCal->points[3].y - pCal->points[0].y;
   int32_t ymd_2 = pCal->points[2].y - pCal->points[1].y;

   DEBUG_PRINT("ymd1:%d, ymd2:%d\n", ymd_1, ymd_2);

   ymd = (ymd_1 + ymd_2)/2;
   ymt = LV_VER_RES_MAX - (2 * pCal->scn_ofst);

   // calculate transfer function for x upper
   // x = (xmt * raw)/xmd + xc
   // rearranging we get
   int32_t xc1 = LV_HOR_RES_MAX;
   xc1 -= ((xmt * (pCal->points[0].x + pCal->points[1].x))/xmd_1);
   xc1 /= 2;

   // Now do x-lower
   int32_t xc2 = LV_HOR_RES_MAX;
   xc2 -= ((xmt * (pCal->points[2].x + pCal->points[3].x))/xmd_2);
   xc2 /= 2;

   DEBUG_PRINT("xc1:%d, xc2:%d\n", xc1, xc2);

   // y right
   int32_t yc1 = LV_VER_RES_MAX;
   yc1 -= ((ymt * (pCal->points[3].y + pCal->points[0].y))/ymd_1);
   yc1 /= 2;

   // y left
   int32_t yc2 = LV_VER_RES_MAX;
   yc2 -= ((ymt * (pCal->points[2].y + pCal->points[1].y))/ymd_2);
   yc2 /= 2;

   DEBUG_PRINT("yc1:%d, yc2:%d\n", yc1, yc2);

   // take averages
   yc = (yc1 + yc2)/2;
   xc = (xc1 + xc2)/2;

   DEBUG_PRINT("xmt:%d ymt:%d, xmd: %d, ymd:%d, xc:%d, yc:%d\n", xmt, ymt, xmd, ymd, xc, yc);

}


static bool  touchLoadCalibration(void)
{
   bool bSuccess = dev_load_calibration(&tpCal);
   
   if(bSuccess)
   {
      touchCalculateCalpoints(&tpCal);     
   }
 
   return bSuccess;

}


