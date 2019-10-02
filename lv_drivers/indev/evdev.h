/**
 * @file evdev.h
 *
 */

#ifndef EVDEV_H
#define EVDEV_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#ifndef LV_DRV_NO_CONF
#ifdef LV_CONF_INCLUDE_SIMPLE
#include "lv_drv_conf.h"
#else
#include "../../lv_drv_conf.h"
#endif
#endif


#if USE_EVDEV

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

/*********************
 *      DEFINES
 *********************/
#define SAMPLE_POINTS   4

// Default calibration points
#define TOUCHCAL_ULX       149
#define TOUCHCAL_ULY       825
#define TOUCHCAL_URX       898
#define TOUCHCAL_URY       852
#define TOUCHCAL_LRX       898
#define TOUCHCAL_LRY       210
#define TOUCHCAL_LLX       144
#define TOUCHCAL_LLY       193
#define TOUCHCAL_DEF_OFST  30

/**********************
 *      TYPEDEFS
 **********************/
typedef struct __attribute__ ((packed))
{
   lv_point_t  points[SAMPLE_POINTS];
   uint16_t        scn_ofst;      // location of calibration circles from corner of screen
   uint16_t        crc;
} t_Tpcal;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Initialize the evdev
 */
bool evdev_init(void);
/**
 * reconfigure the device file for evdev
 * @param dev_name set the evdev device filename
 * @return true: the device file set complete
 *         false: the device file doesn't exist current system
 */
bool evdev_set_file(char* dev_name);
/**
 * Get the current position and state of the evdev
 * @param data store the evdev data here
 * @return false: because the points are not buffered, so no more data to be read
 */
bool evdev_read(lv_indev_drv_t * drv, lv_indev_data_t * data);

bool  evdev_store_calibration(t_Tpcal* pCal);


bool  evdev_load_calibration(t_Tpcal* pCal);

/**********************
 *      MACROS
 **********************/

#endif /* USE_EVDEV */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* EVDEV_H */
