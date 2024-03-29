#include "lvgl/lvgl.h"
#include "lv_drivers/display/fbdev.h"
#include "lv_drivers/indev/touch.h"
#include "lv_examples/lv_apps/demo/demo.h"
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <stdbool.h>
#include "lv_examples/lv_apps/tpcal/tpcal.h"
#include <stdlib.h>
#include <stdio.h>

#define DISP_BUF_SIZE (80*LV_HOR_RES_MAX)

static enum {APP_DEMO, APP_TP_CAL} eAppState;
lv_indev_drv_t  indev_drv;              // input device driver
lv_indev_t *  indev;


int main(void)
{
    /*LittlevGL init*/
    lv_init();

    /*Linux frame buffer device init*/
    fbdev_init();

#if 1

    /* Touchscreen driver device init */
    int32_t drvStatus = touchInit();
    if(drvStatus == TOUCH_DRV_FAIL)
    {
       perror("Cannot initialise touch screen driver\r\n");
       exit(1);
    }
#endif

    /*A small buffer for LittlevGL to draw the screen's content*/
    static lv_color_t buf[DISP_BUF_SIZE];

    /*Initialize a descriptor for the buffer*/
    static lv_disp_buf_t disp_buf;
    lv_disp_buf_init(&disp_buf, buf, NULL, DISP_BUF_SIZE);

    /*Initialize and register a display driver*/
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.buffer = &disp_buf;
    disp_drv.flush_cb = fbdev_flush;
    lv_disp_drv_register(&disp_drv);

#if 1
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchRead;
    //indev_drv.user_data = (void*)&touchCalFunc;
    indev = lv_indev_drv_register(&indev_drv);
#endif

    /*Create a Demo*/
    demo_create();

#if 1
    if(drvStatus == TOUCH_CAL_REQ)
       tpcal_create(indev);
#endif


    /*Handle LitlevGL tasks (tickless mode)*/
    while(1) {
        lv_task_handler();
        usleep(5000);
    }

    return 0;
}


/*Set in lv_conf.h as `LV_TICK_CUSTOM_SYS_TIME_EXPR`*/
uint32_t custom_tick_get(void)
{
    static uint64_t start_ms = 0;
    if(start_ms == 0) {
        struct timeval tv_start;
        gettimeofday(&tv_start, NULL);
        start_ms = (tv_start.tv_sec * 1000000 + tv_start.tv_usec) / 1000;
    }

    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    uint64_t now_ms;
    now_ms = (tv_now.tv_sec * 1000000 + tv_now.tv_usec) / 1000;

    uint32_t time_ms = now_ms - start_ms;
    return time_ms;
}
