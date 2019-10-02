/*!
 AD_touch.c
 **/

#if USE_AD_TOUCH

#include "HardwareProfile.h"
#include "lv_drv_conf.h"
#include "AD_touch.h"
#include "utils/checksum.h"



#include "Compiler.h"

//////////////////////// MACROS ////////////////////////////
//#define CALIBRATIONOFFSET  20

#define SAMPLE_POINTS   4

//////////////////////// A/D Sampling Mode ///////////////////////
// first some error check
#if defined (RESISTIVETOUCH_MANUAL_SAMPLE_MODE) &&  defined (RESISTIVETOUCH_AUTO_SAMPLE_MODE)
#error Cannot have two resistive touch modes enabled.
#endif
#ifndef RESISTIVETOUCH_MANUAL_SAMPLE_MODE
// enable auto sampling mode
#define RESISTIVETOUCH_AUTO_SAMPLE_MODE
// else manual sampling mode is enabled
#endif

#ifndef TOUCHSCREEN_RESISTIVE_PRESS_THRESHOLD
// you may define the threshold with a value, define the new value in the
// HardwareProfile.h
#define TOUCHSCREEN_RESISTIVE_PRESS_THRESHOLD     256	// between 0-0x03ff the lesser this value
// the lighter the screen must be pressed
#endif
#define CALIBRATION_DELAY   300				                // delay between calibration touch points

#ifndef RESISTIVETOUCH_ANALOG
#define RESISTIVETOUCH_ANALOG  0
#endif
#ifndef RESISTIVETOUCH_DIGITAL
#define RESISTIVETOUCH_DIGITAL 1
#endif


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

//////////////////////// TYPES            ////////////////////////////
typedef enum
{
   IDLE,
   SET_X,
   RUN_X,
   GET_X,
   RUN_CHECK_X,
   CHECK_X,
   SET_Y,
   RUN_Y,
   GET_Y,
   CHECK_Y,
   SET_VALUES,
   GET_POT,
   RUN_POT
} TOUCH_STATES;


//////////////////////// LOCAL PROTOTYPES ////////////////////////////
static void TouchHardwareInit(void);
static void Touch_ADCInit(void);
static SHORT TouchGetRawX(void);
static SHORT TouchGetRawY(void);

const t_Tpcal defCal = {
{
   {TOUCHCAL_ULX, TOUCHCAL_ULY},
   {TOUCHCAL_URX, TOUCHCAL_URY},
   {TOUCHCAL_LRX, TOUCHCAL_LRY},
   {TOUCHCAL_LLX, TOUCHCAL_LLY},
}, TOUCHCAL_DEF_OFST, 0};

#ifdef ENABLE_DEBUG_TOUCHSCREEN
void    TouchScreenResistiveTestXY(void);
#endif

t_NVM_Read           pCalDataRead = NVM_READ_FUNC;                // function pointer to data read
t_NVM_Write          pCalDataWrite = NVM_WRITE_FUNC;               // function pointer to data write
t_NVM_SectorErase    pCalDataSectorErase = NVM_SECTOR_ERASE_FUNC;         // function pointer to data sector erase

//////////////////////// STORAGE ////////////////////////////

// Current ADC values for X and Y channels
volatile SHORT  adcX = -1;
volatile SHORT  adcY = -1;
volatile SHORT  adcPot = 0;



// copy of the stored or sampled raw points (this is the calibration data stored)
/*      xRawTouch[0] - x sample from upper left corner;
        xRawTouch[1] - x sample from upper right corner
        xRawTouch[2] - x sample from lower right corner
        xRawTouch[3] - x sample from lower left corner
        yRawTouch[0] - y sample from upper left corner;
        yRawTouch[1] - y sample from upper right corner
        yRawTouch[2] - y sample from lower right corner
        yRawTouch[3] - y sample from lower left corner
*/
volatile SHORT  xRawTouch[SAMPLE_POINTS] = {TOUCHCAL_ULX, TOUCHCAL_URX, TOUCHCAL_LRX, TOUCHCAL_LLX};
volatile SHORT  yRawTouch[SAMPLE_POINTS] = {TOUCHCAL_ULY, TOUCHCAL_URY, TOUCHCAL_LRY, TOUCHCAL_LLY};


volatile TOUCH_STATES state = IDLE;

t_Tpcal tpCal = {0};

//////////////////////// EXPORTED FUNCTIONS ////////////////////////////
/*********************************************************************
* Function: void TouchInit(NVM_WRITE_FUNC pWriteFunc, NVM_READ_FUNC pReadFunc, NVM_SECTORERASE_FUNC pSectorErase, void *initValues)
*
* PreCondition: none
*
* Input: pWriteFunc - non-volatile memory write function pointer
*        pReadFunc - non-volatile memory read function pointer
*        pSectorErase - non-volatile memory sector function pointer
*
* Output: none
*
* Side Effects: none
*
* Overview: Initializes the touch screen hardware.
*
* Note: none
*
********************************************************************/
bool ADtouchInit(void)
{

   TouchHardwareInit();


}

bool ADtouchCheckForCalibration(void)
{
   WORD count;

   bool calRequired = false;

   // check for calibration
   // this tests any touches on the touch screen, user has to touch the screen for more than 1
   // second to make the calibration work
   count = 0;
   while(1)
   {
      DelayMs(100);
      // check if there is a touch
      if ((TouchGetRawX() == -1) && (TouchGetRawY() == -1))
         break;
      else
         count++;
      if (count == 10)
      {
         calRequired = true;
         break;
      }
   }
   return calRequired;
}


/*********************************************************************
* Function: void TouchDetectPosition(void)
********************************************************************/
SHORT ADtouchDetectPosition(void)
{
   static SHORT    tempX, tempY;
   SHORT           temp;

   switch(state)
   {
   case IDLE:
      adcX = -1;
      adcY = -1;
#ifdef ADC_POT
      adcPot = 0;
#endif
      break;

   case SET_VALUES:
#ifdef RESISTIVETOUCH_MANUAL_SAMPLE_MODE
      TOUCH_ADC_START = 0;    // stop sampling
#endif
      if(!TOUCH_ADC_DONE)
         break;

      if((WORD) TOUCHSCREEN_RESISTIVE_PRESS_THRESHOLD < (WORD) ADC1BUF0)
      {
         adcX = -1;
         adcY = -1;
      }
      else
      {
         adcX = tempX;
         adcY = tempY;
      }
      // If the hardware supports an analog pot, if not skip it
#ifdef ADC_POT
      state = RUN_POT;

   case RUN_POT:
      TOUCH_ADC_INPUT_SEL = ADC_POT;
      TOUCH_ADC_START = 1;    // run conversion
      state = GET_POT;
      break;

   case GET_POT:
#ifdef RESISTIVETOUCH_MANUAL_SAMPLE_MODE
      TOUCH_ADC_START = 0;    // stop sampling
#endif
      if(TOUCH_ADC_DONE == 0) {
         break;
      }
      adcPot = ADC1BUF0;
#endif // #ifdef ADC_POT
      state = SET_X;
      return 1;               // touch screen acquisition is done

   case SET_X:

      TOUCH_ADC_INPUT_SEL = ADC_XPOS;

      ResistiveTouchScreen_XPlus_Config_As_Input();
      ResistiveTouchScreen_YPlus_Config_As_Input();
      ResistiveTouchScreen_XMinus_Config_As_Input();
      ResistiveTouchScreen_YMinus_Drive_Low();
      ResistiveTouchScreen_YMinus_Config_As_Output();

#ifdef ADPCFG_YPOS
      ADPCFG_YPOS = RESISTIVETOUCH_DIGITAL;        // set to digital pin
#endif
#ifdef ADPCFG_YPOS
      ADPCFG_XPOS = RESISTIVETOUCH_ANALOG;        // set to analog pin
#endif

      TOUCH_ADC_START = 1;    // run conversion
      state = CHECK_X;
      break;

   case CHECK_X:
   case CHECK_Y:
#ifdef RESISTIVETOUCH_MANUAL_SAMPLE_MODE
      TOUCH_ADC_START = 0;    // stop sampling
#endif
      if(TOUCH_ADC_DONE == 0) {
         break;
      }

      if((WORD) TOUCHSCREEN_RESISTIVE_PRESS_THRESHOLD > (WORD) ADC1BUF0)
      {
         if (state == CHECK_X)
         {
            ResistiveTouchScreen_YPlus_Drive_High();
            ResistiveTouchScreen_YPlus_Config_As_Output();
            tempX     = -1;
            state     = RUN_X;
         }
         else
         {
            ResistiveTouchScreen_XPlus_Drive_High();
            ResistiveTouchScreen_XPlus_Config_As_Output();
            tempY     = -1;
            state     = RUN_Y;
         }
      }
      else
      {
         adcX = -1;
         adcY = -1;
#ifdef ADC_POT
         state = RUN_POT;
#else
         state = SET_X;
         return 1;       // touch screen acquisition is done
#endif
         break;
      }

   case RUN_X:
   case RUN_Y:
      TOUCH_ADC_START = 1;
      state = (state == RUN_X) ? GET_X : GET_Y;
      // no break needed here since the next state is either GET_X or GET_Y
      break;

   case GET_X:
   case GET_Y:
#ifdef RESISTIVETOUCH_MANUAL_SAMPLE_MODE
      TOUCH_ADC_START = 0;    // stop sampling
#endif
      if(!TOUCH_ADC_DONE)
         break;

      temp = ADC1BUF0;
      if (state == GET_X)
      {
         if(temp != tempX)
         {
            tempX = temp;
            state = RUN_X;
            break;
         }
      }
      else
      {
         if(temp != tempY)
         {
            tempY = temp;
            state = RUN_Y;
            break;
         }
      }

      if (state == GET_X)
         ResistiveTouchScreen_YPlus_Config_As_Input();
      else
         ResistiveTouchScreen_XPlus_Config_As_Input();
      TOUCH_ADC_START = 1;
      state = (state == GET_X) ? SET_Y : SET_VALUES;
      break;

   case SET_Y:
#ifdef RESISTIVETOUCH_MANUAL_SAMPLE_MODE
      TOUCH_ADC_START = 0;    // stop sampling
#endif
      if(!TOUCH_ADC_DONE)
         break;

      if((WORD)  TOUCHSCREEN_RESISTIVE_PRESS_THRESHOLD < (WORD) ADC1BUF0)
      {
         adcX = -1;
         adcY = -1;
#ifdef ADC_POT
         state = RUN_POT;
#else
         state = SET_X;
         return 1;       // touch screen acquisition is done
#endif
         break;
      }

      TOUCH_ADC_INPUT_SEL = ADC_YPOS;

      ResistiveTouchScreen_XPlus_Config_As_Input();
      ResistiveTouchScreen_YPlus_Config_As_Input();
      ResistiveTouchScreen_XMinus_Drive_Low();
      ResistiveTouchScreen_XMinus_Config_As_Output();
      ResistiveTouchScreen_YMinus_Config_As_Input();

#ifdef ADPCFG_YPOS
      ADPCFG_YPOS = RESISTIVETOUCH_ANALOG;        // set to analog pin
#endif
#ifdef ADPCFG_YPOS
      ADPCFG_XPOS = RESISTIVETOUCH_DIGITAL;        // set to digital pin
#endif


      TOUCH_ADC_START = 1;    // run conversion

      state = CHECK_Y;
      break;

   default:
      state = SET_X;
      return 1;               // touch screen acquisition is done
   }

   return 0;                       // touch screen acquisition is not done
}

bool ADtouchReadInput(lv_indev_drv_t * drv, lv_indev_data_t*data)
{
   static SHORT x_prev = -1;
   static SHORT y_prev = -1;
   SHORT x,y;

   bool bPressed = true;

   x = TouchGetRawX();
   if(x >= 0)
   {
      data->point.x = x;
      x_prev = x;
   }
   else
   {
      bPressed = false;
      data->point.x = x_prev;
   }

   y = TouchGetRawY();
   if(y >= 0)
   {
      data->point.y = y;
      y_prev = y;
   }
   else
   {
      bPressed = false;
      data->point.y = y_prev;
   }

   if(bPressed)
   {
      data->state = LV_INDEV_STATE_PR;
   }
   else
   {
      data->state = LV_INDEV_STATE_REL;
   }

   return false;
}

bool ADtouchStoreCalibration(t_Tpcal* pCal)
{
   bool bSuccess = false;
   DWORD nvAddr = ADDRESS_RESISTIVE_TOUCH_CAL;
   if (pCalDataWrite != NULL)
   {
      // the upper left X sample address is used since it is the first one
      // and this assumes that all stored values are located in one
      // sector
      if (pCalDataSectorErase != NULL)
      {
         pCalDataSectorErase(nvAddr);

         WORD i;
         for (i = 0; i < SAMPLE_POINTS; i++)
         {
            pCalDataWrite(pCal->points[i].x, nvAddr);
            nvAddr += sizeof(lv_coord_t);
            pCalDataWrite(pCal->points[i].y, nvAddr);
            nvAddr += sizeof(lv_coord_t);
         }
         pCalDataWrite(pCal->scn_ofst, nvAddr);
         nvAddr += sizeof(WORD);
         WORD crc = crc_16((unsigned char*)pCal->points, (sizeof(t_Tpcal) - sizeof(WORD)));
         pCalDataWrite(crc, nvAddr);
         bSuccess = true;
         
         // copy into active calibration points
         memcpy(tpCal.points, pCal->points, sizeof(tpCal.points));
         tpCal.scn_ofst = pCal->scn_ofst;
         tpCal.crc = crc;
      }

   }
   return bSuccess;
}


bool ADtouchLoadCalibration(t_Tpcal* pCal)
{
   DWORD nvAddr = ADDRESS_RESISTIVE_TOUCH_CAL;
   
   bool bSuccess = false;

   if (pCalDataRead != NULL)
   {
      WORD i;
      for (i = 0; i < SAMPLE_POINTS; i++)
      {
         pCal->points[i].x = pCalDataRead(nvAddr);
         nvAddr += sizeof(lv_coord_t);
         pCal->points[i].y = pCalDataRead(nvAddr);
         nvAddr += sizeof(lv_coord_t);
      }
      // Read calibration points offset
      pCal->scn_ofst = pCalDataRead(nvAddr);
      nvAddr += sizeof(WORD);

      // Read CRC
      pCal->crc = pCalDataRead(nvAddr);

      // Now check the CRC
      WORD crc_check = crc_16((unsigned char*)pCal->points, (sizeof(t_Tpcal) - sizeof(WORD)));

      if(crc_check == pCal->crc)
      {
         // TODO add calibration offset to user data, or make it a parameter when initialising
         bSuccess = true;
      }
      else
      {
         /* load default calibration if cal read failed */
         memcpy(pCal, &defCal, sizeof(defCal));
      }
   }
   
   return bSuccess;
}

bool ADtouchRead(lv_indev_drv_t * drv, lv_indev_data_t*data)
{
   static SHORT x_prev, y_prev;
   short x,y;
   bool bPressed = true;

   x = TouchGetRawX();
   if(x >= 0)
   {
      data->point.x = x;
      x_prev = x;
   }
   else
   {
      data->point.x = x_prev;
      bPressed = false;
   }

   y = TouchGetRawY();
   if( y >= 0)
   {
      data->point.y = y;
      y_prev = y;
   }
   else
   {
      data-> point.y = y_prev;
      bPressed = false;
   }

   if(bPressed)
   {
      data->state = LV_INDEV_STATE_PR;
   }
   else
   {
      data->state = LV_INDEV_STATE_REL;
   }

   return false;
}

//////////////////////// PRIVATE FUNCTIONS ////////////////////////////


/*********************************************************************
* Function: void TouchHardwareInit(void)
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: Initializes touch screen module.
*
* Note: none
*
********************************************************************/
void TouchHardwareInit(void)
{
   Touch_ADCInit();

   // set the used D/A port to be analog
#ifdef ADPCFG_XPOS
   ADPCFG_XPOS = RESISTIVETOUCH_ANALOG;
#endif
#ifdef ADPCFG_YPOS
   ADPCFG_YPOS = RESISTIVETOUCH_ANALOG;
#endif
#ifdef ADC_POT
   ADC_POT_PCFG = RESISTIVETOUCH_ANALOG;
#endif

   AD1CSSL = 0;            // No scanned inputs

   state = SET_X;          // set the state of the statemachine to start the sampling

}

/*********************************************************************
* Function: void Touch_ADCInit(void)
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: Initializes the A/D channel used for the touch detection.
*
* Note: none
*
********************************************************************/
void Touch_ADCInit(void)
{
#if defined (RESISTIVETOUCH_AUTO_SAMPLE_MODE)
   // Initialize ADC for auto sampling mode
   AD1CON1 = 0;            // reset
   AD1CON2 = 0;            // AVdd, AVss, int every conversion, MUXA only
   AD1CON3 = 0x1FFF;       // 31 Tad auto-sample, Tad = 256*Tcy
   AD1CON1 = 0x80E0;       // Turn on A/D module, use auto-convert
#elif defined (RESISTIVETOUCH_MANUAL_SAMPLE_MODE)
   // Initialize ADC for manual sampling mode
   AD1CON1 = 0;            // reset
   AD1CON2 = 0;            // AVdd, AVss, int every conversion, MUXA only
   AD1CON3 = 0x0101;       // 1 Tad auto-sample, Tad = 2*Tcy
   AD1CON1 = 0x8000;       // Turn on A/D module, use manual convert
#else
#error Resistive Touch sampling mode is not set.
#endif

}


/*********************************************************************
* Function: SHORT TouchGetRawX()
*
* PreCondition: none
*
* Input: none
*
* Output: x coordinate
*
* Side Effects: none
*
* Overview: returns x coordinate if touch screen is pressed
*           and -1 if not
*
* Note: none
*
********************************************************************/
SHORT TouchGetRawX(void)
{
#ifdef TOUCHSCREEN_RESISTIVE_SWAP_XY
   return adcY;
#else
   return adcX;
#endif
}


/*********************************************************************
* Function: SHORT TouchGetRawY()
*
* PreCondition: none
*
* Input: none
*
* Output: y coordinate
*
* Side Effects: none
*
* Overview: returns y coordinate if touch screen is pressed
*           and -1 if not
*
* Note: none
*
********************************************************************/
SHORT TouchGetRawY(void)
{
#ifdef TOUCHSCREEN_RESISTIVE_SWAP_XY
   return adcX;
#else
   return adcY;
#endif
}









#endif // #if defined (USE_TOUCHSCREEN_RESISTIVE)

