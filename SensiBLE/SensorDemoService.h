/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSORDEMOSERVICE_H
#define SENSORDEMOSERVICE_H

#include "hci.h"
#include "hci_le.h"
#include "hci_const.h"
#include "sm.h"
#include "stm32_bluenrg_ble.h"
#include "bluenrg_utils.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gap.h"
#include "ble_status.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"
#include "link_layer.h"
#include "SPBTLE_RF.h"

/**
 * @brief Structure containing acceleration value of each axis.
 */
typedef struct {
  int32_t AXIS_X;
  int32_t AXIS_Y;
  int32_t AXIS_Z;
} AxesRaw_t;

enum {
  ACCELERATION_SERVICE_INDEX = 0,
  ENVIRONMENTAL_SERVICE_INDEX = 1
};

/** Documentation for C union Service_UUID_t */
typedef union Service_UUID_t_s {
  /** 16-bit UUID
  */
  uint16_t Service_UUID_16;
  /** 128-bit UUID
  */
  uint8_t Service_UUID_128[16];
} Service_UUID_t;

/** Documentation for C union Char_UUID_t */
typedef union Char_UUID_t_s {
  /** 16-bit UUID
  */
  uint16_t Char_UUID_16;
  /** 128-bit UUID
  */
  uint8_t Char_UUID_128[16];
} Char_UUID_t;

#define SENSOR_DEMO_NAME   'S','e','n','s','i','B','L','E'
#define AD_TYPE_COMPLETE_LOCAL_NAME          (0x09)

//Class for bluetooth communication and services
class SensorDemoService
{
public:
   SensorDemoService(void)
   {
      StartBlueNRG = FALSE;
      set_connectable= FALSE;
      connected = FALSE;
      x_axes.AXIS_X = 0;
      x_axes.AXIS_Y = 0;
      x_axes.AXIS_Z = 0;
      g_axes.AXIS_X = 0;
      g_axes.AXIS_Y = 0;
      g_axes.AXIS_Z = 0;
      m_axes.AXIS_X = 0;
      m_axes.AXIS_Y = 0;
      m_axes.AXIS_Z = 0;
      q_axes.AXIS_X = 0;
      q_axes.AXIS_Y = 0;
      q_axes.AXIS_Z = 0;
   }

   /*Initializes the sensor with the default params*/
   int begin();

   /*Enables the device for the connection*/
   int setConnectable();

   /* Update the accelerometer gyroscope and magnetometer */
   tBleStatus AccGyroMag_Update();
   /**
    * @brief  Update quaternions characteristic value
    * @param  SensorAxes_t *data Structure containing the quaterions
    * @retval tBleStatus      Status
    */
   tBleStatus Quat_Update();

   tBleStatus BlueMS_Environmental_Update(int32_t press, int16_t temp);

   tBleStatus Uv_Update(uint16_t UvValue);

   /*******************************************************************************
    * Function Name  : Read_Request_CB.
    * Description    : Update the sensor valuse.
    * Input          : Handle of the characteristic to update.
    * Return         : None.
    *******************************************************************************/
   void Read_Request_CB(uint16_t handle);

   /**
    * @brief  Set random values for all environmental sensor data
    * @param  float pointer to temperature data
    * @param  float pointer to pressure data
    * @retval None
    */
   void Set_Random_Environmental_Values(float *data_t, float *data_p);

   /**
    * @brief  Set random values for all motion sensor data
    * @param  uint32_t counter for changing the rotation direction
    * @retval None
    */
   void Set_Motion_Values(int32_t *x, int32_t *g, int32_t *m);

   /**
    * @brief  Reset values for all motion sensor data
    * @param  None
    * @retval None
    */
   void Reset_Motion_Values(void);



   /*Public variables*/
   uint8_t set_connectable;
   int connected;
   uint8_t StartBlueNRG;
   AxesRaw_t x_axes;
   AxesRaw_t g_axes;
   AxesRaw_t m_axes;
   AxesRaw_t q_axes;

private:

   /*Add the services*/
   tBleStatus Add_HWServW2ST_Service(void);

   /**
    * @brief  Add the SW Feature service using a vendor specific profile
    * @param  None
    * @retval tBleStatus Status
    */
   tBleStatus Add_SWServW2ST_Service(void);

   /*Private variables*/
   uint8_t bdaddr[6];
   uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
   uint16_t HWServW2STHandle, EnvironmentalCharHandle, AccGyroMagCharHandle, ECompassCharHandle, UvCharHandle, ProxCharHandle;
   uint16_t SWServW2STHandle, QuaternionsCharHandle;

   /* UUIDS */
   Service_UUID_t service_uuid;
   Char_UUID_t char_uuid;

   uint16_t connection_handle;
   uint32_t start_time;

};

extern SensorDemoService SensorDemo;

#endif /* SENSORDEMOSERVICE_H */
