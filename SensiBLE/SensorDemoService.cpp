#include "SensorDemoService.h"

/** @brief Macro that stores Value into a buffer in Little Endian Format (2 bytes)*/
#define HOST_TO_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

/** @brief Macro that stores Value into a buffer in Little Endian Format (4 bytes) */
#define HOST_TO_LE_32(buf, val)    ( ((buf)[0] =  (uint8_t) (val)     ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[2] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[3] =  (uint8_t) (val>>24) ) )

/*Macros for UUID*/
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
{\
  uuid_struct[0 ] = uuid_0 ; uuid_struct[1 ] = uuid_1 ; uuid_struct[2 ] = uuid_2 ; uuid_struct[3 ] = uuid_3 ; \
  uuid_struct[4 ] = uuid_4 ; uuid_struct[5 ] = uuid_5 ; uuid_struct[6 ] = uuid_6 ; uuid_struct[7 ] = uuid_7 ; \
  uuid_struct[8 ] = uuid_8 ; uuid_struct[9 ] = uuid_9 ; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
  uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}

/* Hardware Characteristics Service */
#define COPY_HW_SENS_W2ST_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0xE0,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ECOMPASS_W2ST_CHAR_UUID(uuid_struct)      COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x40,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_UV_W2ST_CHAR_UUID(uuid_struct)            COPY_UUID_128(uuid_struct,0x11,0x11,0x11,0x11,0x00,0x02,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_PROX_W2ST_CHAR_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x02,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
/* Software Characteristics Service */
#define COPY_SW_SENS_W2ST_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x02,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_QUATERNIONS_W2ST_CHAR_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x01,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

//Callback function
void SensorDemo_HCI_Event_CB(void *pckt);

int SensorDemoService::begin()
{
  const char BoardName[9] = {SENSOR_DEMO_NAME,0};
  int ret;
  uint8_t  hwVersion;
  uint16_t fwVersion;
  HCI_Init();

  /*Attach callback function*/
  attach_HCI_CB(SensorDemo_HCI_Event_CB);

  BlueNRG_RST();
  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  BlueNRG_RST();

  /*Generate MAC Address*/
  uint8_t data_len_out;
  ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, 6, &data_len_out, bdaddr);

  ret = aci_gatt_init();
  if(ret)
  {
   Serial.print("\r\nGATT_Init failed\r\n");
   goto fail;
  }

  ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);

  if(ret != BLE_STATUS_SUCCESS)
  {
   Serial.print("\r\nGAP_Init failed\r\n");
   goto fail;
  }
  ret = hci_le_set_random_address(bdaddr);

  if(ret)
  {
   Serial.print("\r\nSetting the Static Random BD_ADDR failed\r\n");
   goto fail;
  }
  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                   7, (uint8_t *)BoardName);
  if(ret)
  {
   Serial.print("\r\naci_gatt_update_char_value failed\r\n");
   while(1);
  }

  ret=aci_gap_clear_security_database();
  if (ret != BLE_STATUS_SUCCESS)
  {
   Serial.print("\r\nGAP clear security database failed\r\n");
   goto fail;
  }
  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                   OOB_AUTH_DATA_ABSENT,
                   NULL, 7, 16,
                   USE_FIXED_PIN_FOR_PAIRING, 123456,
                   NO_BONDING);
  if (ret != BLE_STATUS_SUCCESS)
  {
   Serial.print("\r\nGAP setting Authentication failed\r\n");
   goto fail;
  }

  /* Set output power level */
  aci_hal_set_tx_power_level(1,4);

  /*Add services*/
  ret = Add_HWServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
   Serial.print("BlueMS HW Service W2ST added successfully\r\n");
  }
  else
  {
   Serial.print("\r\nError while adding HW Service W2ST\r\n");
  }

  ret = Add_SWServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS) {
  Serial.print("BlueMS SW service added successfully\r\n");
  } else {
  Serial.print("\r\nError while adding SW Service W2ST\r\n");
  }

  StartBlueNRG = TRUE;
  set_connectable = TRUE;

  return ret;

fail:
  return ret;
}

/*Enables the device for the connection*/
int SensorDemoService::setConnectable()
{
  char local_name[9] = {AD_TYPE_COMPLETE_LOCAL_NAME,SENSOR_DEMO_NAME};
  uint8_t manuf_data[27] = {
  2,0x0A,0x00, /* 0 dBm */  // Trasmission Power
  9,0x09,SENSOR_DEMO_NAME,  // Complete Name
  13,0xFF,0x01, /* SKD version */
  0x82,
  0x00,
  0xF4, /* ACC+Gyro+Mag 0xE0 | 0x04 Temp | 0x10 Pressure */
  0x01, /* Sensor Fusion 0x01*/
  0x00, /*  */
  bdaddr[5], /* BLE MAC start -MSB first- */
  bdaddr[4],
  bdaddr[3],
  bdaddr[2],
  bdaddr[1],
  bdaddr[0]  /* BLE MAC stop */
  };

//  manuf_data[18] |= 0x01; /* Sensor Fusion */

  hci_le_set_scan_resp_data(0,NULL);
  aci_gap_set_discoverable(ADV_IND, 0, 0, STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE, sizeof(local_name), local_name, 0, NULL, 0, 0);
  aci_gap_update_adv_data(27, manuf_data);
  set_connectable=FALSE;

  return 0;
}

/* Update the accelerometer gyroscope and magnetometer */
tBleStatus SensorDemoService::AccGyroMag_Update()
{
  uint8_t buff[2+2*3*3];
  tBleStatus ret;

  HOST_TO_LE_16(buff,(millis()>>3));

  HOST_TO_LE_16(buff+2,-x_axes.AXIS_Y);
  HOST_TO_LE_16(buff+4, x_axes.AXIS_X);
  HOST_TO_LE_16(buff+6,-x_axes.AXIS_Z);

  HOST_TO_LE_16(buff+8,g_axes.AXIS_Y);
  HOST_TO_LE_16(buff+10,g_axes.AXIS_X);
  HOST_TO_LE_16(buff+12,g_axes.AXIS_Z);

  HOST_TO_LE_16(buff+14,m_axes.AXIS_Y);
  HOST_TO_LE_16(buff+16,m_axes.AXIS_X);
  HOST_TO_LE_16(buff+18,m_axes.AXIS_Z);

  ret = aci_gatt_update_char_value(HWServW2STHandle, AccGyroMagCharHandle, 0, 2+2*3*3, buff);
  if (ret != BLE_STATUS_SUCCESS){
    Serial.print("Error while updating Acceleration characteristic\r\n") ;
    return BLE_STATUS_ERROR ;
  }

  return BLE_STATUS_SUCCESS;
}

tBleStatus SensorDemoService::Quat_Update()
{
  tBleStatus ret;
  uint8_t buff[2+6];

  q_axes.AXIS_X = x_axes.AXIS_X;
  q_axes.AXIS_Y = x_axes.AXIS_Y;
  q_axes.AXIS_Z = x_axes.AXIS_Z;

  HOST_TO_LE_16(buff,(millis()>>3));

  HOST_TO_LE_16(buff+2,q_axes.AXIS_X);
  HOST_TO_LE_16(buff+4,q_axes.AXIS_Y);
  HOST_TO_LE_16(buff+6,q_axes.AXIS_Z);

  ret = aci_gatt_update_char_value(SWServW2STHandle, QuaternionsCharHandle, 0, 2+6, buff);
  if (ret != BLE_STATUS_SUCCESS){
    Serial.print("Error while updating Sensor Fusion characteristic\r\n") ;
    return BLE_STATUS_ERROR ;
  }

  return BLE_STATUS_SUCCESS;
}

tBleStatus SensorDemoService::BlueMS_Environmental_Update(int32_t press, int16_t temp)
{
  tBleStatus ret;
  uint8_t buff[8];
  HOST_TO_LE_16(buff, millis()>>3);

  HOST_TO_LE_32(buff+2,press);
  HOST_TO_LE_16(buff+6,temp);

  ret = aci_gatt_update_char_value(HWServW2STHandle, EnvironmentalCharHandle, 0, 8, buff);

  if (ret != BLE_STATUS_SUCCESS){
    Serial.print("Error while updating TEMP characteristic\r\n") ;
    return BLE_STATUS_ERROR ;
  }

  return BLE_STATUS_SUCCESS;
}

tBleStatus SensorDemoService::Uv_Update(uint16_t UvValue)
{
  tBleStatus ret;
  uint8_t buff[2+2];
  
  HOST_TO_LE_16(buff, millis()>>3);
  HOST_TO_LE_16(buff+2, UvValue);

  ret = aci_gatt_update_char_value(HWServW2STHandle, UvCharHandle, 0, 2+2, buff);

  if (ret != BLE_STATUS_SUCCESS){
    Serial.print("Error while updating Uv characteristic\r\n") ;
    return BLE_STATUS_ERROR ;
  }

  return BLE_STATUS_SUCCESS;
}

void SensorDemoService::Read_Request_CB(uint16_t handle)
{
  tBleStatus ret;

  if(handle == AccGyroMagCharHandle + 1)
  {
    AccGyroMag_Update();
  }
  else if (handle == EnvironmentalCharHandle + 1)
  {
    float data_t, data_p;
    data_t = 27.0 + ((uint64_t)rand()*5)/RAND_MAX; //T sensor emulation
    data_p = 1000.0 + ((uint64_t)rand()*100)/RAND_MAX; //P sensor emulation
    BlueMS_Environmental_Update((int32_t)(data_p *100), (int16_t)(data_t * 10));
  }

  if(connection_handle !=0)
  {
    ret = aci_gatt_allow_read(connection_handle);
    if (ret != BLE_STATUS_SUCCESS)
    {
    Serial.print("aci_gatt_allow_read() failed\r\n");
    }
  }
}

void SensorDemoService::Set_Random_Environmental_Values(float *data_t, float *data_p)
{
  *data_t = 27.0 + ((uint64_t)rand()*5)/RAND_MAX;     /* T sensor emulation */
  *data_p = 1000.0 + ((uint64_t)rand()*80)/RAND_MAX; /* P sensor emulation */
}

void SensorDemoService::Set_Motion_Values(int32_t *x, int32_t *g, int32_t *m)
{
  x_axes.AXIS_X = x[0];
  x_axes.AXIS_Y = x[1];
  x_axes.AXIS_Z = x[2];

  g_axes.AXIS_X = g[0];
  g_axes.AXIS_Y = g[1];
  g_axes.AXIS_Z = g[2];

  m_axes.AXIS_X = m[0];
  m_axes.AXIS_Y = m[1];
  m_axes.AXIS_Z = m[2];
}

void SensorDemoService::Reset_Motion_Values(void)
{
  x_axes.AXIS_X = (x_axes.AXIS_X)%2000 == 0 ? -x_axes.AXIS_X : 10;
  x_axes.AXIS_Y = (x_axes.AXIS_Y)%2000 == 0 ? -x_axes.AXIS_Y : -10;
  x_axes.AXIS_Z = (x_axes.AXIS_Z)%2000 == 0 ? -x_axes.AXIS_Z : 10;
  g_axes.AXIS_X = (g_axes.AXIS_X)%2000 == 0 ? -g_axes.AXIS_X : 100;
  g_axes.AXIS_Y = (g_axes.AXIS_Y)%2000 == 0 ? -g_axes.AXIS_Y : -100;
  g_axes.AXIS_Z = (g_axes.AXIS_Z)%2000 == 0 ? -g_axes.AXIS_Z : 100;
  m_axes.AXIS_X = (g_axes.AXIS_X)%2000 == 0 ? -m_axes.AXIS_X : 3;
  m_axes.AXIS_Y = (g_axes.AXIS_Y)%2000 == 0 ? -m_axes.AXIS_Y : -3;
  m_axes.AXIS_Z = (g_axes.AXIS_Z)%2000 == 0 ? -m_axes.AXIS_Z : 3;
  q_axes.AXIS_X = -q_axes.AXIS_X;
  q_axes.AXIS_Y = -q_axes.AXIS_Y;
  q_axes.AXIS_Z = -q_axes.AXIS_Z;
}

tBleStatus SensorDemoService::Add_HWServW2ST_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];

  /* Add_HWServW2ST_Service */
  COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
  memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid.Service_UUID_128, PRIMARY_SERVICE,
                         1+3*6, &HWServW2STHandle); //1+3*5
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;

  /* Fill the Environmental BLE Characteristc */
  COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid);
  uuid[14] |= 0x04; /* One Temperature value*/
  uuid[14] |= 0x10; /* Pressure value*/
  memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                          2+2+4,
                          CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          16, 0, &EnvironmentalCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;

  /* Fill the AccGyroMag BLE Characteristc */
  COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid);
  memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                          2+2,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          16, 0, &ECompassCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;

  /* Fill the Lux Characteristc */
//  COPY_CO_LUX_W2ST_CHAR_UUID(uuid);
//  memcpy(&char_uuid.Char_UUID_128, uuid, 16);
//  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
//                          2+2+4,
//                          CHAR_PROP_NOTIFY|CHAR_PROP_READ,
//                          ATTR_PERMISSION_NONE,
//                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
//                          16, 0, &AccGyroMagCharHandle);
//  if (ret != BLE_STATUS_SUCCESS)
//    return BLE_STATUS_ERROR;

  return BLE_STATUS_SUCCESS;
}

tBleStatus SensorDemoService::Add_SWServW2ST_Service(void)
{
  tBleStatus ret;
  int32_t NumberOfRecords=2;
  uint8_t uuid[16];

  COPY_SW_SENS_W2ST_SERVICE_UUID(uuid);
  memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid.Service_UUID_128, PRIMARY_SERVICE,
                         1+3*NumberOfRecords, &SWServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  /* Fill the Quaternions Characteristc */
  COPY_QUATERNIONS_W2ST_CHAR_UUID(uuid);
  memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SWServW2STHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                          2+6,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          16, 0, &QuaternionsCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  /* Fill the ECompass Characteristc */
  COPY_ECOMPASS_W2ST_CHAR_UUID(uuid);
  memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                          2+2,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          16, 0, &AccGyroMagCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;

  return BLE_STATUS_SUCCESS;

fail:
  return BLE_STATUS_ERROR;
}

SensorDemoService SensorDemo;
