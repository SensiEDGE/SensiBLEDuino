/*

 X_NUCLEO_IDB05A1_SensorDemo

 This sketch provides a default example how to use BLE with:
  - X_NUCLEO_IDB05A1 (BlueNRG-MS expansion board) on top of an STM32 Nucleo board

 For the Sensor Service sketch, 3 services are started : Acc, Environnemental and Time.
 For testing the sketch, you can download on the playstore the "ST BLE Sensor"
 application provided by STMICROELECTRONICS.
 Launch the application and enable Bluetooth on your smartphone. Connect it to
 the BLueNRG device. You will see all the services, you can click on each one.

 You can choose the bluetooth MAC address of the device by configuring SERVER_BDADDR.

 Accelerometer values are updated on user action (press user button).
 Environnemental values (Temperature, humidity and pressure) are updated each seconds.
 Each minute a notification is sent to the user and seconds can be read.

 */
#include "SensorDemoService.h"
#include "SPI.h"

#include <Wire.h>

// Sensors
#include <HTS221Sensor.h>
#include <LPS25HBSensor.h>
#include <LSM6DS3Sensor.h>
#include <LIS3MDLSensor.h>
#include <vl53l0x_class.h>
#include <SparkFun_VEML6075_Arduino_Library.h>
#include <APDS9250.h>

#define PIN_SENSIBLE_LED_GRN    (13)
#define PIN_SENSIBLE_LED_RED    (PC6)

#define PIN_BLE_BTN             (PC13)

#define PIN_BLE_SPI_MOSI   (11)
#define PIN_BLE_SPI_MISO   (12)
#define PIN_BLE_SPI_SCK    (3)

#define PIN_BLE_SPI_nCS    (A1)
#define PIN_BLE_SPI_RESET  (7)
#define PIN_BLE_SPI_IRQ    (A0)

#define PIN_BLE_I2C2_SCL   (PB8)
#define PIN_BLE_I2C2_SDA   (PB9)

#define PIN_BLE_XSHUT      (PC5)

#define PIN_BLE_LED    (0xFF)

// I2C wire
TwoWire dev_i2c(PIN_BLE_I2C2_SDA, PIN_BLE_I2C2_SCL);

// Pointers for sensors objects
HTS221Sensor   HumTemp(&dev_i2c);
LPS25HBSensor  PressTemp(&dev_i2c);
LSM6DS3Sensor AccGyr(&dev_i2c, LSM6DS3_ACC_GYRO_I2C_ADDRESS_LOW);
LIS3MDLSensor Magneto(&dev_i2c);
VL53L0X Prox(&dev_i2c, PIN_BLE_XSHUT);
VEML6075 uvLight;
APDS9250 irLight;

//
typedef struct sensor_data {
  uint32_t r = 0;
  uint32_t g = 0;
  uint32_t b = 0;
  uint32_t ir = 0;
  uint32_t als = 0;
} sensor_data_t;

// Variables for sensors data
float humidity, temperature;
float pressure, temperatureP;
int32_t accelerometer[3];
int32_t gyroscope[3];
int32_t magnetometer[3];
uint32_t distance;
float ultraviolet, ultravioletA, ultravioletB;
sensor_data_t infrared;

bool found_APDS9250 = false;



// Variables for update periods calculation
uint32_t prevUpdMsec = 0;
uint32_t prevLedMsec = 0;
uint32_t envSensorsSkipCnt = 0;
uint32_t printSkipCnt = 0;

// Configure BTLE_SPI
SPIClass BTLE_SPI(PIN_BLE_SPI_MOSI, PIN_BLE_SPI_MISO, PIN_BLE_SPI_SCK);

// Configure BTLE pins
SPBTLERFClass BTLE(&BTLE_SPI, PIN_BLE_SPI_nCS, PIN_BLE_SPI_IRQ, PIN_BLE_SPI_RESET, PIN_BLE_LED);

int counter = 0;

void setup() {
  int status;

  Serial.begin(115200);

  /* Configure the User Leds in GPIO Mode */
  pinMode(PIN_SENSIBLE_LED_GRN, OUTPUT);
  pinMode(PIN_SENSIBLE_LED_RED, OUTPUT);
  //digitalWrite(PIN_SENSIBLE_LED_RED, HIGH);

  /* Configure the User Button in GPIO Mode */
  pinMode(PIN_BLE_BTN, INPUT);

  if(BTLE.begin() == SPBTLERF_ERROR)
  {
    Serial.println("Bluetooth module configuration error!");
    while(1);
  }

  if(SensorDemo.begin())
  {
    Serial.println("Sensor service configuration error!");
    while(1);
  }

  // Initialize I2C bus.
  dev_i2c.begin();
  Serial.println("Init I2C");

  // Initlialize components.
  HumTemp.begin();
  HumTemp.Enable();
  Serial.println("Init Hum&Temp");

  PressTemp.begin();
  PressTemp.Enable();
  Serial.println("Init Pressure&Temp");

  AccGyr.begin();
  AccGyr.Enable_X();
  AccGyr.Enable_G();
  Serial.println("Init Acc&Gyr");

  Magneto.begin();
  Magneto.Enable();
  Serial.println("Init Magneto");

  Prox.begin();
  Prox.VL53L0X_Off();
  status = Prox.InitSensor(0x10);
  if(status)
  {
    Serial.println("Init vl53l0x failed...");
  }
  else Serial.println("Init Proximity");

  if (uvLight.begin(dev_i2c) != VEML6075_ERROR_SUCCESS)
  {
    Serial.println("Init veml6075 failed...");
  }
  else Serial.println("Init UV Light");

  if (!irLight.begin(dev_i2c)) {
    Serial.println("Init apds9250 failed...");
  } else {
    found_APDS9250 = true;
    Serial.println("Init IR` Light");
    Serial.print(F("Resolution ")); Serial.println((uint8_t)irLight.getResolution(), BIN);
    Serial.print(F("Meas Rate ")); Serial.println((uint8_t)irLight.getMeasRate(), BIN);
    Serial.print(F("Gain ")); Serial.println((uint8_t)irLight.getGain(), BIN);

//    irLight.setModeALS();
    irLight.setModeRGB();
    delay(500);
  }
}

void loop() {
  BTLE.update();
  Led_Blink();

  User_Process();

  if(SensorDemo.connected == TRUE)
  {
    // nothing
    
  }
  else
  {
    // Keep the Bluetooth module in discoverable mode
    SensorDemo.setConnectable();
  }
}

/**
 * @brief  Process user input (i.e. pressing the USER button on Nucleo board)
 *         and send the updated acceleration data to the remote client.
 *
 * @param  None
 * @retval None
 */
void User_Process(void)
{
  int status;

  if((millis() - prevUpdMsec) >= 100)
  {
    prevUpdMsec = millis();
    // Update environment data every N cycles
    if(++envSensorsSkipCnt >= 10)
    {
      envSensorsSkipCnt = 0;
      // Read humidity and temperature.
      HumTemp.GetHumidity(&humidity);
      HumTemp.GetTemperature(&temperature);

      // Read pressure and temperature.
      PressTemp.GetPressure(&pressure);
      PressTemp.GetTemperature(&temperatureP);

      // Read distance
      status = Prox.GetDistance(&distance);

      if (status == VL53L0X_ERROR_NONE)
      {
        // Output data.
//        char report[64];
//        snprintf(report, sizeof(report), "| Distance [mm]: %ld |", distance);
//        Serial.println(report);
      }

      // Read ultraviolet
      ultravioletA = uvLight.uva();
      ultravioletB = uvLight.uvb();
      ultraviolet = uvLight.index();

      // Read infrared
//      infrared.als = irLight.getRawALSData();
//      infrared.ir = irLight.getRawIRData();

      infrared.r = irLight.getRawRedData();
      infrared.g = irLight.getRawGreenData();
      infrared.b = irLight.getRawBlueData();


      SensorDemo.BlueMS_Environmental_Update((int32_t)(pressure *100), (int16_t)((temperature + temperatureP) * 5));
    }
    
    // Read accelerometer and gyroscope
    AccGyr.Get_X_Axes(accelerometer);
    AccGyr.Get_G_Axes(gyroscope);
 
    // Read magnetometer.
    Magneto.GetAxes(magnetometer);

    if(SensorDemo.connected == TRUE)
    {
      SensorDemo.Set_Motion_Values(accelerometer, gyroscope, magnetometer);
      SensorDemo.AccGyroMag_Update();
      SensorDemo.Quat_Update();
    }
    
    if(++printSkipCnt >= 10)
    {
      printSkipCnt = 0;
      // Print sensors values to serial port
      Print_Sensors();
    }
  }
}


/*Callback function*/
void SensorDemo_HCI_Event_CB(void *pckt)
{
   hci_uart_pckt *hci_pckt =(hci_uart_pckt*) pckt;
   hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

   Serial.print("In Callback: ");

   if(hci_pckt->type != HCI_EVENT_PKT)
   {
      return;
   }
   /*Switch event*/
   switch(event_pckt->evt)
   {

   /*if disconnected*/
   case EVT_DISCONN_COMPLETE:
   {
      SensorDemo.connected = FALSE;
      SensorDemo.set_connectable = TRUE;
      delay(100);
      Serial.print("Disconnected\n");
   }
   break;
   case EVT_LE_META_EVENT:
   {
      evt_le_meta_event *evt = (evt_le_meta_event *)event_pckt->data;

      switch(evt->subevent)
      {
      /*if connected*/
      case EVT_LE_CONN_COMPLETE:
      {
         SensorDemo.connected=TRUE;
         delay(100);
         Serial.print("Connected\n");
      }
      break;
      }
   }
   break;
   case EVT_VENDOR:
   {
      evt_blue_aci *blue_evt = (evt_blue_aci *)event_pckt->data;
      switch(blue_evt->ecode){

      case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
          evt_gatt_read_permit_req *pr = (evt_gatt_read_permit_req *)blue_evt->data;
          SensorDemo.Read_Request_CB(pr->attr_handle);
        }
        break;
      }
   }
   break;
   default:
      Serial.print("Error\n");
      break;
   }
}

void Print_Sensors()
{
  // Print HTS221 data
  Serial.println("========================");
  Serial.print("Hum[%]: ");
  Serial.println(humidity, 2);
  Serial.print("Temp[C] (HTS221): ");
  Serial.println(temperature, 2);

  // Print LPS25 data
  Serial.println("------------------------");
  Serial.print("Pres[hPa]: ");
  Serial.println(pressure, 2);
  Serial.print("TempP[C] (LPS25): ");
  Serial.println(temperatureP, 2);

  // Print distance
  Serial.println("------------------------");
  Serial.print("Distance [mm]: ");
  Serial.println(distance);

  // Print LIS3MDL data
  Serial.println("------------------------");
  Serial.print("Mag[mGauss]: ");
  Serial.print(magnetometer[0]);
  Serial.print(" ");
  Serial.print(magnetometer[1]);
  Serial.print(" ");
  Serial.println(magnetometer[2]);
  
  // Print accelerometer data
  Serial.println("------------------------");
  Serial.println("Acc");
  Serial.print("X: ");
  Serial.println(accelerometer[0]);
  Serial.print("Y: ");
  Serial.println(accelerometer[1]);
  Serial.print("Z: ");
  Serial.println(accelerometer[2]);
  // Print gyroscope data
  Serial.println("------------------------");
  Serial.println("Gyr");
  Serial.print("X: ");
  Serial.println(gyroscope[0]);
  Serial.print("Y: ");
  Serial.println(gyroscope[1]);
  Serial.print("Z: ");
  Serial.println(gyroscope[2]);

  // Print UV
  Serial.println("------------------------");
  Serial.print("UVa: ");
  Serial.println(ultravioletA);
  Serial.print("UVb: ");
  Serial.println(ultravioletB);
  Serial.print("uv Light: ");
  Serial.println(ultraviolet);

  // Print IR
  Serial.println("------------------------");
//  Serial.print(F("ALS   ")); Serial.println(infrared.als);
//  Serial.print(F("IR    ")); Serial.println(infrared.ir);

  Serial.print(F("Red   ")); Serial.println(infrared.r, HEX);
  Serial.print(F("Green ")); Serial.println(infrared.g, HEX);
  Serial.print(F("Blue  ")); Serial.println(infrared.b, HEX);
}

void Led_Blink()
{
 
  // Blink LED depending on the connection state
  if(SensorDemo.connected == TRUE)
  {
    if(millis() - prevLedMsec > 1000)
    {
      prevLedMsec = millis();
      digitalWrite(PIN_SENSIBLE_LED_GRN, HIGH);
      delay(10);
      digitalWrite(PIN_SENSIBLE_LED_GRN, LOW);
    }
  }
  else
  {    
    if(millis() - prevLedMsec > 300)
    {
      prevLedMsec = millis();
      digitalWrite(PIN_SENSIBLE_LED_GRN, HIGH);
      delay(10);
      digitalWrite(PIN_SENSIBLE_LED_GRN, LOW);
    }
  }
}
