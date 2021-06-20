#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include <MPU9250.h>
#include "GPS_Air530.h"
#include <OneWire.h>
#include <DallasTemperature.h>


void VextON(void)
{
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
}

void VextOFF(void) //Vext default OFF
{
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, HIGH);
}

int state = 0;

//-------------------------BUTTON START-------------------------
#define INT_GPIO GPIO1
#define em_GPIO GPIO6
#define timetillsleep 20000
static TimerEvent_t sleep;
uint8_t lowpower=0;
int flag = 0;
int emergent = 0;
int emergency_count = 0;
void onSleep()
{
  Serial.printf("Going into lowpower mode. Press user key to wake up\r\n");
  lowpower=1;
}
void onWakeUp()
{
  delay(10);
  if(digitalRead(INT_GPIO) == 0)
  {
    flag++;
    Serial.printf("Woke up by GPIO, %d ms later into lowpower mode.\r\n",timetillsleep);
    lowpower=0;
    //timetillsleep ms later into lowpower mode;
  }
  if(flag > 4)
  {
    state = 1;
    flag = 0;
    TimerSetValue( &sleep, timetillsleep );
    TimerStart( &sleep );
  }
  else
  {
    state = 0;
  }
}

void emergency()
{
  if(digitalRead(em_GPIO) == 0)
  {
    emergency_count++;
    Serial.print("em_GPIO : ");
    Serial.println(emergency_count);
  }
  if(emergency_count > 4)
  {
    emergent = 1;
  }
  if(emergency_count > 8)
  {
    emergent = 0;
    emergency_count = 0;
  }
}
//-------------------------BUTTON END-------------------------

//-------------------------TEMPERATURE START-------------------------
#define ONE_WIRE_BUS GPIO10
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

int temperature = 0;

void getTemperature()
{
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  Serial.print("Temperature for the device 1 (index 0) is: ");
  Serial.println(sensors.getTempCByIndex(0));
  temperature = sensors.getTempCByIndex(0);
}
//-------------------------TEMPERATURE START-------------------------


//-------------------------PULSE START-------------------------
int PulseSensorPurplePin = ADC2;
int Signal;
int Threshold = 1680;
int pulse = 0;

void getPulse()
{
  int pulse_start = millis();
  pulse = 0;
  while(pulse_start + 5000 > millis())
  {
    Signal = analogRead(PulseSensorPurplePin);
    if(Signal > Threshold)
    {
      pulse++;
    }
    delay(10);
  }
  Serial.print("pulse : ");
  Serial.println(pulse);
}
//-------------------------PULSE END-------------------------

//-------------------------BUZZER START-------------------------
uint16_t a = 0;
void buzzing(int button)
{
  Serial.println("buzzing");
    int buzzCount = 0;
    while (true)
    {
        buzzCount++;
        analogWrite(PWM2, a);
        delay(20);
        a += 255;
        if ((buzzCount > 30) && (button = 0))
        {
            analogWrite(PWM2, LOW);
            return;
        }
        if ((buzzCount > 60) && (button = 1))
        {
            analogWrite(PWM2, LOW);
            return;
        }
        if ((buzzCount > 90) && (button = 2))
        {
            analogWrite(PWM2, LOW);
            return;
        }
    }
}
//-------------------------BUZZER END-------------------------

//-------------------------GPS START-------------------------
void displayInfo()
{
  Serial.print("Date/Time: ");
  if (Air530.date.isValid())
  {
    Serial.printf("%d/%02d/%02d",Air530.date.year(),Air530.date.day(),Air530.date.month());
  }
  else
  {
    Serial.print("INVALID");
  }

  if (Air530.time.isValid())
  {
    Serial.printf(" %02d:%02d:%02d.%02d",Air530.time.hour(),Air530.time.minute(),Air530.time.second(),Air530.time.centisecond());
  }
  else
  {
    Serial.print(" INVALID");
  }
  Serial.println();
  
  Serial.print("LAT: ");
  Serial.print(Air530.location.lat(),6);
  Serial.print(", LON: ");
  Serial.print(Air530.location.lng(),6);
  Serial.print(", ALT: ");
  Serial.print(Air530.altitude.meters());

  Serial.println(); 
  
  Serial.print("SATS: ");
  Serial.print(Air530.satellites.value());
  Serial.print(", HDOP: ");
  Serial.print(Air530.hdop.hdop());
  Serial.print(", AGE: ");
  Serial.print(Air530.location.age());
  Serial.print(", COURSE: ");
  Serial.print(Air530.course.deg());
  Serial.print(", SPEED: ");
  Serial.println(Air530.speed.kmph());
  Serial.println();
}

void makeGPSDATA()
{
  uint32_t starttime = millis();
  while( (millis()-starttime) < 1000 )
  {
    while (Air530.available() > 0)
    {
      Air530.encode(Air530.read());
    }
  }
  displayInfo();
  if (millis() > 5000 && Air530.charsProcessed() < 10)
  {
    Serial.println("No GPS detected: check wiring.");
    while(true);
  }
}
//-------------------------GPS END-------------------------


//-------------------------MPU9250 START-------------------------
MPU9250 mySensor;
//-------------------------MPU9250 END-------------------------


//-------------------------LORA START-------------------------
/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */

/* OTAA para*/
uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 };
uint8_t appKey[] = { 0x0F, 0xD3, 0x6F, 0xE4, 0x51, 0xFF, 0x12, 0x32, 0x41, 0xD5, 0xDA, 0x47, 0x54, 0x0C, 0x0A, 0x96 };
/* ABP para*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;

/*LoraWan channelsmask*/
uint16_t userChannelsMask[6]={ 0x0003,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 10000;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
  VextON();
  delay(50);
  if(state == 1)
  {
    buzzing(2);
  }
  else if (emergent == 1)
  {
    buzzing(3);
  }
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
  *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
  *if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
  *if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
  *for example, if use REGION_CN470, 
  *the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */
  float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
  Wire.begin();
  mySensor.setWire(&Wire);
 
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();


  makeGPSDATA();
  float lat, lon, alt, course, speed, hdop, sats;
  lat = Air530.location.lat();
  lon = Air530.location.lng();
  alt = Air530.altitude.meters();
  course = Air530.course.deg();
  speed = Air530.speed.kmph();
  sats = Air530.satellites.value();
  hdop = Air530.hdop.hdop();

  getTemperature();
  getPulse();

  
  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
  

  mySensor.accelUpdate();
  aX = mySensor.accelX();
  aY = mySensor.accelY();
  aZ = mySensor.accelZ();
  aSqrt = mySensor.accelSqrt();
  Serial.print("aX:");
  Serial.println(aX);
  Serial.print("aY:");
  Serial.println(aY);
  Serial.print("aZ:");
  Serial.println(aZ);
  Serial.print("aSqrt:");
  Serial.println(aSqrt);
  

  mySensor.gyroUpdate();
  gX = mySensor.gyroX();
  gY = mySensor.gyroY();
  gZ = mySensor.gyroZ();
  Serial.print("gX:");
  Serial.println(gX);
  Serial.print("gY:");
  Serial.println(gY);
  Serial.print("gZ:");
  Serial.println(gZ);
  

  mySensor.magUpdate();
  mX = mySensor.magX();
  mY = mySensor.magY();
  mZ = mySensor.magZ();
  mDirection = mySensor.magHorizDirection();
  Serial.print("mX:");
  Serial.println(mX);
  Serial.print("mY:");
  Serial.println(mY);
  Serial.print("mZ:");
  Serial.println(mZ);
  Serial.print("mDirection:");
  Serial.println(mDirection);
  
  Wire.end();
  uint16_t batteryVoltage = getBatteryVoltage();
  unsigned char *puc;

  puc = (unsigned char *)(&aX);
  appDataSize = 0;
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  puc = (unsigned char *)(&aY);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  puc = (unsigned char *)(&aZ);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  puc = (unsigned char *)(&lat);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  puc = (unsigned char *)(&lon);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  puc = (unsigned char *)(&pulse);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  puc = (unsigned char *)(&temperature);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  puc = (unsigned char *)(&state);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  puc = (unsigned char *)(&emergent);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  int batterylev = batteryVoltage;
  puc = (unsigned char *)(&batterylev);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  

  Serial.print("BatteryVoltage:");
  Serial.println(batterylev);
  VextOFF();
}
//-------------------------LORA END-------------------------

void setup() {
  digitalWrite(GPIO11, HIGH);
  
  
  //system start
  Serial.begin(115200);
  Serial.println("SYSTEM INIT");
  //system end

  //button start
  pinMode(INT_GPIO,INPUT);
  attachInterrupt(INT_GPIO,onWakeUp,FALLING);
  pinMode(em_GPIO,INPUT);
  attachInterrupt(em_GPIO,emergency,FALLING);
  TimerInit( &sleep, onSleep );
  //button end
  
  //pulse start
  //NONE
  //pulse end
  
  //temperature start
  sensors.begin();
  //temperature end
  
  //LoRa start
#if(AT_SUPPORT) 
enableAt();
#endif
  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();
  //LoRa end
  
  //GPS start
  Air530.begin();
  //GPS end

}

int SEND_COUNT = 0;

void loop()
{
  if(lowpower){
    Serial.print("lowpowermode :");
    Serial.println(lowpower);
    lowPowerHandler();
  }
  else
  {
      if(SEND_COUNT > 4)
  {
    buzzing(1);
    pinMode(GPIO11,OUTPUT);
    digitalWrite(GPIO11, LOW);
    deviceState = DEVICE_STATE_JOIN;
  }
  switch( deviceState )
  {
    case DEVICE_STATE_INIT:
    {
#if(LORAWAN_DEVEUI_AUTO)
      LoRaWAN.generateDeveuiByChipID();
#endif
#if(AT_SUPPORT)
      getDevParam();
#endif
      printDevParam();
      LoRaWAN.init(loraWanClass,loraWanRegion);
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      Serial.print("State send count : ");
      Serial.println(++SEND_COUNT);
      prepareTxFrame( appPort );
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      // Schedule next packet transmission
      txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      LoRaWAN.sleep();
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
  }

}
