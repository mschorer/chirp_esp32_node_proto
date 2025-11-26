/**
 * 
 * FOR THIS EXAMPLE TO WORK, YOU MUST INSTALL THE "LoRaWAN_ESP32" LIBRARY USING
 * THE LIBRARY MANAGER IN THE ARDUINO IDE.
 * 
 * This code will send a two-byte LoRaWAN message every 15 minutes. The first
 * byte is a simple 8-bit counter, the second is the ESP32 chip temperature
 * directly after waking up from its 15 minute sleep in degrees celsius + 100.
 *
 * If your NVS partition does not have stored TTN / LoRaWAN provisioning
 * information in it yet, you will be prompted for them on the serial port and
 * they will be stored for subsequent use.
 *
 * See https://github.com/ropg/LoRaWAN_ESP32
*/

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <SensorData.h>

#define LL_OFF  0
#define LL_LOW  1
#define LL_MED  2
#define LL_HGH  2

#define LOGLEVEL  LL_LOW

//#define LOGLEVEL  LL_LOW

#define RADIOLIB_DEBUG_PROTOCOL 1
#define HELTEC_V3FIX
//#define ADC_SCALE 0.00403532794741887
//#define ADC_SCALE ( 1/238.7)
#define ADC_SCALE 0.00433

#define LED_OFF 0
#define LED_LOW 3
#define LED_MID 15
#define LED_BRIGHT  50

#define FALSE 0
#define TRUE 1

#define XSHUT_PIN 47

#include <heltec_unofficial.h>
#include <LoRaWAN_ESP32.h>

#include <stdarg.h>
#include <stdio.h>

//---- VL530x
//#define TOF_L0X

#ifdef TOF_L0X
  #include <Adafruit_VL53L0X.h>

  Adafruit_VL53L0X tof = Adafruit_VL53L0X();
#else
  //#include <Adafruit_VL53L1X.h>
  #include "SparkFun_VL53L1X.h"

  //Adafruit_VL53L1X tof = Adafruit_VL53L1X();
  SFEVL53L1X tof;
#endif

#include "SensorData.h"

bool hasTOF = false;

//---- OneWire ----
// data cable connected to D4 pin
#define ONE_WIRE_BUS GPIO_NUM_3

union busAddress {
  byte raw[8];
  long data[2];
};

byte sensors = 0;
busAddress sensorIDs[ 32];

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

//---- BME280 ----

#define SEALEVELPRESSURE_HPA (1013.25)

#define BME_I2C_ADDR 0x76
Adafruit_BME280 bme; // I2C
bool bmeAvailable = FALSE;

//---- LoRaWan ----

// Pause between sends in seconds, so this is every 15 minutes. (Delay will be
// longer if regulatory or TTN Fair Use Policy requires it.)
#define MINIMUM_DELAY 300

#define RELAX_DELAY 120
#define RELAX_FB    3

#define ALERT_DELAY 60
#define ALERT_FB    3

#define EMERG_DELAY 30
#define EMERG_FB    10

#define AUTO_FALLBACK 16

#define LORA_DUTY_CYCLE 0     // for legal limit: 0, forTTN: 1250

RTC_DATA_ATTR uint16_t loraDelay = MINIMUM_DELAY;
RTC_DATA_ATTR uint16_t loraDlyFb = 0;

// you can also retrieve additional information about an uplink or 
// downlink by passing a reference to LoRaWANEvent_t structure
LoRaWANEvent_t uplinkDetails;
LoRaWANEvent_t downlinkDetails;

uint8_t fPort = 1;

//RTC_DATA_ATTR uint8_t count = 0;

LoRaWANNode* node;

//----------------------------------------------

#define DOWN_SIZE 32

SensorData down( DOWN_SIZE);
SensorData up( 128);

LoraNode *localNode;

  //LoraGps gps;
LoraBME280 *localBME;
LoraDS18B20 *localOneWire;
LoraToF *localToF;

LoraToF *remoteToF = 0;

//---- RTSC persistent storage ----

struct dataMem {
  uint32_t average;
  uint16_t history[8];
  uint8_t count;
  uint8_t index;
  uint16_t remote;
};

#define RTC_BLOCK_OFFSET 64
struct rtcMem {
  struct dataMem data;
  long crc32;
};

RTC_DATA_ATTR rtcMem history;

void restoreData( rtcMem *pData) {
  uint32_t crcNow = calculateCRC32( (uint8_t *)&pData->data, sizeof(pData->data));

    Serial.printf( "restore [%x] [%x]\n", crcNow, pData->crc32);

   if ( pData->crc32 != crcNow) {
    initData( &pData->data);
   }
}

void persistData( rtcMem *pData) {
  pData->crc32 = calculateCRC32( (uint8_t *)&pData->data, sizeof(pData->data));
  Serial.printf( "backup [%x]\n", pData->crc32);
}

void initData( dataMem* history) {
  // preset data here
  history->average = 0;
  for( int i=0; i < 8; i++) {
  history->history[i] = 0;
  }
  history->index = 0;
  history->count = 0;
  history->remote = 0;
}

void addTof( dataMem* history, uint16_t range) {
  history->average -= history->history[ history->index];
  history->history[ history->index] = range;
  history->average += range;

  history->index++;
  if ( history->index > 7)
    history->index = 0;
  if ( history->count < 8)
    history->count++;

  Serial.printf( "hist [%i-%i] %i\n", history->count, history->index, history->average);
}

uint16_t getAveragedTof( dataMem* history) {
  return history->average / history->count;
}

//---- Code ----

void sprintAt( int16_t x, int16_t y, char *fmt, ...) {
  char gstring[48];
  va_list args;

  va_start(args, fmt);

  vsprintf( gstring, fmt, args);
  display.drawString( x, y, gstring);
}

void drawVBar( uint8_t x,  uint8_t y, uint8_t w, uint8_t h, uint8_t headroom) {
  display.drawRect( x, y, w, headroom);
  display.fillRect( x, y+headroom, w, h-headroom);
}

void drawHBar( uint8_t x,  uint8_t y, uint8_t w, uint8_t h, uint8_t fill) {
  display.fillRect( x, y, fill, h);
  display.drawRect( x+fill, y, w-fill, h);
}

void drawNode( uint8_t x, uint8_t y, LoraNode *node, char* name) {
  uint8_t batLevel = 0;
  char status = '.';

  if ( node) {
    if ( node->vbat > 0) {
      batLevel = min( 50, (node->vbat -325) /2);
      sprintAt( 40, y, "%.2fV", (float)node->vbat / 100);
    } else {
        sprintAt( 40, y, "-.--V" );
    }
    switch( node->meta & STS_MMASK) {
      case STS_EMERG: status = '*'; break;
      case STS_ALERT: status = '+'; break;
      case STS_RELAX: status = '-'; break;
    }
    sprintAt( 30, y, "%c", status);
  }
  drawHBar( 78, y+1, 50, 5, batLevel);
  sprintAt( 0, y, name);
}

//---- Code -----------------------------------------------------------------

void setup() {
  byte i;
  byte present = 0;
  byte type_s = 0;
  byte data[12];
  byte devCount = 0;
  byte sensor = 0;
  char sensorId[16];
  char sensorType[8];

  //---- start ----
  heltec_setup();
  heltec_led(LED_LOW);

  restoreData( &history);

  //display.init();
  display.setFont(ArialMT_Plain_10);
  display.clear();
  display.drawHorizontalLine( 0, 39, 64);
  display.drawVerticalLine( 64, 39, 22);
  //display.drawHorizontalLine( 64, 63, 64);
  //display.display();
  
  // add node
  localNode = (LoraNode *) up.addSensor( SensorData::SensorType::NODE, sizeof( LoraNode));
  //LoraGps gps;
  //localBME = (LoraBME280 *) up.addSensor( SensorData::SensorType::BME280, sizeof( LoraBME280));
  //localOneWire = (LoraDS18B20 *) up.addSensor( SensorData::SensorType::DS18B20, sizeof( LoraDS18B20));
  //localToF = (LoraToF *) up.addSensor( SensorData::SensorType::TOF, sizeof( LoraToF));

  // Obtain directly after deep sleep
  // May or may not reflect room temperature, sort of. 
  float temp = heltec_temperature();
  //Serial.printf("Temperature: %.1f °C\n", temp);

  analogReadResolution(12);
  
  pinMode(VBAT_CTRL, OUTPUT);
  digitalWrite(VBAT_CTRL, HIGH);
  delay(5);
  int vint = analogRead(VBAT_ADC);
  float vbat = vint * ADC_SCALE;
  // pulled up, no need to drive it
  pinMode(VBAT_CTRL, INPUT);

  // 0 = external power source
  // 1 = lowest (empty battery)
  // 254 = highest (full battery)
  // 255 = unable to measure
  uint8_t battLevel = 0;
  if (vint < 980)
    battLevel = ( vint - 470) / 2;

  Serial.printf( "Bat: %i\n", vint);
  int vperc = heltec_battery_percent(vbat);
  //Serial.printf("%i%%\n", vperc);
  Serial.printf("%.1f°C / %.2fV / %3i%% / %02x\n", temp, vbat, vperc, battLevel);

  // prepare data -----------------------------

  localNode->meta = STS_DCARE;
  localNode->cputemp = temp * 10;
  localNode->vbat = vbat * 100;

  Serial.printf( "esp temp/vbat[ %i %i ]\n", localNode->cputemp, localNode->vbat);
  drawNode( 0, 0, localNode, "Node");

  /*
  for( int i=0; i < 4; i++) {
    pinMode( gpioMap[ i], OUTPUT);
    digitalWrite( gpioMap[ i], HIGH);
#ifdef LOGLEVEL
    Serial.print( "preset gpio [");
    Serial.print( gpioMap[ i]);
    Serial.print( "] = [");
    Serial.print( HIGH);
    Serial.println( "]");
#endif
  }
  */

  //---- I2C bus ----

  Wire1.begin( SDA, SCL, 400000);

  //---- DS18B20 on OneWire ----

  DS18B20.begin();

  //Loop through all DS1820
  sensors = 0;
  while ( oneWire.search( sensorIDs[sensors].raw)) {
    //Topic is built from a static String plus the ID of the DS18B20
    if ( crcCheck( sensorIDs[sensors].raw, 7)) {
      Serial.printf( "crc %i", sensors);
      continue;
    }

    // the first ROM byte indicates which chip
    switch (sensorIDs[sensors].raw[0]) {
      case 0x10:
        strcpy( sensorType, "DS18S20");  // or old DS1820
        type_s = 1;
        break;
      case 0x28:
        strcpy( sensorType, "DS18B20");
        type_s = 0;
        break;
      case 0x22:
        strcpy( sensorType, "DS1822");
        type_s = 0;
        break;
      default:
        strcpy( sensorType, "-------");
        return;
    }

    oneWire.reset();
    oneWire.select( sensorIDs[sensors].raw);
    oneWire.write( 0x44, 1);        // start conversion, with parasite power on at the end

    sensorId[0] = 0;
    char hex[32];
    for ( byte x = 0; x < 8; x++) {
      sprintf( hex, "%02x", sensorIDs[sensors].raw[x]);
      strcat( sensorId, hex);
    }
#ifdef LOGLEVEL
    Serial.print( "found: [");
    Serial.print( sensorType);
    Serial.print( "] @ [");
    Serial.print( sensorId);
    Serial.println( "]");
#endif
    // we might do a oneWire.depower() here, but the reset will take care of it.

    present = oneWire.reset();

    sensors++;
  }
  oneWire.reset_search();
#ifdef LOGLEVEL
  Serial.print( "found #[");
  Serial.print( sensors);
  Serial.println( "] oneWire sensors");
#endif

  //---- ToF sensor ----

#ifdef TOF_L0X
  if (tof.begin( VL53L0X_I2C_ADDR, true, &Wire1 , Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE)) {
    hasTOF = true;
  } else {
    Serial.println(F("Failed to boot VL53L0X"));
  }
#else
  if ( tof.begin( Wire1) == 0) {
    hasTOF = true;

    Serial.println(F("VL53L1X sensor OK!"));

    Serial.print(F("Sensor ID: 0x"));
    Serial.println(tof.getSensorID(), HEX);
  } else {
    Serial.println(F("Error on init of VL sensor: "));
  }
#endif
  //localToF->dist = -2;

  //---- BME ----

  if ( ! bme.begin( BME_I2C_ADDR, &Wire1)) {
#ifdef LOGLEVEL
    Serial.println( "BME280 not found, skipping ...");
#endif
    bmeAvailable = FALSE;
  } else {
    bmeAvailable = TRUE;
    // weather monitoring
#ifdef LOGLEVEL
    Serial.println("BME 280 found [forced, 1x-oversampling,filter off]");
#endif
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
  }

  if ( bmeAvailable)
    bme.takeForcedMeasurement();

  //------------------------------------------

  if ( sensors > 1) {
    sensors = 1;
  }

  uint16_t owDelay = 820;

  //---- ToF ----
  if ( hasTOF) {

    localToF = (LoraToF *) up.addSensor( SensorData::SensorType::TOF, sizeof( LoraToF));

#ifdef TOF_L0X
    
    VL53L0X_RangingMeasurementData_t measure;
    tof.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      localToF->dist = measure.RangeMilliMeter;
      Serial.print("Distance (mm): ");
      Serial.println( localToF->dist);
    } else {
      Serial.println(" out of range ");
      localToF->dist = -1;
    }

    display.setFont(ArialMT_Plain_16);
    sprintAt( 80, 45, "%4.d", localToF->dist);
#else
    tof.setDistanceModeLong();

    //tof.startOneshotRanging();
    tof.startRanging();

    uint32_t sum = 0;
    uint16_t sample = 0;
    uint16_t msmtsTaken = 0;

    localToF->dist = -1;
    while( owDelay > 0) {
      if ( tof.checkForDataReady()) {
        sample = tof.getDistance();
        tof.clearInterrupt();

        sum += (uint32_t) sample;
        msmtsTaken++;

        Serial.print(F("Range: "));
        Serial.print( sample);
        Serial.print(F(" @"));
        Serial.println( owDelay);

        if (msmtsTaken > 7) break;
      }
      delay(1);
      owDelay--;
    };
    tof.stopRanging();

    if ( msmtsTaken) {
      localToF->dist = sum / msmtsTaken;

      addTof( &history.data, localToF->dist);
    } else {
      localToF->dist = -1;
    }

    int signalRate = tof.getSignalRate();
    Serial.print("Signal rate: ");
    Serial.println(signalRate);

    byte rangeStatus = tof.getRangeStatus();
    Serial.print("Range Status: ");
      //Make it human readable
    switch (rangeStatus) {
      case 0:
        Serial.print("Good");
        break;
      case 1:
        Serial.print("Sigma fail");
        break;
      case 2:
        Serial.print("Signal fail");
        break;
      case 7:
        Serial.print("Wrapped target fail");
        break;
      default:
        Serial.print("Unknown: ");
        Serial.print(rangeStatus);
        break;
    }
    Serial.println();

    uint16_t hist = getAveragedTof( &history.data);
    Serial.printf( "Range %i [%i] [%i]\n", localToF->dist, hist, localToF->dist - hist);

    display.setFont(ArialMT_Plain_16);
    sprintAt( 88, 48, "% 4d", localToF->dist);

#endif
    display.setFont(ArialMT_Plain_10);
    sprintAt( 0, 38, "% 4d", history.data.remote);
  } else {
    Serial.println("skipping ToF");

    display.setFont(ArialMT_Plain_10);
    sprintAt( 5, 38, "----");

    display.setFont(ArialMT_Plain_16);
    sprintAt( 88, 48, "----");
  }

  sensor = 0;
  if ( sensors) {    
    localOneWire = (LoraDS18B20 *) up.addSensor( SensorData::SensorType::DS18B20, sizeof( LoraDS18B20));

    // delay further for the remainder of time we didn't use up in waiting for the ranging sensor
    if ( owDelay > 0) {
      Serial.printf( "final delay for DS18B20 %i\n", owDelay);
      delay( owDelay);
    } else {
      Serial.println( "delay for DS18B20 already expired.");
    }

    while ( sensor < sensors) {
      oneWire.select( sensorIDs[sensor].raw);
      oneWire.write( 0xBE);         // Read Scratchpad

      /*
        Serial.print("  Data = ");
        Serial.print(present, HEX);
        Serial.print(" ");
      */
      for ( i = 0; i < 9; i++) {           // we need 9 bytes
        data[i] = oneWire.read();
      }
      present = oneWire.reset();

      if ( crcCheck( data, 8)) {
        sensor++;
        continue;
      }

      int16_t raw = (data[1] << 8) | data[0];
      if (type_s) {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
          // "count remain" gives full 12 bit resolution
          raw = (raw & 0xFFF0) + 12 - data[6];
        }
      } else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
      }

      //convert RAW Temperature to celsius
      double temp = raw * 0.0625;

      if ( temp != 85.0) {
        // add temperature
        localOneWire->temp[sensor] = raw;
        Serial.printf( "Sensor %i [%i] [%f]\n", sensor, raw, temp);
      } else {
#ifdef LOGLEVEL
        Serial.println("skip");
#endif
      }

      devCount++;
      sensor++;
    }

    if ( sensor) {
      Serial.printf( "temps[ %i ]\n", localOneWire->temp[0]);

      display.setFont(ArialMT_Plain_10);
      sprintAt( 5, 14, "C    % 2.1f", ((float) localOneWire->temp[0])/10);
    }
  }
  if ( sensor == 0) {
    Serial.println( "no DS18x20");
  
    display.setFont(ArialMT_Plain_10);
    sprintAt( 5, 14, "C    --.-");
  }

  //---- BME ----

  if ( bmeAvailable) {
    float bme_temp = bme.readTemperature() * 10.0;
    float bme_hmd = bme.readHumidity() * 10.0;
    float bme_prs = bme.readPressure() / 10.0;

    Serial.printf( "temp/humid/press [ %f %f %f ]\n", bme_temp, bme_hmd, bme_prs);

    localBME = (LoraBME280 *) up.addSensor( SensorData::SensorType::BME280, sizeof( LoraBME280));

    localBME->temp = (int16_t) bme_temp;
    localBME->hmd = (uint16_t) bme_hmd;
    localBME->prs = (uint16_t) bme_prs;

    display.setFont(ArialMT_Plain_10);
    sprintAt( 5, 25, "C    % 2.1f", (float)localBME->temp/10);
    sprintAt( 72, 14, "hum%%   % 4d", localBME->hmd/10);
    sprintAt( 72, 25, "mbar    % 4d", localBME->prs/10);
  } else {
    Serial.println( "no BME280");

    display.setFont(ArialMT_Plain_10);
    sprintAt( 5, 25, "C    --.-");
    sprintAt( 72, 14, "hum%%    ---");
    sprintAt( 72, 25, "mbar     ----");
  }

  display.display();

  persistData( &history);

  // send (and receive data) ------------------------------------

  // initialize radio
  Serial.print("init ... ");
  int16_t state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.println("error - postpone.");
    goToSleep();
  }

  node = persist.manage(&radio);

  if (!node->isActivated()) {
    Serial.println("RESTORE FAILED!");
    goToSleep();
  }

  Serial.println( "session restored!");
  
  Serial.print("[LoRaWAN] DevAddr: ");
  Serial.println((unsigned long)node->getDevAddr(), HEX);

  Serial.printf( "rssi %.1f snr %.1f frq %.1f\n", radio.getRSSI(), radio.getSNR(), radio.getFrequencyError());
  heltec_led(LED_MID);

  // If we're still here, it means we joined, and we can send something

  // Enable the ADR algorithm (on by default which is preferable)
  node->setADR(true);
  // Set a datarate to start off with
  node->setDatarate(5);
  // Manages uplink intervals to the TTN Fair Use Policy
  node->setDutyCycle(true, LORA_DUTY_CYCLE);   // zero sets max val by law, TTN 30s/24h: [1250]ms/1h);
  // Update dwell time limits - 400ms is the limit for the US
  node->setDwellTime(true, 400);  // zero sets max val by law, 400);

  node->setDeviceStatus(battLevel);

  //==============================================================

  Serial.print( "Sending ... ");

  // Retrieve the last uplink frame counter
  uint32_t fCntUp = node->getFCntUp();
  Serial.println( "FCNTUP "+String( fCntUp));

  // up.printBuffer();

  size_t downlinkSize = DOWN_SIZE;
  
  if(fCntUp == 1) {
    Serial.println(F("and requesting LinkCheck and DeviceTime"));
    node->sendMacCommandReq(RADIOLIB_LORAWAN_MAC_LINK_CHECK);
    node->sendMacCommandReq(RADIOLIB_LORAWAN_MAC_DEVICE_TIME);

    state = node->sendReceive( up.buffer, up.eod, fPort, down.buffer, &downlinkSize, true, &uplinkDetails, &downlinkDetails);
  } else {
    state = node->sendReceive( up.buffer, up.eod, fPort, down.buffer, &downlinkSize, false, &uplinkDetails, &downlinkDetails);
  }
  down.eod = (uint8_t) downlinkSize;

  if(state == RADIOLIB_ERR_NONE) {
    Serial.println("OK.");
    heltec_led(LED_BRIGHT);
  } else if (state > 0) {
  // Check if a downlink was received 
  // (state 0 = no downlink, state 1/2 = downlink in window Rx1/Rx2)
    Serial.println(F("OK. Downlink data!"));
    // Did we get a downlink with data for us
    if(downlinkSize > 0) {
      Serial.println(F("Downlink data: "));
      arrayDump(down.buffer, down.eod);

      handleDownlink( &down);

      if ( remoteToF) {
        Serial.printf( "TOF: %i (%i)\n", remoteToF->dist, history.data.remote);

        display.setFont(ArialMT_Plain_16);
        sprintAt( 24, 48, "%4d", remoteToF->dist);

        history.data.remote = remoteToF->dist;
        persistData( &history);
      }
    } else {
      Serial.println(F("<MAC commands only>"));

      display.setFont(ArialMT_Plain_16);
      sprintAt( 24, 48, "----");
    }

    dumpDownlinkStats( state);

    heltec_led(LED_BRIGHT);
  } else {
    Serial.printf("Error %d\n", state);
    heltec_led(LED_LOW);
  }

  display.display();

  goToSleep();    // Does not return, program starts over next round
}

void loop() {
  heltec_loop();
}

void handleDownlink( SensorData *down) {
  uint8_t index = 0;
  LoraSensor* remoteStatus;

  while( index < down->eod) {
    switch( down->buffer[ index]) {
        case SensorData::SensorType::STATUS:
          remoteStatus = (LoraSensor *) &down->buffer[index];
          index += sizeof( LoraSensor);

          localNode->meta = remoteStatus->meta;
          Serial.printf( "status [%i]\n", localNode->meta);
        break;

        case SensorData::SensorType::ID:
          index += sizeof( LoraID);
          Serial.println( "  id");
        break;

        case SensorData::SensorType::NODE:
          index += sizeof( LoraNode);
          Serial.println( "  node");
        break;

        case SensorData::SensorType::DS18B20:
          index += sizeof( LoraDS18B20);
          Serial.println( "  ds18b20");
        break;

        case SensorData::SensorType::BME280:
          index += sizeof( LoraBME280);
          Serial.println( "  bme280");
        break;

        case SensorData::SensorType::TOF:
          remoteToF = (LoraToF *) &down->buffer[index];
          index += sizeof( LoraToF);
        break;

        case SensorData::SensorType::GPS:
          index += sizeof( LoraGps);
          Serial.println( "  gps");
        break;

        case SensorData::SensorType::FILL:
          index += sizeof( LoraFill);
          Serial.println( "  fill");
        break;

        default:
          Serial.println( "xTra data.");
          index = down->eod;
    }
  };
}

void goToSleep() {
  // allows recall of the session after deepsleep
  persist.saveSession(node);

  // -------------------

  uint8_t meta = localNode->meta & STS_MMASK;
  switch( meta) {
    case STS_EMERG:
      loraDelay = EMERG_DELAY;
      loraDlyFb = EMERG_FB;
      Serial.println( "meta EMERG!");
    break;

    case STS_ALERT:
      loraDelay = ALERT_DELAY;
      loraDlyFb = ALERT_FB;
      Serial.println( "meta ALERT!");
    break;

    case STS_RELAX:
      loraDelay = RELAX_DELAY;
      loraDlyFb = RELAX_FB;
      Serial.println( "meta RELAX!");
    break;

    case STS_DCARE:
    default:
      loraDelay = MINIMUM_DELAY;
      loraDlyFb = 0;
      Serial.println( "meta DCARE!");
  }

  if ( loraDlyFb > 0) {
    loraDlyFb--;
  } else {
    loraDelay = MINIMUM_DELAY;
  }

  // -------------------

  // Calculate minimum duty cycle delay (per FUP & law!)
  uint32_t dutyCycle = node->timeUntilUplink();
  // And then pick it or our MINIMUM_DELAY, whichever is greater
  uint32_t delayMs = max(dutyCycle, (uint32_t)loraDelay * 1000);

  Serial.printf("DeepSleep for [%d][%d@%d]\n", dutyCycle, loraDelay, loraDlyFb);

  delayMs = blinkMode( meta, delayMs);

  // and off to bed we go
  heltec_deep_sleep( delayMs/1000);
}

uint32_t blinkMode( uint8_t meta, uint32_t delayMs) {
  uint8_t loops = 0;
  //Serial.printf( "metamode [%02x]\n", meta);

  while( loops < 64) {
    //Serial.printf("(%d)", loops);

    switch( loops % 8) {
      case 6:
        if ( meta < STS_EMERG) break;
      case 4:
        if ( meta < STS_ALERT) break;
      case 2:
        if ( meta < STS_RELAX) break;
      case 0:
        //Serial.printf( "[%i]", loops);
        heltec_led( LED_LOW);
        break;

      default:
        heltec_led(LED_OFF);
        //Serial.printf( "#%i#", loops);
    }

    loops++;
    delayMs -= 100;
    delay( 100);
  }

  return delayMs;
}

// result code to text - these are error codes that can be raised when using LoRaWAN
// however, RadioLib has many more - see https://jgromes.github.io/RadioLib/group__status__codes.html for a complete list
String stateDecode(const int16_t result) {
  switch (result) {
  case RADIOLIB_ERR_NONE:
    return "ERR_NONE";
  case RADIOLIB_ERR_CHIP_NOT_FOUND:
    return "ERR_CHIP_NOT_FOUND";
  case RADIOLIB_ERR_PACKET_TOO_LONG:
    return "ERR_PACKET_TOO_LONG";
  case RADIOLIB_ERR_RX_TIMEOUT:
    return "ERR_RX_TIMEOUT";
  case RADIOLIB_ERR_CRC_MISMATCH:
    return "ERR_CRC_MISMATCH";
  case RADIOLIB_ERR_INVALID_BANDWIDTH:
    return "ERR_INVALID_BANDWIDTH";
  case RADIOLIB_ERR_INVALID_SPREADING_FACTOR:
    return "ERR_INVALID_SPREADING_FACTOR";
  case RADIOLIB_ERR_INVALID_CODING_RATE:
    return "ERR_INVALID_CODING_RATE";
  case RADIOLIB_ERR_INVALID_FREQUENCY:
    return "ERR_INVALID_FREQUENCY";
  case RADIOLIB_ERR_INVALID_OUTPUT_POWER:
    return "ERR_INVALID_OUTPUT_POWER";
  case RADIOLIB_ERR_NETWORK_NOT_JOINED:
	  return "RADIOLIB_ERR_NETWORK_NOT_JOINED";
  case RADIOLIB_ERR_DOWNLINK_MALFORMED:
    return "RADIOLIB_ERR_DOWNLINK_MALFORMED";
  case RADIOLIB_ERR_INVALID_REVISION:
    return "RADIOLIB_ERR_INVALID_REVISION";
  case RADIOLIB_ERR_INVALID_PORT:
    return "RADIOLIB_ERR_INVALID_PORT";
  case RADIOLIB_ERR_NO_RX_WINDOW:
    return "RADIOLIB_ERR_NO_RX_WINDOW";
  case RADIOLIB_ERR_INVALID_CID:
    return "RADIOLIB_ERR_INVALID_CID";
  case RADIOLIB_ERR_UPLINK_UNAVAILABLE:
    return "RADIOLIB_ERR_UPLINK_UNAVAILABLE";
  case RADIOLIB_ERR_COMMAND_QUEUE_FULL:
    return "RADIOLIB_ERR_COMMAND_QUEUE_FULL";
  case RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND:
    return "RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND";
  case RADIOLIB_ERR_JOIN_NONCE_INVALID:
    return "RADIOLIB_ERR_JOIN_NONCE_INVALID";
/*
  case RADIOLIB_ERR_N_FCNT_DOWN_INVALID:
    return "RADIOLIB_ERR_N_FCNT_DOWN_INVALID";
  case RADIOLIB_ERR_A_FCNT_DOWN_INVALID:
    return "RADIOLIB_ERR_A_FCNT_DOWN_INVALID";
*/

  case RADIOLIB_ERR_MIC_MISMATCH:
 	  return "The downlink MIC could not be verified (incorrect key or invalid FCnt)";
  case RADIOLIB_ERR_MULTICAST_FCNT_INVALID:
 	  return "Multicast frame counter is invalid (outside bounds).";

  case RADIOLIB_ERR_DWELL_TIME_EXCEEDED:
    return "RADIOLIB_ERR_DWELL_TIME_EXCEEDED";
  case RADIOLIB_ERR_CHECKSUM_MISMATCH:
    return "RADIOLIB_ERR_CHECKSUM_MISMATCH";
  case RADIOLIB_ERR_NO_JOIN_ACCEPT:
    return "RADIOLIB_ERR_NO_JOIN_ACCEPT";
  case RADIOLIB_LORAWAN_SESSION_RESTORED:
    return "RADIOLIB_LORAWAN_SESSION_RESTORED";
  case RADIOLIB_LORAWAN_NEW_SESSION:
    return "RADIOLIB_LORAWAN_NEW_SESSION";
  case RADIOLIB_ERR_NONCES_DISCARDED:
    return "RADIOLIB_ERR_NONCES_DISCARDED";
  case RADIOLIB_ERR_SESSION_DISCARDED:
    return "RADIOLIB_ERR_SESSION_DISCARDED";
  }
  return "See https://jgromes.github.io/RadioLib/group__status__codes.html";
}

void dumpDownlinkStats(const int16_t state) {
  // print RSSI (Received Signal Strength Indicator)
  Serial.print(F("[LoRaWAN] RSSI:\t\t"));
  Serial.print(radio.getRSSI());
  Serial.println(F(" dBm"));

  // print SNR (Signal-to-Noise Ratio)
  Serial.print(F("[LoRaWAN] SNR:\t\t"));
  Serial.print(radio.getSNR());
  Serial.println(F(" dB"));

  // print extra information about the event
  Serial.println(F("[LoRaWAN] Event information:"));
  Serial.print(F("[LoRaWAN] Confirmed:\t"));
  Serial.println(downlinkDetails.confirmed);
  Serial.print(F("[LoRaWAN] Confirming:\t"));
  Serial.println(downlinkDetails.confirming);
  Serial.print(F("[LoRaWAN] Datarate:\t"));
  Serial.println(downlinkDetails.datarate);
  Serial.print(F("[LoRaWAN] Frequency:\t"));
  Serial.print(downlinkDetails.freq, 3);
  Serial.println(F(" MHz"));
  Serial.print(F("[LoRaWAN] Frame count:\t"));
  Serial.println(downlinkDetails.fCnt);
  Serial.print(F("[LoRaWAN] Port:\t\t"));
  Serial.println(downlinkDetails.fPort);
  Serial.print(F("[LoRaWAN] Time-on-air: \t"));
  Serial.print(node->getLastToA());
  Serial.println(F(" ms"));
  Serial.print(F("[LoRaWAN] Rx window: \t"));
  Serial.println(state);

  uint8_t margin = 0;
  uint8_t gwCnt = 0;
  if(node->getMacLinkCheckAns(&margin, &gwCnt) == RADIOLIB_ERR_NONE) {
    Serial.print(F("[LoRaWAN] LinkCheck margin:\t"));
    Serial.println(margin);
    Serial.print(F("[LoRaWAN] LinkCheck count:\t"));
    Serial.println(gwCnt);
  }

  uint32_t networkTime = 0;
  uint16_t fracSecond = 0;
  if(node->getMacDeviceTimeAns(&networkTime, &fracSecond, true) == RADIOLIB_ERR_NONE) {
    Serial.print(F("[LoRaWAN] DeviceTime Unix:\t"));
    Serial.println(networkTime);
    Serial.print(F("[LoRaWAN] DeviceTime second:\t1/"));
    Serial.println(fracSecond);
  }
}
// helper function to display any issues
void debug(bool failed, const __FlashStringHelper* message, int state, bool halt) {
  if(failed) {
    Serial.print(message);
    Serial.print(" - ");
    Serial.print(stateDecode(state));
    Serial.print(" (");
    Serial.print(state);
    Serial.println(")");
    while(halt) { delay(1); }
  }
}

// helper function to display a byte array
void arrayDump(uint8_t *buffer, uint16_t len) {
  for(uint16_t c = 0; c < len; c++) {
    char b = buffer[c];
    if(b < 0x10) { Serial.print('0'); }
    Serial.print(b, HEX);
    Serial.print(' ');
  }
  Serial.println();
}

byte crcCheck( byte* buf, byte maxIdx) {
  byte crc = OneWire::crc8( buf, maxIdx);
  if ( crc != buf[maxIdx]) {
#ifdef CRC_DEBUG
    Serial.print("CRC err ");
    Serial.print( maxIdx, DEC);
    Serial.print(" ");
    Serial.print( crc, HEX);
    Serial.print( " != ");
    Serial.println( buf[maxIdx], HEX);
#endif
    return TRUE;
  }

  return FALSE;
}

uint32_t calculateCRC32(const uint8_t *data, size_t len) {
#ifdef CRC_DEBUG
  Serial.print( "crc32 @");
  Serial.print( (uint32_t) data);
  Serial.print( " #");
  Serial.print( len);
#endif
  uint32_t crc = 0xffffffff;
  while (len--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
#ifdef CRC_DEBUG
  Serial.print( " = ");
  Serial.print( crc,  16);
  Serial.println("");
#endif

  return crc;
}