
/***************************************************
  Campervan Control system

  This program reads battery voltage, temeperature and water level (via an ultrasonic
  sensor) and displays on an interactive Adafruit resistive touchscreen.

  Written by Dan Wilson
 ****************************************************/

#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Wire.h"      // this is needed even tho we aren't using it
#include "Adafruit_ILI9341.h"
#include "Adafruit_STMPE610.h"
#include "OneWire.h"
#include "math.h"


//Pin allocation - analog
#define PIN_BATT 0          //Analog input pin for battery voltage
#define PIN_TEMP_INT 1      //Analog input pin for internal temperature
//#define PIN_TEMP_EXT 2      //Analog input pin for external temperature
#define PIN_CURR_1 4        //Analog input pin for circuit 1 current
#define PIN_CURR_2 5        //Analog input pin for circuit 2 current

//Pin allocation - digital
#define PIN_TEMP_SUPPLY 2   //Digital supply pin for thermistors
#define PIN_US_TRIGGER 6    //Ultrasound trigger pin
#define PIN_US_ECHO 7       //Ultrasound echo pin
#define PIN_US_POWER 5      //Ultrasound power supply pin
#define PIN_ONEWIRE 4       //OneWire bus pin

#define DELAY_WATER 60000        //Water measurement peridoicity (60,000ms = 1 minute)
#define DELAY_TEMP 120000        //Temperature measurement peridoicity (120,000ms = 2 minutes)
#define DELAY_POWER 5000         //Voltage measurement peridoicity (5,000ms = 5 seconds)

#define HIST_WATER 1             //The number of water readings to take between storing one
#define HIST_TEMP 8              //The number of temperature readings to take between storing one
#define HIST_POWER 8             //The number of voltage readings to take between storing one
#define HIST_CURRENT 1

#define TFT_DC 9            //Data/Command pin for the TFT
#define TFT_CS 10           //Chip select pin for the TFT
#define TFT_BACKLIGHT 3     //Backlight supply pin for TFT screen

#define STMPE_CS 8          //Chip select pin for the touchscreen

//TFT Screen configuration

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

//Touch screen configuration
// This is calibration data for the raw touch data to the screen coordinates
#define TS_MINX 150
#define TS_MINY 130
#define TS_MAXX 3800
#define TS_MAXY 4000

// The STMPE610 uses hardware SPI on the shield, and #8
Adafruit_STMPE610 ts = Adafruit_STMPE610(STMPE_CS);

//Character size constants (in pixels)
#define CHAR_X 6
#define CHAR_Y 8

//DS18B20 Temperature sensor parameters
// OneWire commands
#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH     0x48  // Copy EEPROM
#define READSCRATCH     0xBE  // Read EEPROM
#define WRITESCRATCH    0x4E  // Write to EEPROM
#define RECALLSCRATCH   0xB8  // Reload from last known
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition

// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC 8

//Mode varables
uint8_t intActivePage;      // Holds which display page is currently active (range 1..4)
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 1000;  // the debounce time; increase if the output flickers
long sleepDelay = 120000;   // Time of inactivity (ms) before the screen back light turns off
bool screenSleep = false;   // Current sleep state of the screen (i.e. if the backlight on or off)

//-----------------------------------------------------------------------------------------------------------------
//                                readVcc CODE
//-----------------------------------------------------------------------------------------------------------------
const double VOLT_REF_VALUE = 1.102; //Based on 1.1 * (5075/5068)
//See https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/

//Function to read Vcc from the board (See https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/)
long readVcc()
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = VOLT_REF_VALUE * 1023 * 1000 / result; // Calculate Vcc (in mV);
  return result; // Vcc in millivolts
}

//-----------------------------------------------------------------------------------------------------------------
//                                SENSOR CODE
//-----------------------------------------------------------------------------------------------------------------


//Meta class for sensors
class Sensor
{
    // Class Member Variables
    int numReadings = 0;
    word* sensHistory = 0;
    long lngInterval = 1000;
    byte intHistInterval = 1;
    String sensorName;

    // These maintain the current state
    unsigned long previousMillis;   // will store last time the sensor was measured
    int index;                      // holds the last history slot written to
    int maxIndex;                   // The maximum index counter yet used
    byte intHistCountdown;          // Countdown to next reading which will be logged in the history
    word lastReading;               // Store last reading

    // Constructor - creates a Sensor
    // and initializes the member variables and state
  public:
    Sensor(int readings, long interval, byte histInterval, String strName)
    {
      //Set member variables
      //Set up history
      numReadings = readings;
      lngInterval = interval;
      intHistInterval = histInterval;
      sensHistory = new word [readings];

      //Set Series name
      sensorName = strName;

      //Set up state variables
      previousMillis = 0;
      index = -1;
      maxIndex = -1;
      intHistCountdown = 0;
      lastReading = 0;
    }

    virtual word readSensor()
    {
      return 0;
    }

    bool Update()
    {
      //Check if a reading is due
      if ((millis() - previousMillis < lngInterval) && (maxIndex > 0))
      {
        return false;
      }

      //Re-set previousMillis marker
      previousMillis = millis();

      lastReading = readSensor();

      //Decrement history countdown
      intHistCountdown--;

      if (intHistCountdown <= 0)
      {
        //Increment index
        index++;
        if (index >= numReadings)
        {
          index = 0;
        }

        if (index > maxIndex) maxIndex = index;

        sensHistory[index] = lastReading;

        //Re-set countdown
        intHistCountdown = intHistInterval;

        //Serial.print(sensorName);
        //Serial.print(": ");
        //Serial.println(sensHistory[index],2);
      }

      return true;
    }

    //Returns readings in queue order from 0 to the maximum number of readings
    double getReading(int readingIndex)
    {
      int arrayIndex;
      // Check if we have looped around yet
      if (maxIndex < (numReadings - 1))
      {
        //if the required readings is more than max Index, default to zero
        if (readingIndex > maxIndex)
        {
          return 0;
        }
        else
        {
          return interpret(sensHistory[readingIndex]);
        }
      }
      else
      {
        if (readingIndex < (numReadings - index))
        {
          arrayIndex = index + readingIndex;
        }
        else
        {
          arrayIndex = readingIndex - (maxIndex - index);
        }

        return interpret(sensHistory[arrayIndex]);
      }

    }

    virtual double interpret(word intData)
    {
      return (double) intData;
    }

    void testData(int intOffset)
    {
      int i;
      for (i = 0; i < numReadings; i++)
      {
        sensHistory[i] = (word) 512 * sin((i + intOffset) * 3.142 / (numReadings / 2)) + 511;
        //Serial.println(sensHistory[i],2);
      }

      index = numReadings - 1;
      maxIndex = index;
    }

    //Return latest reading
    double getReading()
    {
      return interpret(lastReading);
    }

    //Return total number of readings
    int getNumReadings()
    {
      return maxIndex;
    }

    double getMinValue()
    {
      int i;
      double val;

      if (maxIndex > -1)
      {
        val = sensHistory[0];

        for (i = 1; i <= maxIndex; i++)
        {
          if (val > sensHistory[i]) val = sensHistory[i];
        }
      }
      else
      {
        val = 0;
      }

      return interpret(val);
    }

    double getMaxValue()
    {
      int i;
      double val;

      if (maxIndex > -1)
      {
        val = sensHistory[0];

        for (i = 1; i <= maxIndex; i++)
        {
          if (val < sensHistory[i]) val = sensHistory[i];
        }
      }
      else
      {
        val = 0;
      }

      return interpret(val);
    }

    String getSeriesName()
    {
      return sensorName;
    }

    void setPeriodicity(long lngDelay)
    {
      lngInterval = lngDelay;

      return;
    }

    ~Sensor()
    {
      if (sensHistory != 0)
      {
        delete [] sensHistory;
      }
    }
};

class Thermistor : public Sensor
{
    // Class Member Variables
    int pinSupply;
    int pinData;


    //---------Thermistor Parameters
    double R0 = 4700; //Thermistor Nominal Value (ohms)
    double B = 3977; //Thermistor beta parameter (K)
    double T0 = 298.15; //Nominal value temperature (25 deg C) (K)

    // Constructor - creates a Thermistor
    // and initializes the member variables and state
  public:
    Thermistor(int dataPin, int supplyPin, int readings, long interval, byte histInterval, String strName): Sensor(readings, interval, histInterval, strName)
    {
      //Set member variables
      //Set pins
      pinData = dataPin;
      pinSupply = supplyPin;

      //Initialise pins
      pinMode(pinSupply, OUTPUT);
      digitalWrite(pinSupply, LOW);

    }

    //Reads voltage from analog pin and returns temperature in centigrade
    // +Vcc --- R0(Thermistor) ---- intPin ----- R0(Fixed) ----- 0V
    word readSensor()
    {
      word val;

      digitalWrite(pinSupply, HIGH);  //Turn on thermistor

      val = (word) analogRead(pinData);    // read the input pin
      delay(1);

      val = (word) analogRead(pinData);    // read the input pin again (seems to avoid issue where pin reads high)

      digitalWrite(pinSupply, LOW); //Turn off thermistor

      return val;
    }

    double interpret(word intData)
    {
      double dblVolt, dblRes, dblTemp, Vcc;

      Vcc = readVcc() / 1000.0;

      dblVolt = Vcc * (intData / 1023.0); // Convert analog reading to voltage

      dblRes = ((Vcc / 2.0) / dblVolt) * R0; //Calculate thermistor resistance

      dblTemp = (1 / ((1 / T0) + (1 / B) * log(dblRes / R0))) - 273.15; //Steinhart–Hart equation (Beta parameter equation) (converted to centigrade)

      return dblTemp;
    }


};

//Sensor class to handle DS1820/1822 OneWire temperature sensors
class TempSensor : public Sensor
{
    // Class Member Variables
    OneWire* dataBus;
    byte addr[8];
    byte type_s;
    bool validSensor;
  

    // Constructor - creates a Voltmeter
    // and initializes the member variables and state
  public:
    TempSensor(OneWire* ds, int readings, long interval, byte histInterval, String strName): Sensor(readings, interval, histInterval, strName)
    {
      //Set member variables
      dataBus=ds;
      validSensor = true;

      //Local counter
      byte i;

      //Check that a valid sensor is attached
      if ( !dataBus->search(addr)) {
        validSensor=false;
        return;
      }
    
      // Get the type of sensor
      // the first ROM byte indicates which chip
      switch (addr[0]) {
        case 0x10:        //DS18S20 or DS1820
          type_s = 1;
          break;
        case 0x28:        //DS18B20
          type_s = 0;
          break;
        case 0x22:        //DS1822
          type_s = 0;
          break;
        default:          //Device is not a DS18x20 family device
          validSensor=false;
          return;
      } 
    }

    word readSensor()
    {
      word val;
       byte i;
        byte data[12];
        int16_t raw;
        float celsius;

        //If no sensor is attached, default to absolute zero (-273.15 degrees C)
        if (validSensor==false)
        {
          val = -27315;
          return val;
        }

        //Prepare to fetch data
        dataBus->reset();
        dataBus->select(addr);
        dataBus->write(STARTCONVO, 1);        // start conversion, with parasite power on at the end
        
        delay(1000);     // maybe 750ms is enough, maybe not
        // we might do a ds.depower() here, but the reset will take care of it.
        
        dataBus->reset();
        dataBus->select(addr);    
        dataBus->write(READSCRATCH);         // Read Scratchpad

        //Fetch data
        for ( i = 0; i < 9; i++) {           // we need 9 bytes
          data[i] = dataBus->read();
        }

        
        //Assume 12 bit resolution
        //Shift data 3 bits up the register to ensure sign  bit is used - raw data is scaled to 2^-7
        raw = (((int16_t) data[TEMP_MSB]) << 11) | (((int16_t) data[TEMP_LSB]) << 3);

        //Convert raw data to celsius - divide raw data by 2^7
        celsius = (float)raw / 128.0;
        
        
        //Convert calculated temperature to a word variable to save memory (multiply by 100 to keep value to 2dp)
        val = (word) round(celsius * 100);
     
      return val;
    }

    double interpret(word intData)
    {
      double dblTemp;

      //Convert stored value back to double temperature reading
      dblTemp = (double) (intData / 100.0);

      return dblTemp;

    }

    bool valid()
    {
      return validSensor;
    }
};

class Voltmeter : public Sensor
{
    // Class Member Variables
    int pinData;

    //Battery Pot Div Params
    double RBatt1 = 2; // MOhms
    double RBatt2 = 1; // MOhms

    //State of Charge constants (y=mx+c)
    double dblSOCm = 1.19;
    double dblSOCc = -14.0;

    // Constructor - creates a Voltmeter
    // and initializes the member variables and state
  public:
    Voltmeter(int dataPin, int readings, long interval, byte histInterval): Sensor(readings, interval, histInterval, "Voltage")
    {
      //Set member variables
      //Set pins
      pinData = dataPin;
    }

    word readSensor()
    {
      word val;

      val = (word) analogRead(pinData);    // read the input pin
      delay(1);

      val = (word) analogRead(pinData);    // read the input pin again (seems to avoid issue where pin reads high)

      return val;
    }

    double interpret(word intData)
    {
      double dblVolt, dblBattVolt, Vcc;

      Vcc = readVcc() / 1000.0;

      dblVolt = Vcc * (intData / 1023.0); // Convert analog reading to voltage

      dblBattVolt = dblVolt * (RBatt1 + RBatt2) / RBatt2;

      //Serial.print("Read Voltage as: ");
      //Serial.println(dblBattVolt,2);

      return dblBattVolt;

    }

    //Calculate State of Charge from voltage
    double getSOC()
    {
      double dblSOC;

      dblSOC = ((dblSOCm * getReading()) + dblSOCc) * 100;

      if (dblSOC < 0)
      {
        dblSOC = 0;
      }
      else if (dblSOC > 100)
      {
        dblSOC = 100;
      }

      return dblSOC;
    }
};

class Ammeter : public Sensor
{
    // Class Member Variables
    int pinData;

    //ACS Parameters
    const int mVperAmp = 66; // use 185 for 5A Module, 100 for 20A Module and 66 for 30A Module
   

    // Constructor - creates a Voltmeter
    // and initializes the member variables and state
  public:
    Ammeter(int dataPin, int readings, long interval, byte histInterval): Sensor(readings, interval, histInterval, "Current")
    {
      //Set member variables
      //Set pins
      pinData = dataPin;
    }

    word readSensor()
    {
      word val;

      val = (word) analogRead(pinData);    // read the input pin
      delay(1);

      val = (word) analogRead(pinData);    // read the input pin again (seems to avoid issue where pin reads high)

      return val;
    }

    double interpret(word intData)
    {
      double dblVolt, dblAmp, Vcc, ACSoffset;

      Vcc = (double) readVcc() * 0.863;
      ACSoffset = Vcc / 2;

      dblVolt = (intData / 1023.0) * Vcc; // Gets you mV
      dblAmp = ((dblVolt - ACSoffset) / mVperAmp);

      return dblAmp;

    }
};



class Ultrasound : public Sensor
{
    // Class Member Variables
    int pinEcho;
    int pinTrigger;
    int pinPower;

    //Water measurement parameters
    double dblSpeedSound = 343.0; // Speed of sound (m/s)
    double tankProfileZ[4] = {0, 380, 380, 600}; //Vertical axis
    double tankProfileX[4] = {290, 350, 650, 650}; //Length
    double tankProfileY[4] = {200, 200, 200, 200}; //Width

    //Calculate the volume of water in the tank based on the water depth.
    //For each tank section, x(z) = ((x1-x0)/(z1-z0))*z + x0 and y(z)=((y1-y0)/(z1-z0))*z + y0
    //Therefore, the area at depth z is A(z)=x(z) * y(z)
    //and the volume is the definite integral of A(z) with respect to z between the bottom of the section at the water depth or the top of the section, whichever is the smaller
    // V(z) = alpha/3 * z^3 + beta/2 * z^2 + gamma * z
    //  where
    //      alpha = (x1-x0)       (y1-y0)
    //              -------   *   -------
    //              (z1-z0)       (z1-z0)
    //
    //      beta  = (x1-x0)        (y1-y0)
    //              ------- * y0 + ------- * x0
    //              (z1-z0)        (z1-z0)
    //
    //      gamma = x0 * y0

    //  As the measurements are in mm, the volume is in mm^3 so must be divided by 1e6 to return litres.
    double calcWaterVolume(double waterDepth)
    {

      int numSteps = sizeof(tankProfileZ) / sizeof(double);
      double maxZ;
      float alpha, beta, gamma;
      int i;
      double waterVolume;

      maxZ = tankProfileZ[numSteps - 1];

      waterVolume = 0;

      for (i = 1; i < numSteps; i++)
      {
        //Check if this is a real band as opposed to a transition (where a horizontal surface occurs)
        if (tankProfileZ[i] - tankProfileZ[i - 1] > 0)
        {
          alpha = ((tankProfileX[i] - tankProfileX[i - 1]) * (tankProfileY[i] - tankProfileY[i - 1]))/pow((tankProfileZ[i] - tankProfileZ[i - 1]),2);
          beta = (((tankProfileX[i] - tankProfileX[i - 1]) * tankProfileY[i - 1]) + ((tankProfileY[i] - tankProfileY[i - 1]) * tankProfileX[i - 1]))/(tankProfileZ[i] - tankProfileZ[i - 1]);
          gamma = tankProfileX[i - 1] * tankProfileY[i - 1];
  
          //See if level is in the band 
          if (waterDepth >= tankProfileZ[i])
          {
            //If the level is above the band - integrate between top and bottom and add to volume
            waterVolume += ((alpha / 3.0) * pow(tankProfileZ[i], 3) + (beta / 2.0) * pow(tankProfileZ[i], 2) + gamma * tankProfileZ[i]) -
                           ((alpha / 3.0) * pow(tankProfileZ[i - 1], 3) + (beta / 2.0) * pow(tankProfileZ[i - 1], 2) + gamma * tankProfileZ[i - 1]);
          }
  
          else if ((waterDepth >= tankProfileZ[i - 1]) && (waterDepth < tankProfileZ[i]))
          {
  
            //If the water level is within band - integrate between the bottom and the current depth and add to volume
            waterVolume += ((alpha / 3.0) * pow(waterDepth, 3) + (beta / 2.0) * pow(waterDepth, 2) + gamma * waterDepth) -
                           ((alpha / 3.0) * pow(tankProfileZ[i - 1], 3) + (beta / 2.0) * pow(tankProfileZ[i - 1], 2) + gamma * tankProfileZ[i - 1]);
  
          }
  
        }
      }

      //Convert water volume from cubic millimeters to litres
      waterVolume = waterVolume / 1000000.0;

      return waterVolume;

    }

    // Constructor - creates an ultrasound sensor
    // and initializes the member variables and state
  public:
    Ultrasound(int powerPin, int triggerPin, int echoPin, int readings, long interval, byte histInterval): Sensor(readings, interval, histInterval, "Water level")
    {
      //Set member variables
      //Set pins
      pinEcho = echoPin;
      pinTrigger = triggerPin;
      pinPower = powerPin;

      //Initialise pins
      pinMode(powerPin, OUTPUT);
      pinMode(triggerPin, OUTPUT);
      pinMode(echoPin, INPUT);

      digitalWrite(powerPin, HIGH);
    }

    word readSensor()
    {
      long duration;
      word distance;

      //digitalWrite(pinPower, HIGH); //Turn on sensor

      digitalWrite(pinTrigger, LOW);  //Reset trigger
      delayMicroseconds(2); // Wait for signal to settle
      digitalWrite(pinTrigger, HIGH); //Start trigger pulse
      delayMicroseconds(10); // Pulse duration
      digitalWrite(pinTrigger, LOW); //End pulse

      duration = pulseIn(pinEcho, HIGH); //Measure echo time delay
      distance = (word) round(((duration / 2) * dblSpeedSound) / 1000.0); //Distance (in mm)

      //Serial.print("Read distance as ");
      //Serial.println(distance,2);

      //digitalWrite(pinPower, LOW); //Turn off sensor

      
      return distance;
    }

    //Public function - return current water volume
    double calcWaterVolume()
    {
      int numSteps = sizeof(tankProfileZ) / sizeof(double);
      double maxZ;

      maxZ = tankProfileZ[numSteps - 1];

      return calcWaterVolume(maxZ - getReading());
    }

    //Return current volume as a percentage of the total volume of the tank
    double calcPercentRemaining()
    {
      int numSteps = sizeof(tankProfileZ) / sizeof(double);


      return calcWaterVolume() / calcWaterVolume(tankProfileZ[numSteps - 1]) * 100.0;


    }
};

// Meta class for display pages
class DisplayPage
{
  public:
    virtual void refresh(Sensor* updatedRead);

    DisplayPage()
    {

    }
};

// Graphing page
class GraphPage : public DisplayPage
{
  private:
    // Class Member Variables
    int numSeries = 0;
    Sensor* series[3];
    uint16_t seriesColours[3] = {ILI9341_BLUE, ILI9341_RED, ILI9341_GREEN};
    int intNumSeries = 3;
    String strYTitle;
    String strXTitle;
    long timeMultiplier;


    int getYMax()
    {
      int i, maxValue;

      maxValue = ceil(series[0]->getMaxValue());

      //Serial.print("Series 0 Max: ");
      //Serial.println(maxValue);

      for (i = 1; i < (intNumSeries); i++)
      {
        //Serial.print("Series ");
        //Serial.print(i);
        //Serial.print(" Max: ");
        //Serial.println(series[i]->getMaxValue());
        if (maxValue < ceil(series[i]->getMaxValue()))
        {
          maxValue = ceil(series[i]->getMaxValue());
        }
      }

      return maxValue;
    }

    int getYMin()
    {
      int i, minValue;

      minValue = series[0]->getMinValue();

      for (i = 1; i < (intNumSeries); i++)
      {
        if (minValue > series[i]->getMinValue())
        {
          minValue = series[i]->getMinValue();
        }
      }

      return minValue;
    }

    int getXMax()
    {
      int i, numReadings;

      numReadings = series[0]->getNumReadings();

      for (i = 1; i < (intNumSeries - 1); i++)
      {
        if (numReadings < series[i]->getNumReadings())
        {
          numReadings = series[i]->getNumReadings();
        }
      }

      return numReadings;
    }

  public:

    GraphPage(String xTitle, String yTitle, Sensor* Series1, Sensor* Series2, Sensor* Series3, long timeUnit): DisplayPage()
    {
      strXTitle = xTitle;
      strYTitle = yTitle;

      timeMultiplier = timeUnit;

      series[0] = Series1;
      series[1] = Series2;
      series[2] = Series3;

      if (Series1 == NULL)
      {
        intNumSeries = 0;
      }
      else if (Series2 == NULL)
      {
        intNumSeries = 1;
      }
      else if (Series3 == NULL)
      {
        intNumSeries = 2;
      }
      else
      {
        intNumSeries = 3;
      }
    }

    //Calculate the increment between markers on the axis
    int calcIncrement(int intAxesMin, int intAxesMax, int intPixelRange, bool horizontal, long timeMultiplier)
    {
      double numIncr;
      int incr, order;

      if (horizontal)
      {
        if (timeMultiplier == NULL)
        {
          numIncr = intPixelRange / (CHAR_X * 1.00 * ceil(log10(intAxesMax)));
        }
        else
        {
          numIncr = intPixelRange / (CHAR_X * 1.00 * (ceil(log10((intAxesMax * timeMultiplier)/3600.00))+3.00));
        }
      }
      else
      {
        numIncr = intPixelRange / (CHAR_Y * 1.00);
      }

      incr = ceil((intAxesMax - intAxesMin) / numIncr);
      order = pow(60, floor(log10(incr)/log10(60)));

      incr = ceil(incr / (order * 1.0)) * order;

      if ((incr / order > 2) && (incr / order <= 5))
      {
        incr = 5 * order;
      }
      else if (incr / order > 5)
      {
        incr = 10 * order;
      }

      return incr;
    }

    int calcIncrement(int intAxesMin, int intAxesMax, int intPixelRange, bool horizontal)
    {
      return calcIncrement(intAxesMin, intAxesMax, intPixelRange, horizontal, NULL);
    }

    

    //Refresh graph
    void refresh(Sensor* updatedRead)
    {
      int i, j, incr;

      //Check if the displayed parameters have updated
      if (updatedRead != NULL)
      {

        bool boolUpdate = false;
        for (i = 0; i < (intNumSeries); i++)
        {
          if (updatedRead == series[i])
          {
            boolUpdate = true;
          }
        }

        if (boolUpdate == false) return;
      }

      int xAxis = (tft.height() / 5) * 4 - 20 - (CHAR_Y * 3);
      int yAxis = 35;

      int minY = getYMin();
      int maxY = getYMax();
      int minX = 0;
      int maxX = getXMax();
      double reading;
      Sensor* curSeries;

      int intHours;
      int intMinutes;

      //Serial.print("minY: ");
      //Serial.println(minY);
      //Serial.print("maxY: ");
      //Serial.println(maxY);

      //Clear top part of display
      tft.fillRect(0, 0, tft.width(), (tft.height() / 5) * 4 - 1, ILI9341_WHITE);
      yield();

      //Draw axes
      tft.drawFastVLine(yAxis, 10, xAxis - 10, ILI9341_BLACK);
      tft.drawFastHLine(yAxis, xAxis, tft.width() - yAxis - 10, ILI9341_BLACK);

      tft.setTextColor(ILI9341_BLACK);
      tft.setTextSize(1);

      //if there are less than 2 readings, give up
      if (maxX - minX < 2) return;

      //Draw Y axis labels
      if (maxY == minY)
      {
        maxY = minY + 1;
      }
      incr = calcIncrement(minY, maxY, xAxis - 10, false);

      for (i = minY; i <= maxY; i = i + incr)
      {
        tft.setCursor(yAxis - 2 - (CHAR_X * 2), map(i, minY, maxY, xAxis, 10) - (CHAR_Y / 2));
        tft.print(i);
      }

      //Draw X axis labels
      incr = calcIncrement(minX, maxX, tft.width() - 20 - yAxis, true, timeMultiplier);


      for (i = minX; i <= maxX; i = i + incr)
      {
        tft.setCursor(map(i, minX, maxX, yAxis, (tft.width() - 20)), xAxis + 2);
        intHours = (int) floor(((maxX - i) * (timeMultiplier)) / 3600);
        intMinutes = (int) (((maxX - i) * (timeMultiplier)) % 3600) * (60.0/3600.0);
        tft.print(intHours);
        tft.print(":");
        if (intMinutes < 10)
        {
          tft.print("0");
        }
        tft.print(intMinutes);
      }

      //Draw titles
      tft.setCursor(((tft.width() - yAxis - 10) / 2) + yAxis - (strXTitle.length() / 2) , xAxis + 4 + CHAR_Y);
      tft.print(strXTitle);

      tft.setRotation(0);

      tft.setCursor(tft.width() - xAxis + ((xAxis - (strYTitle.length() * CHAR_X)) / 2), 5);
      tft.print(strYTitle);

      tft.setRotation(1);


      for (j = 0; j < (intNumSeries); j++)
      {
        curSeries = series[j];

        for (i = 1; i <= maxX; i++)
        {
          tft.drawLine(map(i - 1, minX, maxX, yAxis, tft.width() - 10), map((curSeries->getReading(i - 1) * 10), minY * 10, maxY * 10, xAxis, 10), map(i, minX, maxX, yAxis, tft.width() - 10), map((curSeries->getReading(i) * 10), minY * 10, maxY * 10, xAxis, 10), seriesColours[j]);
        }

        //Show Key

        tft.setCursor(j * (tft.width() / intNumSeries) + 20, xAxis + 4 + 2 * CHAR_Y);
        tft.print(series[j]->getSeriesName());
        tft.setCursor(j * (tft.width() / intNumSeries) + 5, xAxis + 4 + 3 * CHAR_Y);
        tft.print(series[j]->getSeriesName());
        tft.print(": ");
        tft.print(series[j]->getReading(), 1);
        tft.drawFastHLine(j * (tft.width() / intNumSeries) + 5, xAxis + 4 + 5 * (CHAR_Y / 2), 10, seriesColours[j]);

      }

    }


};

class MainPage : public DisplayPage
{
    //Demo Data
    double Powerlevel = 99;
    double Voltage = 12.8;
    double Current = 1.2;
    double dblExtTemp = 11.2;

    Sensor* intTemp;
    Sensor* extTemp;
    Voltmeter* battVolt;
    Ultrasound* tankVolume;
    Sensor* circ1Curr;
    Sensor* circ2Curr;

  public:
    MainPage(Sensor* voltmeter, Sensor* circ1curr, Sensor* circ2curr, Sensor* internalTemp, Sensor* externalTemp, Sensor* waterTank)
    {
      battVolt = voltmeter;
      this->circ1Curr = circ1curr;
      this->circ2Curr = circ2curr;
      intTemp = internalTemp;
      extTemp = externalTemp;
      tankVolume = waterTank;
    }

    void refresh(Sensor* updatedRead)
    {
      int x, i, y, rectWidth, rectHeight, intTxtSize;

      //If we are doing a full refresh then clear the screen and draw columns
      if (updatedRead == NULL)
      {
        //Clear top part of display
        tft.fillRect(0, 0, tft.width(), (tft.height() / 5) * 4 - 1, ILI9341_WHITE);
        yield();

        //Draw vertical dividers

        for (i = 1; i < 3; i++)
        {
          x = (tft.width() / 3) * i;

          tft.drawFastVLine(x, 5, (tft.height() / 5) * 4 - 10, ILI9341_BLACK);

        }
      }

      //Power Section
      //If we're doing a full refresh or power refresh then re-darw the power column
      if ((updatedRead == NULL) || (updatedRead == battVolt))
      {
        //if we're just doing the power section then clear the column
        if (updatedRead == battVolt)
        {
          tft.fillRect(0, 0, (tft.width() / 3) - 1, (tft.height() / 5) * 4 - 1, ILI9341_WHITE);
          yield();

        }

        //Draw battery
        x = (tft.width() / 6) - 20;
        y = 10;

        tft.drawRect(x, y + 3, 40, 20, ILI9341_BLACK);
        tft.fillRect(x + 7, y, 6, 3, ILI9341_BLACK);
        tft.fillRect(x + 27, y, 6, 3, ILI9341_BLACK);
        tft.fillRect(x + 1, y + 5 + (18 - 18 * (battVolt->getSOC() / 100.0)), 38, 18 * (battVolt->getSOC() / 100.0), ILI9341_GREEN);

        tft.setTextColor(ILI9341_BLACK);
        tft.setTextSize(1);
        tft.setCursor(x + 7, y + 9);
        tft.print("-");

        tft.setCursor(x + 27, y + 9);
        tft.print("+");

        //Print power level
        tft.setTextColor(ILI9341_BLACK);
        tft.setTextSize(2);
        tft.setCursor((tft.width() / 6) - (CHAR_X * 2  * 5 / 2), 45);
        tft.print(battVolt->getSOC(), 1);
        tft.print("%");

        //Print voltage level
        tft.setTextColor(ILI9341_BLACK);
        tft.setTextSize(1);
        tft.setCursor(5, 100);
        tft.print(F("Voltage: "));
        tft.print(battVolt->getReading(), 1);
        tft.println("V");

        //Print current level
        tft.print(F(" Current: "));
        tft.print(abs(circ1Curr->getReading()) + abs(circ2Curr->getReading()), 2);
        tft.println("A");

        //tft.print("Vcc: ");
        //tft.print((double)readVcc(), 2);
        //tft.println("mV");

      }

      //Temperature Section
      //If we're doing a full refresh then draw the thermometer
      if (updatedRead == NULL)
      {
        //Draw thermometer
        x = (tft.width() / 2) - 4;
        y = 10;

        tft.drawFastVLine(x + 2, y + 1, 18, ILI9341_BLACK);
        tft.drawFastVLine(x + 6, y + 1, 18, ILI9341_BLACK);
        tft.drawFastHLine(x + 3, y, 3, ILI9341_BLACK);

        tft.drawPixel(x + 1, y + 19, ILI9341_BLACK);
        tft.drawPixel(x + 7, y + 19, ILI9341_BLACK);
        tft.drawFastVLine(x, y + 20, 2, ILI9341_BLACK);
        tft.drawFastVLine(x + 8, y + 20, 2, ILI9341_BLACK);
        tft.drawPixel(x + 1, y + 22, ILI9341_BLACK);
        tft.drawPixel(x + 7, y + 22, ILI9341_BLACK);
        tft.drawFastHLine(x + 2, y + 23, 5, ILI9341_BLACK);
        tft.drawFastVLine(x + 4, y + 10, 12, ILI9341_RED);


        //Print temperature headings
        tft.setTextColor(ILI9341_BLACK);
        tft.setTextSize(1);
        tft.setCursor(tft.width() / 3 + 5, 60);
        tft.print("Internal:");

        tft.setCursor(tft.width() / 3 + 5, 120);
        tft.print("External:");
      }

      //If we're just updating the temperature then clear the previous figures
      if ((updatedRead == intTemp) || (updatedRead == extTemp))
      {
        tft.fillRect((tft.width() / 3) + 1, 75, (tft.width() / 3) - 2, (CHAR_Y * 2), ILI9341_WHITE);
        tft.fillRect((tft.width() / 3) + 1, 135, (tft.width() / 3) - 2, (CHAR_Y * 2), ILI9341_WHITE);
        yield();
      }

      //If we're updating the temperature then print the new figures
      if (((updatedRead == intTemp) || (updatedRead == extTemp)) || (updatedRead == NULL))
      {
        //Print temperatures
        tft.setTextColor(ILI9341_BLACK);
        tft.setTextSize(2);
        tft.setCursor((tft.width() / 2) - (CHAR_X * 2  * 5 / 2), 75);
        tft.print(intTemp->getReading(), 1);
        tft.print(" C");

        tft.setCursor((tft.width() / 2) - (CHAR_X * 2  * 5 / 2), 135);
        tft.print(extTemp->getReading(), 1);
        tft.print(" C");
      }


      //Water section
      if ((updatedRead == NULL) || (updatedRead == tankVolume))
      {
        //if we're just doing the power section then clear the column
        if (updatedRead == tankVolume)
        {
          tft.fillRect((tft.width() / 3) * 2 + 1, 0, tft.width(), (tft.height() / 5) * 4 - 1, ILI9341_WHITE);
          yield();

        }

        //Draw water tank
        x = (tft.width() / 6) * 5 - 15;
        y = 10;

        tft.drawFastVLine(x, y + 3, 20, ILI9341_BLACK);
        tft.drawFastVLine(x + 29, y + 3, 20, ILI9341_BLACK);
        tft.drawFastHLine(x, y + 23, 30, ILI9341_BLACK);
        tft.fillRect(x + 1, y + 6 + (18 - 18 * (tankVolume->calcPercentRemaining() / 100.0)), 28, 18 * (tankVolume->calcPercentRemaining() / 100.0), ILI9341_BLUE);

        //Print water level
        tft.setTextColor(ILI9341_BLACK);
        tft.setTextSize(2);
        tft.setCursor((tft.width() / 6) * 5 - (CHAR_X * 2  * 6 / 2), 45);
        tft.print(tankVolume->calcPercentRemaining(), 1);
        tft.print("%");

        //Print water volume
        tft.setTextColor(ILI9341_BLACK);
        tft.setTextSize(1);
        tft.setCursor(tft.width() / 3 * 2 + 5, 100);
        tft.print("Remaining: ");
        tft.setCursor(tft.width() / 3 * 2 + 10, 100 + CHAR_Y);
        tft.print(tankVolume->calcWaterVolume(), 1);
        tft.println(" litres");
      }
    }

};

class WaterPage : public DisplayPage
{
    Ultrasound* tankVolume;
    int x, y;

  public:
    WaterPage(Sensor* ultrasound)
    {
      tankVolume = ultrasound;
    }

    void refresh(Sensor* updatedRead)
    {
      //Water section
      if ((updatedRead == NULL) || (updatedRead == tankVolume))
      {
        //Set update frequency to every second
        tankVolume->setPeriodicity(1000);

        //If we are doing a full re-draw
        if (updatedRead == NULL)
        {

          //Clear top part of display
          tft.fillRect(0, 0, tft.width(), (tft.height() / 5) * 4 - 1, ILI9341_WHITE);
          yield();


          //Draw water tank
          x = (tft.width() / 3) * 2 + 10;
          y = 10;

          tft.drawFastVLine(x, y + 3, 150, ILI9341_BLACK);
          tft.drawFastVLine(x + 50, y + 3, 150, ILI9341_BLACK);
          tft.drawFastHLine(x, y + 153, 50, ILI9341_BLACK);

        }
        else
        {
          //Clear text area
          tft.fillRect(10 + 20 * CHAR_X, 10 + CHAR_Y * 2 + 5, x - (10 + 20 * CHAR_X) - 1, 10 + CHAR_Y * 4 + 5, ILI9341_WHITE);

          //Clear tank graphic
          tft.fillRect(x + 1, y + 6, 49, 148, ILI9341_WHITE);
        }

        //Draw title
        tft.setTextColor(ILI9341_BLACK);
        tft.setTextSize(2);
        tft.setCursor(10, 10);
        tft.println("Water Tank");

        //Print details
        tft.setTextSize(1);
        tft.setCursor(10, 10 + CHAR_Y * 2 + 5);
        tft.print("Volume Remaining: ");


        tft.setCursor(10, 10 + CHAR_Y * 3 + 5);
        tft.print("Percent Remaining: ");


        tft.setCursor(10, 10 + CHAR_Y * 4 + 5);
        tft.print("(Measurement: ");


        //Update measurements
        //Print details
        tft.setTextSize(1);
        tft.setCursor(10 + 20 * CHAR_X, 10 + CHAR_Y * 2 + 5);
        tft.print(tankVolume->calcWaterVolume(), 1);
        tft.println(" litres");
        tft.println("");
        //Serial.print("Water: ");
        //Serial.println(tankVolume->calcWaterVolume(),1);

        tft.setCursor(10 + 20 * CHAR_X, 10 + CHAR_Y * 3 + 5);
        tft.print(tankVolume->calcPercentRemaining(), 1);
        tft.println("%");
        tft.println("");

        tft.setCursor(10 + 20 * CHAR_X, 10 + CHAR_Y * 4 + 5);
        tft.print(tankVolume->getReading(), 2);
        tft.println(" mm)");

        //Draw tank level
        tft.fillRect(x + 1, y + 6 + (148 - 148 * (tankVolume->calcPercentRemaining() / 100.0)), 49, 148 * (tankVolume->calcPercentRemaining() / 100.0), ILI9341_BLUE);

      }
    }

};


//-----------------------------------------------------------------------------------------------------------------
//                                MAIN CODE
//-----------------------------------------------------------------------------------------------------------------

//---------Initialise sensors
OneWire  ds(PIN_ONEWIRE);  // OneWire bus (a 4.7K resistor is necessary);
TempSensor extTemp(&ds, 90, DELAY_TEMP, HIST_TEMP, "External Temp");                            //External temperature sensor (DS18B20)
Thermistor intTemp(PIN_TEMP_INT, PIN_TEMP_SUPPLY, 90, DELAY_TEMP, HIST_TEMP, "Internal Temp"); //Internal thermistor
//Thermistor extTemp(PIN_TEMP_INT, PIN_TEMP_SUPPLY, 90, DELAY_TEMP, HIST_TEMP, "External Temp"); //External thermistor
Voltmeter battVolt(PIN_BATT, 10, DELAY_POWER, HIST_POWER);                                              //Battery voltage measurement
Ammeter circ1Curr(PIN_CURR_1, 1, DELAY_POWER, HIST_CURRENT);                                              //Circuit 1 current measurement
Ammeter circ2Curr(PIN_CURR_2, 1, DELAY_POWER, HIST_CURRENT);                                              //Circuit 2 current measurement
Ultrasound waterTank(PIN_US_POWER, PIN_US_TRIGGER, PIN_US_ECHO, 2, DELAY_WATER, HIST_WATER);           //Water tank ultrasound sensor

//---------Initialise display pages
GraphPage powerGraph("Time (hours ago)", "Voltage", &battVolt, NULL, NULL, (DELAY_POWER * HIST_POWER)/1000L);
GraphPage tempGraph("Time (hours ago)", "Temperature (C)", &intTemp, &extTemp, NULL, (DELAY_TEMP * HIST_TEMP)/1000L);
MainPage frontPage(&battVolt, &circ1Curr, &circ2Curr, &intTemp, &extTemp, &waterTank);
WaterPage pgWater(&waterTank);
DisplayPage* pages[4] = {&frontPage, &powerGraph, &tempGraph, &pgWater};


void setup() {
  //Serial.begin(9600);
  //Serial.println("Campervan Control System");

  //Initialise TFT screen
  tft.begin();
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH);

  //Rotate screen to landscape
  tft.setRotation(1);

  //Initialise touchscreen and error trap
  if (!ts.begin())
  {
    //Write debug message to screen
    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(0, 0);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(5);
    tft.println(F("Couldn't start touchscreen controller"));
    tft.setTextSize(1);
    tft.println(F("Message will disappear in 5 seconds..."));

    delay(5000);      //Keep message on screen for 5 seconds

    while (1);
  }

  //Initialise page mode
  intActivePage = 0;

  frontPage.refresh(NULL);
  drawMenu();
  //testGraph.refresh();
}


void loop(void)
{

  if (intTemp.Update())
  {
    pages[intActivePage]->refresh(&intTemp);
  }

  if (extTemp.Update())
  {
    pages[intActivePage]->refresh(&extTemp);
  }

  if (battVolt.Update())
  {
    pages[intActivePage]->refresh(&battVolt);
  }

  if (circ1Curr.Update())
  {
    pages[intActivePage]->refresh(&circ1Curr);
  }

  if (circ2Curr.Update())
  {
    pages[intActivePage]->refresh(&circ2Curr);
  }

  if (waterTank.Update())
  {
    pages[intActivePage]->refresh(&waterTank);
  }

  // See if there's any  touch data for us
  if ((ts.bufferEmpty()) || (millis() - lastDebounceTime < debounceDelay))
  {
    //Check if the screen has been idle for more than the sleep time
    if ((screenSleep == false) && (millis() - lastDebounceTime > sleepDelay))
    {
      screenSleep = true;
      digitalWrite(TFT_BACKLIGHT, LOW);
    }

    return;
  }



  //Set debounce marker
  lastDebounceTime = millis();

  //If thr screen is asleep, turn it on and then exit
  if (screenSleep == true)
  {
    screenSleep = false;
    digitalWrite(TFT_BACKLIGHT, HIGH);
    return;
  }


  // Retrieve a point
  TS_Point p = ts.getPoint();
  TS_Point ptScreen;

  ptScreen.x = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
  ptScreen.y = map(p.x, TS_MINX, TS_MAXX, tft.height(), 0);


  //Serial.print("X = "); Serial.print(ptScreen.x);
  //Serial.print("\tY = "); Serial.print(ptScreen.y);
  //Serial.print("\tPressure = "); Serial.println(p.z);

  //Check if touch is in menu section
  if ((ptScreen.y > (tft.height() / 5) * 4 + 5) && (ptScreen.y < ((tft.height() / 5) * 4 + (tft.height() / 5) - 5)))
  {
    //If page has changed
    if (intActivePage != ptScreen.x / (tft.width() / 4))
    {
      //Reset sensor periodicity
      intTemp.setPeriodicity(DELAY_TEMP);
      extTemp.setPeriodicity(DELAY_TEMP);
      waterTank.setPeriodicity(DELAY_WATER);
      battVolt.setPeriodicity(DELAY_POWER);

      intActivePage = ptScreen.x / (tft.width() / 4);

      //Serial.print("\tIndex = "); Serial.println(intActivePage);

      pages[intActivePage]->refresh(NULL);
      drawMenu();


    }
  }

}

void drawMenu()
{
  int i, x, y, rectWidth, rectHeight, intTxtSize;

  //Set up variables
  String strMenu[4] = {"Main", "Power", "Temp", "Water"};
  rectWidth = (tft.width() / 4) - 10;
  rectHeight = (tft.height() / 5) - 10;
  y = (tft.height() / 5) * 4 + 5;
  intTxtSize = 1;

  //Clear bottom section
  tft.fillRect(0, (tft.height() / 5) * 4, tft.width(), tft.height(), ILI9341_WHITE);
  yield();

  //Draw divider
  tft.drawFastHLine(0, (tft.height() / 5) * 4, tft.width(), ILI9341_BLACK);

  //Draw menu buttons
  for (i = 0; i < 4; i++)
  {

    x = (tft.width() / 4) * i + 5;

    //Set Active page to different colour
    if (i == intActivePage)
    {
      tft.fillRoundRect(x, y, rectWidth, rectHeight, 4, ILI9341_GREEN);
    }
    else
    {
      tft.fillRoundRect(x, y, rectWidth, rectHeight, 4, ILI9341_BLUE);
    }

    tft.setCursor(x + (rectWidth / 2) - strMenu[i].length() * (CHAR_X / 2 * intTxtSize), y + (rectHeight / 2) - (CHAR_Y / 2 * intTxtSize));
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(intTxtSize);
    tft.print(strMenu[i]);
  }
}

