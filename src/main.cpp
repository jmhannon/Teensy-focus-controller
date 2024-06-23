// Moonlite-compatible stepper controller GNU GENERAL PUBLIC LICENSE
// https://github.com/TallFurryMan/moonduino/blob/master/Nano_Moonlite_Focuser.ino
// Uses AccelStepper (http://www.airspayce.com/mikem/arduino/AccelStepper/)
// 1	2	3	4	5	6	7	8
//:	C	#	 	 	 	 	 	 	N/A	        Initiate a temperature conversion; the conversion process takes a maximum of 750 milliseconds. The value returned by the :GT# command will not be valid until the conversion process completes.
//:	F	G	#	 	 	 	 	 	N/A	        Go to the new position as set by the ":SNYYYY#" command.
//:	F	Q	#	 	 	 	 	 	N/A	        Immediately stop any focus motor movement.
//:	G	C	#	 	 	 	 	 	XX#	        Returns the temperature coefficient where XX is a two-digit signed (2’s complement) hex number.
//:	G	D	#	 	 	 	 	 	XX#	        Returns the current stepping delay where XX is a two-digit unsigned hex number. See the :SD# command for a list of possible return values.
//:	G	H	#	 	 	 	 	 	00# OR FF#	Returns "FF#" if the focus motor is half-stepped otherwise return "00#"
//:	G	I	#	 	 	 	 	 	00# OR 01#	Returns "00#" if the focus motor is not moving, otherwise return "01#"
//:	G	N	#	 	 	 	 	 	YYYY#	        Returns the new position previously set by a ":SNYYYY" command where YYYY is a four-digit unsigned hex number.
//:	G	P	#	 	 	 	 	 	YYYY#	        Returns the current position where YYYY is a four-digit unsigned hex number.
//:	G	T	#	 	 	 	 	 	YYYY#	        Returns the current temperature where YYYY is a four-digit signed (2’s complement) hex number.
//:	G	V	#	 	 	 	 	 	DD#	        Get the version of the firmware as a two-digit decimal number where the first digit is the major version number, and the second digit is the minor version number.
//:	S	C	X	X	#	 	 	 	N/A	        Set the new temperature coefficient where XX is a two-digit, signed (2’s complement) hex number.
//:	S	D	X	X	#	 	 	 	N/A	        Set the new stepping delay where XX is a two-digit, unsigned hex number. Valid values to send are 02, 04, 08, 10 and 20, which correspond to a stepping delay of 250, 125, 63, 32 and 16 steps per second respectively.
//:	S	F	#	 	 	 	 	 	N/A	        Set full-step mode.
//:	S	H	#	 	 	 	 	 	N/A	        Set half-step mode.
//:	S	N	Y	Y	Y	Y	#	 	N/A	        Set the new position where YYYY is a four-digit unsigned hex number.
//:	S	P	Y	Y	Y	Y	#	 	N/A	        Set the current position where YYYY is a four-digit unsigned hex number.
//:	+	#	 	 	 	 	 	 	N/A	        Activate temperature compensation focusing.
//:	-	#	 	 	 	 	 	 	N/A	        Disable temperature compensation focusing.
//:	P	O	X	X	#	 	 	 	N/A	        Temperature calibration offset, XX is a two-digit signed hex number, in half degree increments.
//:     Y       M       #                                               N/A             Enhance temperature reading (0.125 degree)
//:     Y       B       X       X       #                               N/A             Set backlash where XX is a two-digit unsigned hex number
//:     Z       B       #                                               XX#             Get backlash
//:     Y       T       Y       Y       Y       Y       #               N/A             Set max steps where YYYY is a four-digit unsigned hex number
//:     Z       T       #                                               YYYY#           Get max steps
//:     Y       X       X       X       #                               N/A             Set TempComp threshold where XX is a two-digit unsigned hex number in unit of 0.25 degree
//:     Z       X       #                                               XX#             Get TempComp threshold
//:	Y       +	#	 	 	 	 	 	N/A	        Activate temperature compensation focusing.
//:	Y       -	#	 	 	 	 	 	N/A	        Disable temperature compensation focusing.
//:	Z       +	#	 	 	 	 	 	00 or 01#       Get temperature compensation.
//:	Z	A	#	 	 	 	 	 	YYYY#	        Returns the average temperature * 100 where YYYY is a four-digit signed (2’s complement) hex number.
// Example 1: :PO02# offset of +1°C
// Example 2: :POFB# offset of -2.5°C
//  added responses JMH
//: F H #                   Starts find home routine responds OK when complete
//: P F #          <string>#     Gets the current focuser type.
//: P V #          <string>#     Gets the current firmware version.
//: P S #          ddddd#        Gets the current focuser serial number.
//: S M #          N/A           Starts motor to move to the "NEW" focus position.
//: G S #          XX#           HEX bit 0 switch 1, bit 1 switch 2
//: G A #          XX#           HEX bit 0 rotation home switch, bit 1 out limit switch, bit 2 in limit switch
#include <Arduino.h>
#include <strings.h>
#include <AccelStepper.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <main.h>

#define fwversion "10"
// Speed per "SD" unit
#define SPEEDMULT 30 // base unit of stepper speed

#define TEMPSENSOR_ARRAY_SIZE 30 // array to track temperature history and to scalculate average temperatures
#define TEMPERATURE_DEFAULT 25   // default temperature

#define TEMPCOMP_THRESHOLD 1        // Temperature change threshold to trigger TempComp movement since last TempComp
#define TEMPCOMP_HYSTERESIS 1       // Hysteresis to report error, without moving focuser ???
#define TEMPCOMP_MOVEDELAY 2000     // DELAY between 2 steps druing TempComp move
#define STEPPER_DISABLEDELAY 5000   // DELAY to disable output driver after last move
#define TEMPSENSOR_READDELAY 5000   // Temperature sensor read interval if driver does not poll
#define TEMPSENSOR_SAMPLEDELAY 5000 // Temperature sample interval to calculate average temperature. For 30 samples at 5s interval will average out temperature in last 150s.

#define step_pin 14 // driver pins
#define dir_pin 19
#define step_size 18       // high is half step
#define enable_stepper 17  // high to enable
#define limit1 16          // positive step direction mirror fully in
#define limit2 15          // negative step direction mirror fully out
#define PIN_INPUT_BUT_FW 7 // Maunal movement button
#define PIN_INPUT_BUT_BW 8
#define PIN_INPUT_SENSOR 23  // Tempeature sensors
#define PIN_OUTPUT_STATUS 22 // To report error when temperature has gone up over hysteresis threshold when TempComp is on.
#define LED 13
#define LEDBLINK_INTERVAL 250 // 250ms
#define LEDBLINK_CYCLE 16     // 16*250 = 4s
///////////////////////////
// Stepper
///////////////////////////

AccelStepper stepper(AccelStepper::DRIVER, step_pin, dir_pin, false);

///////////////////////////
// Temperature Sensor
///////////////////////////

DeviceAddress TempSensor_Addr;
OneWire OneWire_TempSensor(PIN_INPUT_SENSOR);
DallasTemperature TempSensor(&OneWire_TempSensor);

///////////////////////////
// Temperature Signals
///////////////////////////

boolean TempSensor_Present = false;             // DS188B20 present
float TempSensor_Reading = TEMPERATURE_DEFAULT; // temperature reading from DS18B20
int16_t TempSensor_Raw = 0;                     // Raw temperature returned to the driver

///////////////////////////
// Serial Interface Signals
///////////////////////////

#define MAXCOMMAND 8
char inChar;
char cmd[MAXCOMMAND];
char param[MAXCOMMAND];
char packet[MAXCOMMAND];
boolean eoc = false;
int idx = 0;

///////////////////////////
// Motor Control Signals
///////////////////////////

long TargetPosition = 0;
long CurrentPosition = 0;
boolean isRunning = false;
// max/min limit when moving focuser manually.
// Max can be set via serial command YX.
long MaxSteps = 25000; // check on this ??
long MinSteps = 0;
#define acceleration 200
///////////////////////////
// Speed multipler
///////////////////////////

// multiplier of SPEEDMUX.
int SpeedFactor = 16;
int SpeedFactorRaw = 2;

///////////////////////////
// Temperature Compensation
///////////////////////////

// TemoComp coefficient is signed integer
int TempCoefficientRaw = 0;
int TempCoefficient = 0;

// TemmpComp temperature drop threshold to trigger TempComp.
// NOW temperature increase does not trigger TempComp, instead it will be reported as ERROR.
float TempCompThreshold = TEMPCOMP_THRESHOLD;
int TempCompThresholdRaw = 0;

boolean TempCompEn = false;

boolean TempCompError = false;

// TempComp original position and temeprature.
// this is to avoid losing steps, eg Coefficient*Threshold < 1, so it will not move if we only keep track of the different between 2 "regions".
// so we need to use the original temperature and position to calculate the "supposed to be" target position.
float TempCompOriginalTemperature = TEMPERATURE_DEFAULT;
long TempCompOriginalPosition = 0;
long TempCompTargetPosition = 0;
float TempCompLastTemperature = TEMPERATURE_DEFAULT;

float TempSensor_Array[TEMPSENSOR_ARRAY_SIZE];
float TempSensor_Array_Total = 0;
float TempSensor_Average = TEMPERATURE_DEFAULT;
boolean TempSensor_Valid_Array[TEMPSENSOR_ARRAY_SIZE];
int TempSensor_Valid_Total;

// Backlash, NOT being used
int Backlash = 0;

///////////////////////////
// Timer
///////////////////////////

long millisLastMove = 0;            // Last move timer to turn off stepper output
long millisLastTempSensorLatch = 0; // Last temperature sample timer
long millisLastTempSensorRead = 0;  // Last temperature sensor read timer
long millisLastTempCompMove = 0;    // Last move timer during TempComp
unsigned long microsstartloop = 0;
///////////////////////////
// Manual move control
///////////////////////////

#define BUT_READING_RELEASED 1
#define BUT_READING_PRESSED 0

int lastReadingButFW = BUT_READING_RELEASED; //
int lastReadingButBW = BUT_READING_RELEASED;

// Button press timer to increase motor move steps (ie, effective motor speed).
long millisButFWPressed = 0;
long millisButBWPressed = 0;

///////////////////////////
// EEPROM interface
///////////////////////////

#define EEPROM_POS_LOC 0
long lastSavedPosition = 0;

///////////////////////////
// LED signals
///////////////////////////

long millisLastLEDBlink = 0;
int blinkTimer = 0;

///////////////////////////
// Misc signals
///////////////////////////

// Moonlite compatability mode - 0.5 degree temparture reading accuracy
// Set to false will return 0.125 accuracy
boolean MoonliteMode = true;

int i;

void setup()
{
  Serial.begin(9600);

  pinMode(PIN_INPUT_SENSOR, INPUT);
  pinMode(PIN_INPUT_BUT_FW, INPUT_PULLUP);
  pinMode(PIN_INPUT_BUT_BW, INPUT_PULLUP);
  pinMode(PIN_OUTPUT_STATUS, OUTPUT);
  pinMode(limit1, INPUT_PULLUP); // home position
  pinMode(limit2, INPUT_PULLUP); // max position
  pinMode(step_size, OUTPUT);
  pinMode(enable_stepper, OUTPUT);
  digitalWrite(enable_stepper, HIGH);
  digitalWrite(step_size, HIGH); // always half step
  pinMode(LED, OUTPUT);

  // Initialize temperature array
  for (i = 0; i < TEMPSENSOR_ARRAY_SIZE; i++)
  {
    TempSensor_Array[i] = TEMPERATURE_DEFAULT;
    TempSensor_Valid_Array[i] = false;
  }

  // Initialize temperature sensor DS18B20
  OneWire_TempSensor.reset_search();
  OneWire_TempSensor.search(TempSensor_Addr);
  TempSensor.begin();
  if (TempSensor.getDeviceCount() != 0)
  {
    TempSensor_Present = true;
    TempSensor.setResolution(TempSensor_Addr, 11);
  }

  // read temperature to establish base temperature
  if (TempSensor_Present)
  {
    TempSensor.requestTemperatures();
    double Scratch_Double = TempSensor.getTempC(TempSensor_Addr);
    if (Scratch_Double == -127.00)
    { // Error
    }
    else
    {
      TempSensor_Reading = Scratch_Double;
      TempSensor_Raw = TempSensor.getTemp(TempSensor_Addr);
    }
  }
  millisLastTempSensorRead = millis();
  millisLastTempSensorLatch = millis();

  // initalize motor
  stepper.setMaxSpeed(SpeedFactor * SPEEDMULT);
  stepper.setAcceleration(acceleration);
  millisLastMove = millis();
  stepper.setPinsInverted(true, false, false); // invert direction pin
  stepper.setEnablePin(enable_stepper);
  // initialize serial command
  memset(packet, 0, MAXCOMMAND);

  // read saved position from
  EEPROM.get(EEPROM_POS_LOC, CurrentPosition);
  stepper.setCurrentPosition(CurrentPosition);
  // ?? keep from making a move when starting up
  stepper.moveTo(CurrentPosition);
  lastSavedPosition = CurrentPosition;
}

void loop()
{
  double Scratch_Double;
  char tempString[32];
  // process the command we got
  if (eoc)
  {
    memset(cmd, 0, MAXCOMMAND);
    memset(param, 0, MAXCOMMAND);
    // get rid of channel number
    if (packet[0] == '1' || packet[0] == '2' || packet[0] == '3' || packet[0] == '4')
    {
      memmove(packet, packet + 1, strlen(packet));
    }
    int len = strlen(packet);

    if (packet[0] == 'C' || packet[0] == '+' || packet[0] == '-')
    {
      strncpy(cmd, packet, 1);
    }
    else
    {
      strncpy(cmd, packet, 2);
      if (len > 2)
      {
        strncpy(param, packet + 2, len - 2);
      }
    }

    memset(packet, 0, MAXCOMMAND);
    eoc = false;
    idx = 0;

    // the stand-alone program sends :C# :GB# on startup
    // :C# is a temperature conversion, doesn't require any response

    // initiate temperature conversion
    if (!strcasecmp(cmd, "C"))
    {
      // do nothing
      // if (TempSensor_Present) {
      //  TempSensor.requestTemperatures();
      //}
    }

    // initiate a move - nightcrawler
    else if (!strcasecmp(cmd, "SM"))
    {
      // Ignore move when Temp Comp is enabled
      // Need to revisit as there could be MOVE due to filter change
      if (!TempCompEn)
      {
        stepper.enableOutputs();
        stepper.moveTo(TargetPosition);
        stepper.run();
      }
    }

    // stop a move - nightcrawler
    // stepper.stop() stops motor gracefully, as a result motor may continue running for sometime (upto 1000 step at max speed setting), depending the current speed.
    // if we stop the motor abruptly then somehow stepper library does not handle current/target position correctly.
    else if (!strcasecmp(cmd, "SQ"))
    {
      // Serial.println("Stop"); //debug??
      stepper.stop();
    }

    // return stepper delay motor speed this crashes the moonlite controller app ??
    // moolite app wants 4 digits spec says 2 digits
    // get the current motor speed, only values of 02, 04, 08, 10, 20, which is set by SD
    else if (!strcasecmp(cmd, "GD"))
    {
      char tempString[6];
      sprintf(tempString, "%04X", SpeedFactorRaw); // may need some tinkering of values
      Serial.print(tempString);
      Serial.print("#");
    }

    // get the temperature coefficient which is set by SC
    else if (!strcasecmp(cmd, "GC"))
    {
      sprintf(tempString, "%02X", TempCoefficientRaw);
      Serial.print(tempString);
      Serial.print("#");
    }

    // whether half-step is enabled or not, always return "00" ??
    else if (!strcasecmp(cmd, "GH"))
    {
      Serial.print("00#");
    }

    // motor is moving - 01 if moving, 00 otherwise - moonlight
    else if (!strcasecmp(cmd, "GI"))
    {
      if (isRunning)
      {
        Serial.print("01#");
      }
      else
      {
        Serial.print("00#");
      }
    }
    // get the new motor position (target) set by SN - nightcrawler
    else if (!strcasecmp(cmd, "GN"))
    {
      sprintf(tempString, "%04lX", TargetPosition);
      Serial.print(tempString);
      Serial.print("#");
    }

    // get the current motor position - nightcrawler
    else if (!strcasecmp(cmd, "GP"))
    {
      CurrentPosition = stepper.currentPosition();
      sprintf(tempString, "%04lX", CurrentPosition);
      Serial.print(tempString);
      Serial.print("#");
    }

    // get temperature
    else if (!strcasecmp(cmd, "GT"))
    {
      // Skip temperature reading when motor is running
      if (TempSensor_Present && (stepper.distanceToGo() == 0))
      {
        TempSensor.requestTemperatures();
        Scratch_Double = TempSensor.getTempC(TempSensor_Addr);
        if (Scratch_Double == -127.00)
        { // Error
        }
        else
        {
          TempSensor_Reading = Scratch_Double;
          TempSensor_Raw = TempSensor.getTemp(TempSensor_Addr) / 64; // expects temp in 1/2 degree steps
        }

        // reset temp sensor read timer. moonlite app reports 1/2 temp sent.
        millisLastTempSensorRead = millis();
        sprintf(tempString, "%04X", TempSensor_Raw); // prints 8 digits for negative 2 complement numbers??
        Serial.print(tempString);
        Serial.print("#");
      }
    }
      // firmware value, always return "10"
      else if (!strcasecmp(cmd, "GV"))
      {
        Serial.print(fwversion);
        Serial.print("#");
      }

      // set the temperature coefficient
      else if (!strcasecmp(cmd, "SC"))
      {
        TempCoefficientRaw = hexstr2long(param);
        // covert signed 8-bit to signed int
        if ((TempCoefficientRaw & 0x80))
        { // negtive
          TempCoefficient = TempCoefficientRaw - 256;
        }
        else
        {
          TempCoefficient = TempCoefficientRaw;
        }
      }

      // set speed, only acceptable values are 02, 04, 08, 10, 20 -- nightcrawler??
      else if (!strcasecmp(cmd, "SR"))
      {
        SpeedFactorRaw = hexstr2long(param);

        // SpeedFactor: smaller value means faster
        SpeedFactor = 32 / SpeedFactorRaw;
        stepper.setMaxSpeed(SpeedFactor * SPEEDMULT);
      }

      // reset compatability mode
      else if (!strcasecmp(cmd, "YM"))
      {
        MoonliteMode = false;
      }

      // set current motor position
      else if (!strcasecmp(cmd, "SP"))
      {
        CurrentPosition = strtol(param, NULL, 16);
        stepper.setCurrentPosition(CurrentPosition);
      }

      // set new motor position
      else if (!strcasecmp(cmd, "SN"))
      {
        // Ingore move command when Temp Comp is enabled
        if (!TempCompEn)
        {
          TargetPosition = strtol(param, NULL, 16);
        }
      }

      // enable TempComp ??
      else if (!strcasecmp(cmd, "Y+"))
      {
        TempCompEn = true;

        // Latch current position and average temperature.
        TempCompOriginalTemperature = TempSensor_Average;
        TempCompOriginalPosition = stepper.currentPosition();

        TempCompLastTemperature = TempSensor_Average;
        TempCompTargetPosition = TempCompOriginalPosition;
      }

      // disable TempComp, currently not used ??
      else if (!strcasecmp(cmd, "Y-"))
      {
        TempCompEn = false;
      }
      // ??
      else if (!strcasecmp(cmd, "Z+"))
      {
        if (TempCompEn)
        {
          Serial.print("01#");
        }
        else
        {
          Serial.print("00#");
        }
      }

      // LED backlight value, always return "00"
      else if (!strcasecmp(cmd, "GB"))
      {
        Serial.print("00#");
      }

      // set backlash
      else if (!strcasecmp(cmd, "YB"))
      {
        Backlash = hexstr2long(param);
      }

      // get backlash set by YB
      else if (!strcasecmp(cmd, "ZB"))
      {
        sprintf(tempString, "%02X", Backlash);
        Serial.print(tempString);
        Serial.print("#");
      }

      // set TempComp threshold in unit of 0.25 degree
      else if (!strcasecmp(cmd, "YT"))
      {
        TempCompThresholdRaw = hexstr2long(param);
        TempCompThreshold = (float)TempCompThresholdRaw / 4; // covert to degree
      }

      // get TempComp threshold set by YT
      else if (!strcasecmp(cmd, "ZT"))
      {
        sprintf(tempString, "%02X", TempCompThresholdRaw);
        Serial.print(tempString);
        Serial.print("#");
      }

      else if (!strcasecmp(cmd, "YX"))
      { // nightcrawler ??
        MaxSteps = hexstr2long(param);
      }

      else if (!strcasecmp(cmd, "ZX"))
      {
        sprintf(tempString, "%04lX", MaxSteps); // nightcrawler ??
        Serial.print(tempString);
        Serial.print("#");
      }

      else if (!strcasecmp(cmd, "ZA"))
      { // nightcrawler
        int TempInt;
        TempInt = (int)(TempSensor_Average * 100);
        if (TempInt >= 0)
        {
          TempInt = TempInt & 0xFFFF;
        }
        else
        { // convert to 2's complement
          TempInt = ~abs(TempInt) & 0xFFFF;
        }

        sprintf(tempString, "%04X", TempInt);
        Serial.print(tempString);
        Serial.print("#");
      }
      // focuser type - nightcrawler
      else if (!strcasecmp(cmd, "PF"))
      {
        Serial.print("2.5 NC");
        Serial.print("#");
      }

      // Current focuser serial number - nightcrawler
      else if (!strcasecmp(cmd, "PS"))
      {
        Serial.print("0001");
        Serial.print("#");
      }

      else if (!strcasecmp(cmd, "FH"))
      {
        find_home();
      }
      // limit switch positions -- Rigel
      else if (!strcasecmp(cmd, "GS"))
      {
        sprintf(tempString, "%02X", (digitalRead(limit1) * 4) + (digitalRead(limit2) * 2));
        Serial.print(tempString);
        Serial.print("#");
      }
      // switch positions -- currently no switches
      else if (!strcasecmp(cmd, "GA"))
      {
        // add code here
        Serial.print("00");
        Serial.print("#");
      }
      // initiate a move
      else if (!strcasecmp(cmd, "FG"))
      {
        // Ignore move when Temp Comp is enabled
        // Need to revisit as there could be MOVE due to filter change
        if (!TempCompEn)
        {
          stepper.enableOutputs();
          stepper.moveTo(TargetPosition);
        }
      }

      // Debug Info
      else if (!strcasecmp(cmd, "SS"))
      {
        debug();
      }
    
  }

  // move motor if not done check for limit switches here stop if limit hit. ??
  if (stepper.distanceToGo() != 0)
  {
    if ((digitalRead(limit1) && digitalRead(limit2)) == 1) // not at limit
    {
      isRunning = true;
      millisLastMove = millis();
      stepper.run(); // causes one step per call until destination is reached.
    }
  }
  // if motor is not moving
  else
  {
    isRunning = false;
  }

  // if a limit switch is hit
  // back off limit and stop

  if (digitalRead(limit1) == 0)
  {
    // Serial.println("At min limit");
    stepper.setCurrentPosition(0);
    long initial_homing = +1;
    stepper.enableOutputs();
    while (digitalRead(limit1) == 0) // hit min limit
    {
      stepper.move(initial_homing);
      initial_homing++;
      stepper.runSpeed();
      delay(5);
    }
    stepper.stop();
    isRunning = false;
    stepper.setCurrentPosition(0);
  }
  if (digitalRead(limit2) == 0) // hit max limit
  {
    // Serial.println("At max limit");
    stepper.setCurrentPosition(MaxSteps);
    long initial_homing = -1;
    stepper.enableOutputs();
    while (digitalRead(limit2) == 0) // at limit
    {
      stepper.move(initial_homing);
      initial_homing--;
      stepper.runSpeed();
      delay(5);
    }
    stepper.stop();
    isRunning = false;
    stepper.setCurrentPosition(MaxSteps);
  }

  // To check if motor has stopped for long time.
  if ((millis() - millisLastMove) > STEPPER_DISABLEDELAY)
  {
    millisLastMove = millis();

    // turn off driver to save power.
    stepper.disableOutputs();

    // Save current location in EEPROM
    if (lastSavedPosition != CurrentPosition)
    {
      EEPROM.put(EEPROM_POS_LOC, CurrentPosition);
      lastSavedPosition = CurrentPosition;
    }
  }

  // TempComp average temperature calculation
  // Read one sample every 5s.
  if (TempCompEn)
  {
    if (millis() - millisLastTempSensorLatch > TEMPSENSOR_SAMPLEDELAY)
    {
      millisLastTempSensorLatch = millis();

      // shift all the samples to the left - entry 0 has latest reading.
      for (i = TEMPSENSOR_ARRAY_SIZE - 1; i > 0; i--)
      {
        TempSensor_Array[i] = TempSensor_Array[i - 1];
        TempSensor_Valid_Array[i] = TempSensor_Valid_Array[i - 1];
      }
      TempSensor_Array[0] = TempSensor_Reading;
      TempSensor_Valid_Array[0] = true;

      // Calculate the average temperature
      // use Valid array to indicate whether an entry has valid data, to speed up calculation when power on.
      TempSensor_Array_Total = 0;
      TempSensor_Valid_Total = 0;
      for (i = 0; i < TEMPSENSOR_ARRAY_SIZE; i++)
      {
        if (TempSensor_Valid_Array[i])
        {
          TempSensor_Array_Total += TempSensor_Array[i];
          TempSensor_Valid_Total++;
        }
      }
      TempSensor_Average = TempSensor_Array_Total / TempSensor_Valid_Total;
    }
  }

  // Read temperature periodically if driver/app does not initiate temperature read
  if (millis() - millisLastTempSensorRead > TEMPSENSOR_READDELAY)
  {
    millisLastTempSensorRead = millis();
    // Skip temperature reading when motor is running
    if (TempSensor_Present && (stepper.distanceToGo() == 0))
    {
      TempSensor.requestTemperatures();
      Scratch_Double = TempSensor.getTempC(TempSensor_Addr);
      if (Scratch_Double == -127.00)
      { // Error
      }
      else
      {
        TempSensor_Reading = Scratch_Double;
        TempSensor_Raw = TempSensor.getTemp(TempSensor_Addr);
      }
    }
  }

  // DistanceToGo == 0

  // TempComp focuser move
  // currently it only moves focuser in one direction, after temperature has dropped more than threshold, but report error (light pin13 LED on Nano board) if temperature has gone up over the hysteresis setting.
  // I have seen that there might be temperary temperature rise by 1 degree or so but it is very rare and usually it goes back down within 30min or so, that is the reason that is does not implement "back up" function.
  if (TempCompEn)
  {
    float TempCompTempChange = TempSensor_Average - TempCompLastTemperature;

    // debug use only
    // if (abs(TempCompTempChange) > TempCompThreshold) {
    // Calculate new position when temperature changes (drops) more than threshold
    if (TempCompTempChange < -TempCompThreshold)
    {
      // TargetPosition = TempCompLastPosition + (int)((TempSensor_Average - TempCompLastTemperature) * TempCoefficient);

      TempCompLastTemperature = TempSensor_Average;
      TempCompTargetPosition = TempCompOriginalPosition + (int)((TempSensor_Average - TempCompOriginalTemperature) * TempCoefficient);

      TempCompError = false;
    }
    // report error if temperature has gone up more than Hysteresis
    // there is a LEC on  pin13
    else if (TempCompTempChange > TEMPCOMP_HYSTERESIS)
    {
      // digitalWrite(PIN_OUTPUT_ERROR, HIGH);
      TempCompError = true;
    }
    else
    {
      // digitalWrite(PIN_OUTPUT_ERROR, LOW);
      TempCompError = false;
    }

    // Move focuser one step at a time with delay of TEMPCOMP_MOVEDELAY
    // It may be ok to move all steps at once with accelstepper, but it is better to have larger delay when taking images.
    if (millis() - millisLastMove > TEMPCOMP_MOVEDELAY)
    {
      if (stepper.currentPosition() < TempCompTargetPosition)
      {
        stepper.enableOutputs();
        stepper.move(1);
      }
      if (stepper.currentPosition() > TempCompTargetPosition)
      {
        stepper.enableOutputs();
        stepper.move(-1);
      }
    }
  }

  // disable manual movement when Temp Comp is enabled
  else
  { // TempCompEn
    // forward move
    if (digitalRead(PIN_INPUT_BUT_FW) == BUT_READING_RELEASED) // max direction black button
    {
      if (lastReadingButFW == BUT_READING_PRESSED)
      {
        stepper.stop();
      }
      lastReadingButFW = BUT_READING_RELEASED;
    }
    else
    {
      if (lastReadingButFW == BUT_READING_RELEASED)
      {
        stepper.enableOutputs();
        millisButFWPressed = millis();
      }
      // To not run over end stop. // add test limit switch and back off if hit
      // long NewStep = min(pow(10, min(2, (int)((millis() - millisButFWPressed) / 1000))) * 10, MaxSteps - stepper.currentPosition());
      long NewStep = MaxSteps - stepper.currentPosition();
      stepper.move(NewStep);
      millisLastMove = millis();
      lastReadingButFW = BUT_READING_PRESSED;
    }

    // backward moves
    if (digitalRead(PIN_INPUT_BUT_BW) == BUT_READING_RELEASED)
    {
      if (lastReadingButBW == BUT_READING_PRESSED)
      {
        stepper.stop();
      }
      lastReadingButBW = BUT_READING_RELEASED;
    }
    else
    {
      if (lastReadingButBW == BUT_READING_RELEASED) // home direction red button
      {
        stepper.enableOutputs();
        millisButBWPressed = millis();
      }
      // change this so that stepper keeps going until it hits limit switch and then back off
      // this will allow the button to be used to home the focuser.
      // To not run into end stop. // add test limit switches and back off if hit
      // long NewStep = min(pow(10, min(2, (int)((millis() - millisButBWPressed) / 1000))) * 10, stepper.currentPosition());
      long NewStep = stepper.currentPosition();
      stepper.move(-NewStep);
      millisLastMove = millis();
      lastReadingButBW = BUT_READING_PRESSED;
    }
  } // TempCompEn
} // end loop

// Blink LED for status:
// blink 0.25s lit every 4s: Gathering temperature
// blink 1s lit every 4s: Average temperature acquired
// blink 2s lit every 4s: TempComp Enabled
// flashing: TempComp Error

// read the command until the terminating # character
void serialEvent()
{
  while (Serial.available() && !eoc)
  {
    inChar = Serial.read();
    if (inChar != '#' && inChar != ':')
    {
      packet[idx++] = inChar;
      if (idx >= MAXCOMMAND)
      {
        idx = MAXCOMMAND - 1;
      }
    }
    else
    {
      if (inChar == '#')
      {
        eoc = true;
      }
    }
  }
}

long hexstr2long(char *line)
{
  long ret = 0;

  ret = strtol(line, NULL, 16);
  return (ret);
}

void find_home()
{
  long initial_homing = -1;
  stepper.enableOutputs();
  while (digitalRead(limit1) == 1)
  {
    stepper.move(initial_homing);
    initial_homing--;
    stepper.runSpeed();
  }
  // if you don't set current position to 0 here the back off runs for a long time without
  // doing anything
  stepper.setCurrentPosition(0);
  // back off until home switch opens
  initial_homing = 1;
  stepper.enableOutputs();
  while (digitalRead(limit1) == 0)
  {
    stepper.move(initial_homing);
    initial_homing++;
    stepper.runSpeed();
    // seems to need a delay here to see the limit switch setting properly??
    delay(5);
  }
  stepper.setCurrentPosition(0);
}
void debug()
{
  Serial.print("Temperature Sensor Present: ");
  Serial.print(TempSensor_Present);
  Serial.println("");

  Serial.print("Temperature Coefficient: ");
  Serial.print(TempCoefficient);
  Serial.println("");

  if (TempSensor_Present)
  {
    Serial.print("Temperature: ");
    Serial.print(TempSensor_Reading);
    Serial.println("");
  }

  for (i = 0; i < TEMPSENSOR_ARRAY_SIZE; i++)
  {
    Serial.println(TempSensor_Valid_Array[i]);
    Serial.println(TempSensor_Array[i]);
    Serial.println("");
  }
  Serial.print("Temperature Average ");
  Serial.print(TempSensor_Average);
  Serial.println("");

  Serial.print("TempComp Last Temperature ");
  Serial.print(TempCompLastTemperature);
  Serial.println("");

  Serial.print("TempComp Original Temperature ");
  Serial.print(TempCompOriginalTemperature);
  Serial.println("");

  // Serial.print("TempComp Orignial Position ");
  // Serial.print(TempCompOriginalPosition);
  // Serial.println("");

  // Serial.print("TempComp Target Position");
  // Serial.print(TempCompTargetPosition);
  // Serial.println("");

  // Serial.print("Last Position ");
  // Serial.print(TempCompLastPosition);
  // Serial.println("");

  Serial.print("Stepper current Position ");
  Serial.print(stepper.currentPosition());
  Serial.println("");

  Serial.print("FW Button Status ");
  Serial.print(digitalRead(PIN_INPUT_BUT_FW));
  Serial.println("");

  Serial.print("BW Button Status ");
  Serial.print(digitalRead(PIN_INPUT_BUT_BW));
  Serial.println("");

  Serial.print("Stepper Distance to go ");
  Serial.print(stepper.distanceToGo());
  Serial.println("");
  Serial.print("Limit1 Home Status ");
  Serial.print(digitalRead(limit1));
  Serial.println("");

  Serial.print("Limit2 max Status ");
  Serial.print(digitalRead(limit2));
  Serial.println("");
  Serial.print("Stepper Target Position ");
  Serial.print(stepper.targetPosition());
  Serial.println("");
  Serial.print("Target Position ");
  Serial.print(TargetPosition);
  Serial.println("");
}