/*
 * 4 input pins for water, fan, etc. should be pulsing > 5/sec < 20/sec
 * 1 input pin from Main Controller, high when wanting Air Assist - output 1 from AWC608 low run air assist, high stop air assist
 * 1 analog pin for water temperature
 * 
 * 1 Seeeduino XIAO SAMD21 board 
 * 
 * 1 Nextion display (NX4827T043) connected via serial port - need 1K resistor inline with Tx from Nextion
 *    Shows - water temp in C and F
 *    Shows - speeds of water/fan/etc. sensors
 *    Shows 4 buttons, see above
 *    Shows state of Enable/Disable laser firing
 * 
 * 4 Buttons on Nextion display
 *    Turn on/off Water Cooling
 *    Turn on/off Exhaust fan
 *    Turn on/off Air Assist pump
 *    Turn on/off internal light
 *    
 * 6 Relays connected via I2C to 2 boards (0x8, 0x6D) (Sparkfun KIT-16833 and COM-16566)
 *    R1: Turn on/off Water Cooling   - Connected to button, 110V AC
 *    R2: Turn on/off Exhaust fan     - Connected to button, 110V AC
 *    R3: Turn on/off Air Assist pump - Connected to button, 110V AC
 *    R4: Not used
 *    R5: Turn on/off internal lights - Connected to button, 12V DC
 *    R6: Open/close Air Assist valve - Open when pump off or pump on and main controller wants air assist, 24V DC
 *    R7: Enable/Disable laser firing - Disable if Temp under/over and water/fan/etc pulse rate is under/over, 24V DC
 *    R8: Not used
 *
 *  D0-3  pulse input pins      Input from water flow, fan, etc sensors (0-200pps)
 *  D4,5  I2C                   Output to relay boards
 *  D6,7  Serial1               I/O to NexTion Display
 *  D8    Debug mode            Enable debug if low
 *  D9    Air assist input      Input from Main Controller output 1 (low active)
 *  A10   Temp input            Input from Thermistor Temperature sensor via eblock
 *  D13   Error state led       Output internal board LED used to show error state
 *  
 */
#include <Nextion.h>
#include "NexButton.h"
#include "NexText.h"

#include <Wire.h>
#include <SparkFun_Qwiic_Relay.h>

bool debugon = false;                                                                     // debug off by default
#define DEBUGON_PIN   8                                                                   // if pin low, debugon is set to true
#define DEBUGDELAY    1000                                                                // delay between debug outputs
/*
 * pulse in limits
 */
#define PULSE100      100
#define MAXPULSEFREQ  (50*PULSE100)                                                       // * 100    50/sec
#define MINPULSEFREQ  (5*PULSE100)                                                        // * 100     5/sec

/*
 * temperature limits
 */
#define TEMP10        10
#define MAXTEMP       (27*TEMP10)                                                         // * 10     27C
#define MINTEMP       (10*TEMP10)                                                         // * 10     10C

#define TEMPLOWPASSFILTER   8
#define PULSELOWPASSFILTER  4
/* 
 *  digital input and temperature input values 
 */
bool AirAssistValveInput = true;
short intemp[2];
volatile unsigned short pulseperiod[4];

/* 
 *  last UI values - save updating UI except when needed 
 */
unsigned short oldpulseperiod[4];
short oldintemp[2];
bool oldCurrent_Cool_State;

/* 
 *  Controller state 
 */
bool Air_Valve;                                                                           // Air Assist valve on/off
bool Current_Cool_State;                                                                  // Laser disabled state on/off

/*
 * LED to show error state on initialization
 */
#define LED_PIN                 PIN_LED                                                   // led error status pin
#define LED_OFF                 HIGH
#define LED_ON                  LOW

void errorState() { digitalWrite(LED_PIN, LED_ON); }
void clearState() { digitalWrite(LED_PIN, LED_OFF); }

/* 
 *  local sensor information 
 */
#define  TIME_PIN0               0                                                        // pulse in pins
#define  TIME_PIN1               1
#define  TIME_PIN2               2
#define  TIME_PIN3               3
#define  INPUT_PIN               9                                                        // digital input pin
#define  TEMP_PIN                A10                                                      // eblock/temp in pin

#define ANALOGRESOLUTION  ADC_RESOLUTION                                                  // ADC resolution
#define ANALOGRANGEMAX    ((1<<ANALOGRESOLUTION)-1)                                       // max value of ADC

const unsigned long debounceDelay = 5000;                                                 // the debounce time (us), max freq 100/sec
const unsigned long stoppedDelay = 1000000;                                               // after 1sec without any pin state changes, set to 0 pulses/sec
const byte pinpin[] = { TIME_PIN0, TIME_PIN1, TIME_PIN2, TIME_PIN3, INPUT_PIN };

#define PINPINSIZE              (sizeof(pinpin)/sizeof(pinpin[0]))
#define AIRASSISTPIN            4                                                         // pinstate index for Air Assist pin state
                                                                                          // these are read/updated in interrupt handlers
volatile byte pinstate[PINPINSIZE];                                                       // debounced pin state
volatile unsigned long debtime[PINPINSIZE];                                               // time of last pin change for debounce
volatile unsigned long risetime[PINPINSIZE];                                              // time of last debounced rising edge
volatile unsigned long averriseperiodtemp[PINPINSIZE];                                    // time of last rising to rising edge period

/*
 *  Display information
 */
bool displayavailable = false;                                                            // whether we found display correctly
const int displayStartupDelay = 500;                                                      // delay for display to initialize

#define RED     63488                                                                     // UI colors
#define GREEN    2016
#define WHITE   65535
#define GREY    50712
                                                                                          // UI elements
NexButton CoolOn = NexButton(0, 6, "CoolOn");                                             // Buttons to enable/disable water/exhaust/etc.
NexButton ExhaustOn = NexButton(0, 7, "ExhaustOn");
NexButton AirOn = NexButton(0, 8, "AirOn");
NexButton LightsOn = NexButton(0, 9, "LightsOn");
NexText c1 = NexText(0, 2, "c1");                                                         // water/pump/etc. pulses/sec
NexText c2 = NexText(0, 3, "c2");
NexText c3 = NexText(0, 4, "c3");
NexText c4 = NexText(0, 5, "c4");
NexText TempF = NexText(0, 1, "TempF");                                                   // water temperature
NexText TempC = NexText(0, 13, "TempC");
NexText CoolingOK = NexText(0, 10, "CoolingOK");                                          // Laser enable/disabled
NexText tempTextF = NexText(0, 11, "tempTextF");                                          // static text
NexText tempTextC = NexText(0, 12, "tempTextC");

char buffer[20];                                                                          // int to text buffer used to update text in display

NexTouch *nex_listen_list[] =                                                             // UI buttons for callbacks
{
    &CoolOn,
    &ExhaustOn,
    &AirOn,
    &LightsOn,
    NULL
};

/*
 *  Relays information
 */
#define I2CCLOCK              400000                                                      // 400K clock

#define RELAY_NONE            0                                                           // used if no relay for a button

#define RELAY_COOL            RELAY_ONE                                                   // relay layout - board 1 - solid state relays
#define RELAY_EXHAUST         RELAY_TWO
#define RELAY_AIRASSISTPUMP   RELAY_THREE
#define RELAY_UNUSED_1_1      RELAY_FOUR

#define RELAY_LIGHTS          (RELAY_FOUR+RELAY_ONE)                                      // relay layout - board 2 - normal relays
#define RELAY_AIRASSISTVALVE  (RELAY_FOUR+RELAY_TWO)
#define RELAY_DISABLELASER    (RELAY_FOUR+RELAY_THREE)
#define RELAY_UNUSED_2_1      (RELAY_FOUR+RELAY_FOUR)

Qwiic_Relay quadRelay1(QUAD_SSR_DEFAULT_ADDRESS);                                         // relay slave addresses - quad solid state relays 28-380 VAC 40A
Qwiic_Relay quadRelay2(QUAD_DEFAULT_ADDRESS);                                             // relay slave addresses - quad electromagnet relays 240VAC/30VDC 5A

struct {
  bool relayavailable;                                                                    // we found the relay board during initialization
  Qwiic_Relay *relay;
  bool Relaystate[4];                                                                     // current relay state on/off
} relays[2] = { {false, &quadRelay1, {false, false, false, false}},
                {false, &quadRelay2, {false, false, false, false}}};

#define GETBOARD(r) ((r-1)/4)                                                             // relay no to relay board
#define GETRELAY(r) (((r-1)%4)+1)                                                         // relay no to relay on board

void SetRelay(int relay, bool value)
{
  int b = GETBOARD(relay);                                                                // which relay board
  int r = GETRELAY(relay)-1;                                                              // which relay index (1 less than relay number on board)
  if(relay && relays[b].relayavailable && (value != relays[b].Relaystate[r]))             // relay and relay board available and is the relay state changing
  {
    relays[b].relay->toggleRelay(r+1);                                                    // since we know the relay state, toggle the relay when we want to change it
    relays[b].Relaystate[r] = value;                                                      // update internal state to match
    if(debugon) Serial.printf("SetRelay %d %d %s\n", relay, relays[b].Relaystate[r], value?"ON":"OFF");
  }
}

#define COOLBUTTON      0                                                                 // button indexes
#define EXHAUSTBUTTON   1
#define AIRASSISTBUTTON 2
#define LIGHTSBUTTON    3

#define BUTTONSTATESIZE   (sizeof(ButtonState)/sizeof(ButtonState[0]))
#define BUTTONINDEX(p)    (p-ButtonState)
/*
 *  Button callback handlers
 */
struct _ButtonState
{
  bool state;
  bool oldstate;
  int relay;
  bool relaystate;
  NexButton *Button;
} ButtonState[] = { { false, true, RELAY_COOL, false, &CoolOn },
                    { false, true, RELAY_EXHAUST, false, &ExhaustOn },
                    { false, true, RELAY_AIRASSISTPUMP, false, &AirOn },
                    { false, true, RELAY_LIGHTS, false, &LightsOn } };

void ButtonCallback(void *p)
{
  struct _ButtonState *ptr = (struct _ButtonState *)p;
  ptr->state = !ptr->state;                                                               // toggle button state
  if(debugon) Serial.printf("Button %d %s\n", BUTTONINDEX(ptr), ptr->state?"On":"Off"); 
}

/*
 *      GRE    GFE    GRE    BFE  BRE GFE   BRE BFE GRE           Good/Bad Rising/Falling Edge
 * c    1CDRU  0CDU   1CDRU  0C   1C  0CDU  1C  0C  1CDRU         cstate, Curtime, DebTime, Rising/Rising Period, Update Debtime
 * p    0   1  1  0   0   1  1    1   1  0  0   0   0   1         pinstate
*/
/*
 *  Pulse input and air assist input pin interrupt handlers
 */
void dointerrupt(byte p)
{
  byte cstate = digitalRead(pinpin[p]);                                                   // current input value
  unsigned long curtime = micros();                                                       // current time
  if((pinstate[p] != cstate) && ((curtime - debtime[p]) > debounceDelay))                 // pin change, ignore changes unless debounce period from last change
  {                                                                                       // will ignore frequencies > 1/debounce
    if((cstate == HIGH) && (p != AIRASSISTPIN))                                           // pulse input is high calculate period of time from last rising edge
    {
      averriseperiodtemp[p] = (averriseperiodtemp[p]*(PULSELOWPASSFILTER-1) + (curtime - risetime[p]))/PULSELOWPASSFILTER;
      risetime[p] = curtime;                                                              // time of this rising edge for next time
      if(averriseperiodtemp[p])
        pulseperiod[p] = (1000000L*PULSE100)/averriseperiodtemp[p];                       // calculate pulses/sec * 100
    }
    debtime[p] = curtime;                                                                 // save time for debouncing next change - should be > debounce period from this change
    pinstate[p] = cstate;                                                                 // save pin state
  }
}

void edgeinterrupt0() { dointerrupt(0); }                                                 // Interrupt handlers on all the input pins, all go to the same handler
void edgeinterrupt1() { dointerrupt(1); }
void edgeinterrupt2() { dointerrupt(2); }
void edgeinterrupt3() { dointerrupt(3); }
void edgeinterrupti() { dointerrupt(AIRASSISTPIN); }                                      // we skip the riseperiod calculations for this interrupt

/*
 *  initialize debug output
 */
void setupDebug()
{
  Serial.begin(115200);                                                                   // serial debug
  pinMode(DEBUGON_PIN,INPUT_PULLUP);                                                      // input pin to override debug mode
  debugon = !digitalRead(DEBUGON_PIN);                                                    // if low make sure debug mode is on, otherwise the default in the code
  if(debugon)
  {
    while(!Serial);                                                                       // wait until we have a monitor on the usb serial port
    Serial.printf("Debug setup\n");
  }
}

/*
 * Initialize the error state LED
 */
void setupLED()
{
  pinMode(LED_PIN,OUTPUT);                                                                // setup LED pin
  digitalWrite(LED_PIN, LED_OFF);                                                         // LED off
}

/*
 * Initialize the display and button callbacks
 */
void setupDisplay()
{
  if(debugon) Serial.printf("SetupDisplay\n");
  displayavailable = true;                                                                // assume we will find the display
  delay(displayStartupDelay);                                                             // delay to allow display to initialize
  if(!nexInit(115200))                                                                    // initialize connection to display, using a modified nextion library
  {                                                                                       // sometimes the 1st command to the display fails
    if(debugon) Serial.printf("Display1\n");
    nexSerial.end();
    if(!(displayavailable = nexInit(115200)))                                             // retry init display
    {
      if(debugon) Serial.printf("Check connections to Nextion Display\n");
      errorState();                                                                       // set the error led, we have a problem
    }
  }
  for(int i = 0; i < BUTTONSTATESIZE; i++)
    ButtonState[i].Button->attachPop(ButtonCallback, &ButtonState[i]);                    // configure button callbacks
}

/*
 * Initialize the I2C bus to talk to the relay cards
 */
void setupI2C()
{
  if(debugon) Serial.printf("SetupI2C\n");
  Wire.begin();                                                                           // init I2C as master
  Wire.setClock(I2CCLOCK);                                                                // change clock speed to 400K
}

/*
 * Initialize the pulse inputs, air assist input and temp input pins
 */
void (* const int_table[])() = { edgeinterrupt0, edgeinterrupt1, edgeinterrupt2, edgeinterrupt3, edgeinterrupti };

void setupSensors()
{
  if(debugon) Serial.printf("SetupSensors\n");

  analogReadResolution(ANALOGRESOLUTION);                                                 // setup temp analog pin

  for(int i = 0; i < PINPINSIZE; i++)
  {
    pinMode(pinpin[i], INPUT_PULLUP);                                                     // pulse in pins and air assist input pin
    pinstate[i] = digitalRead(pinpin[i]);                                                 // init pin state
    debtime[i] = risetime[i] = micros();
    attachInterrupt(digitalPinToInterrupt(pinpin[i]), int_table[i], CHANGE);              // with interrupts
  }
}

void initRelay(int r)
{
  if(!(relays[r].relayavailable = relays[r].relay->begin()))                              // init relay board and check its available
  {
    if(debugon) Serial.printf("Check connections to Qwiic Relay%d\n", r+1);
    errorState();                                                                         // set the error led, we failed to find one of the relay boards
  }
  else
  {
    relays[r].relay->turnAllRelaysOff();                                                  // all off on the board
    relays[r].Relaystate[0] = !!relays[r].relay->getState(RELAY_ONE);                     // init our relay state, should be off but get it from relay board anyway
    relays[r].Relaystate[1] = !!relays[r].relay->getState(RELAY_TWO);
    relays[r].Relaystate[2] = !!relays[r].relay->getState(RELAY_THREE);
    relays[r].Relaystate[3] = !!relays[r].relay->getState(RELAY_FOUR);
  }
}

/*
 * Initialize the 2 relay boards
 */
void setupRelays()
{
  if(debugon) Serial.printf("SetupRelays\n");
  initRelay(0);
  initRelay(1);
}

void setup()
{
  setupDebug();                                                                           // check if debug is on and init debug serial port

  setupLED();                                                                             // error led

  setupDisplay();                                                                         // display

  setupI2C();                                                                             // I2C bus for relay boards

  setupSensors();                                                                         // sensors

  setupRelays();                                                                          // relays
}

#define NOLOOPTIME    8
unsigned long looptime[NOLOOPTIME];
#define MAXLOOPTIME     15000                                                             // max loop time in microseconds
#define SKIPCOUNT       10                                                                // lots of initialization the first few times round loop
/*
 * Monitor timing of main loop and error if too large. Loop times are < 7-8ms and most are < 1ms
 *  Most of the delay is when updating the display, non display times < 1ms
 */
void MonitorTimingLoop()
{
  static int startc = SKIPCOUNT, maxcount[NOLOOPTIME];
  static unsigned long avertime[NOLOOPTIME], mintime[NOLOOPTIME] = {0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff}, maxtime[NOLOOPTIME];

  if(startc)                                                                              // skip average/min/max during initialization
  {
    if(!(--startc)) avertime[0] = looptime[0];
  }
  else
  {
    for(int i = 0; i < NOLOOPTIME; i++)
    {
      avertime[i] = (avertime[i] + looptime[i])/2;                                        // track average, min and max times for parts of loop
      if(looptime[i] < mintime[i]) mintime[i] = looptime[i];
      if(looptime[i] > maxtime[i]) maxtime[i] = looptime[i];
      if(looptime[i] > MAXLOOPTIME) maxcount[i]++;
    }

    if(debugon)
    {
      static unsigned long lastdistime = 0;
      if((millis() - lastdistime) > DEBUGDELAY)
      {
        lastdistime = millis();
        Serial.printf("Loop times%s: aver %lu, min %lu, max %lu maxcount %d\n",
          (maxtime[0] > MAXLOOPTIME)?" too large":"", avertime[0], mintime[0], maxtime[0], maxcount[0]);
        for(int i =1; i < NOLOOPTIME; i++)
          if(maxtime[i] > MAXLOOPTIME)
            Serial.printf("loop times%d: aver %lu, min %lu, max %lu maxcount %d\n", i, avertime[i], mintime[i], maxtime[i], maxcount[i]);
      }
    }
  }
}

/*
 *  get sensor values from pulse input sensor, air assist input sensor and temp sensor and translate them into usable values
 */
void UpdatePinInputs()
{
  static unsigned long distime;

  for(int i = 0;i < sizeof(pulseperiod)/sizeof(pulseperiod[0]);i++)
  {
    unsigned long d = debtime[i];
    unsigned long m = micros();
    unsigned long delta = m - d;                                                          // read into local variables in this order so make sure debtime isn't updated after micros() is called
    if((delta > stoppedDelay) && (delta < 2*stoppedDelay))                                // if its been over a sec since last change, reset pulseperiod
    {
      if(debugon && pulseperiod[i])
        Serial.printf("reset pulse %d %d %lu %lu %lu %d\n", i, pinstate[i], m, d, averriseperiodtemp[i], pulseperiod[i]);
      pulseperiod[i] = 0;                                                                 // reset pulseperiod
    }
  }
  AirAssistValveInput = !pinstate[AIRASSISTPIN];                                          // inverted debounced air assist input pin state

  if(debugon && ((millis() - distime) > DEBUGDELAY))
  {
    static bool aa;
    static short pp[4];
    if(((aa != AirAssistValveInput) || (pp[0] != pulseperiod[0]) || (pp[1] != pulseperiod[1]) || (pp[2] != pulseperiod[2]) || (pp[3] != pulseperiod[3])))
    {
      Serial.printf("Sensor %d %d %d %d %d\n", pulseperiod[0], pulseperiod[1], pulseperiod[2], pulseperiod[3], AirAssistValveInput);
      aa = AirAssistValveInput; pp[0] = pulseperiod[0]; pp[1] = pulseperiod[1]; pp[2] = pulseperiod[2]; pp[3] = pulseperiod[3];
      distime = millis();
    }
  }
}

#define MINEBLOCKTEMP   (-22*TEMP10)
#define MAXEBLOCKTEMP   (56*TEMP10)

void UpdateTempInputs()
{
  static unsigned long distime;

  short Currenttemp = (Currenttemp*(TEMPLOWPASSFILTER-1) + analogRead(TEMP_PIN))/TEMPLOWPASSFILTER; // low pass on temp
  intemp[1] = map(Currenttemp, 0, ANALOGRANGEMAX, MINEBLOCKTEMP, MAXEBLOCKTEMP);          // linear map of eblock/ADC to temp C * 10
  intemp[0] = (intemp[1] * 9)/ 5 + 32*TEMP10;                                             // standard translation of C to F, except temp * 10

  if(debugon && ((millis() - distime) > DEBUGDELAY))
  {
    static short T[2];
    distime = millis();
    if((T[0] != intemp[0]) || (T[1] != intemp[1]))
    {
      Serial.printf("Temp Sensor %d %d\n", intemp[1], intemp[0]);
      T[0] = intemp[0]; T[1] = intemp[1];
    }
  }
}

#define TEMPOUTOFRANGE           ((intemp[1] < MINTEMP) || (intemp[1] > MAXTEMP))
#define PULSEOUTOFRANGE(i)       ((pulseperiod[i] < MINPULSEFREQ) || (pulseperiod[i] > MAXPULSEFREQ))
/*
 *  Check whether we are cooling laser correctly and disable laser if there is a problem
 */
void ValidateCoolingInputs()
{
                                                                                          // if one of the cooling fans, pump, water flow sensor is bad
                                                                                          // or water temp is out of range we have a problem -> disable laser water protect
  bool newState = (TEMPOUTOFRANGE || PULSEOUTOFRANGE(0) || PULSEOUTOFRANGE(1) || PULSEOUTOFRANGE(2) || PULSEOUTOFRANGE(3));

  if (Current_Cool_State != newState)                                                     // cooling state has changed
  {
    Current_Cool_State = newState;
    if(debugon) Serial.printf("Laser %s\n", Current_Cool_State?"Disabled":"Enabled");
    SetRelay(RELAY_DISABLELASER, Current_Cool_State);                                     // update whether laser enabled or disabled
  }
}

/*
 * Calculate air assist valve state
 */
void UpdateAirAssistValve()
{
  bool newState = ButtonState[AIRASSISTBUTTON].state?AirAssistValveInput:true;            // If AirAssist pump is on then follow AirAssist input otherwise valve should be open
  if(Air_Valve != newState)                                                               // Air valve state needs to change
  {
    Air_Valve = newState;
    if(debugon) Serial.printf("Air Assist Valve %s\n", Air_Valve?"Open":"Closed");
    SetRelay(RELAY_AIRASSISTVALVE, Air_Valve);                                            // update Air assist valve relay
  }
}

/*
 * Update relay state to match button state
 */
void UpdateRelayState()
{
  for(int i = 0; i < BUTTONSTATESIZE; i++)
    if(ButtonState[i].relaystate != ButtonState[i].state)                                 // need to change relay state
    {
      ButtonState[i].relaystate = ButtonState[i].state;
      SetRelay(ButtonState[i].relay, ButtonState[i].relaystate);                          // SetRelay checks a relays current state and changes it if required, 0 is no relay
    }
}

#define UpdateDelay   1000
#define ERRORBIT(x)   (1<<(x))
unsigned long errorbits[] = {ERRORBIT(0), ERRORBIT(2), ERRORBIT(4), ERRORBIT(6), ERRORBIT(8), ERRORBIT(10), ERRORBIT(12), ERRORBIT(13), ERRORBIT(14), ERRORBIT(15), ERRORBIT(16) };
char *ts[] = { "TempF", "TempC", "Pulse 0", "Pulse 1", "Pulse 2", "Pulse 3" };
NexText *Text[] = { &TempF, &TempC, &c1, &c2, &c3, &c4 };
/*
 *  Updating Display routines
 */
bool UpdateTPDisplay(int index, unsigned long sret, unsigned long *rets)
{
  static unsigned int lasttime[6] = {0, 0, 0, 0, 0, 0};
  bool ret = false;
  bool color;
  if((millis() - lasttime[index]) > UpdateDelay)                                          // slow down updates
  {
    if(index < 2)
    {
      ret = (oldintemp[index] != intemp[index]);
      sprintf(buffer, "%d.%d", intemp[index]/TEMP10, abs(intemp[index]%TEMP10));          // format for temperature
      color = TEMPOUTOFRANGE;
    }
    else
    {
      ret = (oldpulseperiod[index-2] != pulseperiod[index-2]);
      if(pulseperiod[index-2])
        sprintf(buffer, "%d.%d", pulseperiod[index-2]/PULSE100, pulseperiod[index-2]%PULSE100); // format for pulses
      else
        strcpy(buffer, "stopped");                                                        // no pules
      color = PULSEOUTOFRANGE(index-2);
    }
    if(ret)
    {
      if(displayavailable)
      {
        if(!Text[index]->setText(buffer)) *rets |= sret;
        if(!Text[index]->Set_background_color_bco(color?RED:WHITE)) *rets |= (sret<<1);
      }
      if(!*rets)
      {
        if(index < 2)
          oldintemp[index] = intemp[index];                                               // save state if updated display temp
        else
          oldpulseperiod[index-2] = pulseperiod[index-2];                                 // save state if updated display pulse
        lasttime[index] = millis();
      }
      if(debugon) Serial.printf("Display %s '%s' %lx\n", ts[index], buffer, *rets);
    }
    else
      lasttime[index] = millis();
  }
  return ret;
}

unsigned long UpdateCoolDisplay(unsigned long sret, unsigned long *rets)
{
  bool ret = false;
  if(ret = (oldCurrent_Cool_State != Current_Cool_State))
  {
    if(displayavailable)
      if(!CoolingOK.Set_background_color_bco((Current_Cool_State == true)?RED:GREEN)) *rets |= sret;
    if(!*rets)
      oldCurrent_Cool_State = Current_Cool_State;                                         // save state if updated display
    if(debugon) Serial.printf("Display Cooling state %d %lx\n", Current_Cool_State, *rets);
  }
  return ret;
}

unsigned long UpdateDisplayButton(int index, unsigned long sret, unsigned long *rets)
{
  bool ret = false;

  if(ret = (ButtonState[index].oldstate != ButtonState[index].state))
  {
    if(displayavailable)
      if(!ButtonState[index].Button->Set_background_color_bco(ButtonState[index].state?GREEN:GREY)) *rets |= sret;
    if(!*rets)
      ButtonState[index].oldstate = ButtonState[index].state;                             // save state if updated display
    if(debugon) Serial.printf("Button %d state %d %lx\n", index, ButtonState[index].state, *rets);
  }  
  return ret;
}

/*
 *  Make sure display reflects the state of the controller 
 *    Cycle round updating different parts of the UI but stop once updated 1 part or none to update
 *    Dont update state unless display acks correctly, will retry later if state hasn't been updated
 */
void UpdateDisplay()
{
  static int part = 0;                                                                    // current UI part we want to update
  unsigned long rets = 0;                                                                 // error state from updating UI
  int startpart = part;                                                                   // which UI part this call started from
  bool updateddisplay = false;                                                            // UI part updated

  do
  {
    switch(part)                                                                          // break display updates up, 1 on each time round main loop
    {                                                                                     // keeps loop time down and doesn't flood display with too many commands
    case 0: case 1: case 2: case 3: case 4: case 5:
      updateddisplay = UpdateTPDisplay(part, errorbits[part], &rets);                     // update temp/pulse if changed
      break;
    case 6:
      updateddisplay = UpdateCoolDisplay(errorbits[part], &rets);                         // update cooling state if changed
      break;
    case 7: case 8: case 9: case 10:
      updateddisplay = UpdateDisplayButton(part-7, errorbits[part], &rets);               // update display buttons if changed
      break;
    }
    ++part %= 11;
  } while(!updateddisplay && (startpart != part));                                        // loop until we have updated 1 UI part or tried them all
  if(rets && debugon) Serial.printf("Display update failures %lx\n", rets);
}

#define SensorDelay   100

#define STARTTIME(x)  temptime[x] = micros()
#define ENDTIME(i,x)  looptime[i] = micros() - temptime[x]

void loop()
{
  MonitorTimingLoop();                                                                    // Monitor speed of this loop
  
  static unsigned long lastSensorTime;
  unsigned long temptime[2];

  STARTTIME(0); STARTTIME(1);
  if((millis() - lastSensorTime) > SensorDelay)                                           // update sensors and controller state every SensorDelay milliseconds
  {
    UpdatePinInputs();                                                                    // Update digital pin inputs
    ENDTIME(1,1); STARTTIME(1);
    UpdateTempInputs();                                                                   // Update temp input
    ENDTIME(2,1); STARTTIME(1);
    
    ValidateCoolingInputs();                                                              // Check if we have a cooling problem
    ENDTIME(3,1); STARTTIME(1);
    UpdateAirAssistValve();                                                               // Enable air assist valve if requested
    ENDTIME(4,1); STARTTIME(1);
    UpdateRelayState();                                                                   // Update relay state for buttons
    ENDTIME(5,1); STARTTIME(1);
  }

  nexLoop(nex_listen_list);                                                               // Get any button events from display
  ENDTIME(6,1); STARTTIME(1);

  UpdateDisplay();                                                                        // Update display for current state
  ENDTIME(7,1);
  ENDTIME(0,0);
}
