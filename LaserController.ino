/*
 * 4 input pins for water, fan, etc. should be pulsing > 5/sec < 50/sec
 * 1 input pin from Main Controller, high when wanting Air Assist - output 1 from AWC608 low run air assist, high stop air assist
 * 1 analog pin for water temperature
 * 
 * 1 Seeeduino XIAO SAMD21 board 256K flash, 32K RAM
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
 *  A10   Temperature input     Input from Thermistor Temperature sensor via eblock
 *  D13   Error state led       Output internal board LED used to show error state
 *  
 *  Uses modified Nextion library - adds setting background color and makes serial I/O better
 *  Uses modified wiring_analog.c to fix known issue with SAMD21 analog read - not fixed in Seeedstudio version
 *  
 */
#include "NexButton.h"
#include "NexText.h"

#include <SparkFun_Qwiic_Relay.h>

/*
 * pulse in limits
 */
#define PULSE100      100                                                                 // pulses is stored as *100
#define MAXPULSEFREQ  (50*PULSE100)                                                       // * 100    50/sec  - top of pps range
#define MINPULSEFREQ  (5*PULSE100)                                                        // * 100     5/sec  - bottom of pps range

/*
 * temperature limits
 */
#define TEMP10        10                                                                  // temp is stored as *10
#define MAXTEMP       (27*TEMP10)                                                         // * 10     27C   top of temp range
#define MINTEMP       (10*TEMP10)                                                         // * 10     10C - bottom of temp range

/*
 *  Sensor filtering
 */
#define TEMPLOWPASSFILTER   8                                                             // low pass filter on temperature analogRead
#define PULSELOWPASSFILTER  4                                                             // low pass filter on pulse periods

/*
 * Temp indexes
 */
#define TEMPINDEXF          0                                                             // Temp F
#define TEMPINDEXC          1                                                             // Temp C
#define NOTEMP              2
/* 
 *  digital input and temperature input values 
 */
bool AirAssistValveInput = true;                                                          // Air assist input from Main Controller
short intemp[NOTEMP];                                                                     // Temperatures F and C
volatile unsigned short pulseperiod[4];                                                   // input pulses/sec, output from interrupt handlers for 4 input pulse pins

#define NOPULSEINPUTS   sizeof(pulseperiod)/sizeof(pulseperiod[0])
/* 
 *  last UI values - save updating UI except when it needs to change
 */
bool oldCurrent_Cool_State;
short oldintemp[NOTEMP];
unsigned short oldpulseperiod[NOPULSEINPUTS];



/* 
 *  Controller state 
 */
bool Current_Cool_State;                                                                  // Laser disabled state on/off
bool Air_Valve;                                                                           // Air Assist valve on/off

/*
 *  Time delay
 */
#define DELAYTIME(last, delaysize)         ((millis() - last) > delaysize)                // right time gone bye
#define UPDATETIME(last)                   (last = millis())                              // update time for next check

/*
 *  Debug State
 */
bool debugon = false;                                                                     // debug off by default
#define DEBUGDELAY              1000                                                      // delay between debug outputs
#define DEBUGON_PIN             8                                                         // if pin low, debugon is set to true

/*
 * LED to show error state on initialization
 */
#define LED_PIN                 PIN_LED                                                   // led error status pin
#define LED_OFF                 HIGH
#define LED_ON                  LOW

#define errorState digitalWrite(LED_PIN, LED_ON)                                          // error - led on
#define clearState digitalWrite(LED_PIN, LED_OFF)                                         // clear error - led off

/* 
 *  local sensor information and state 
 */
#define  TIME_PIN0               0                                                        // pulse in pins
#define  TIME_PIN1               1
#define  TIME_PIN2               2
#define  TIME_PIN3               3
#define  INPUT_PIN               9                                                        // digital input pin
#define  TEMP_PIN                A10                                                      // eblock/temp in pin

#define ANALOGRESOLUTION         ADC_RESOLUTION                                           // ADC resolution
#define ANALOGRANGEMAX           ((1<<ANALOGRESOLUTION)-1)                                // max value of ADC

const unsigned long debounceDelay = 5000;                                                 // the debounce time (us), max freq 100/sec
const unsigned long stoppedDelay = 1000000;                                               // after 1sec without any pin state changes, set to 0 pulses/sec
const byte pinpin[] = { TIME_PIN0, TIME_PIN1, TIME_PIN2, TIME_PIN3, INPUT_PIN };          // set of pins for interrupt handling

#define PINPINSIZE              (sizeof(pinpin)/sizeof(pinpin[0]))
#define AIRASSISTPIN            4                                                         // pinstate index for Air Assist pin state

/*
 * these variables are read/updated in interrupt handlers 
 */
volatile byte pinstate[PINPINSIZE];                                                       // debounced pin state
volatile unsigned long debtime[PINPINSIZE];                                               // time of last pin change for debounce
volatile unsigned long risetime[NOPULSEINPUTS];                                           // time of last debounced rising edge
volatile unsigned long averriseperiodtemp[NOPULSEINPUTS];                                 // time of last rising to rising edge period

/*
 *  Display information
 */
bool displayavailable = false;                                                            // whether we found display correctly
const int displayStartupDelay = 500;                                                      // delay for display to initialize

/*
 *  UI colors
 */
#define RED     63488
#define GREEN    2016
#define WHITE   65535
#define GREY    50712

/*
 *  UI elements
 */
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

/*
 *  UI buttons for callbacks
 */
NexTouch *nex_listen_list[] =
{
    &CoolOn,
    &ExhaustOn,
    &AirOn,
    &LightsOn,
    NULL
};

/*
 *  Relay information
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

/*
 *  Turn relay on/off
 */
bool SetRelay(int relay, bool value)
{
  int b = GETBOARD(relay);                                                                // which relay board
  int r = GETRELAY(relay)-1;                                                              // which relay index (1 less than relay number on board)
  if(relay && relays[b].relayavailable && (value != relays[b].Relaystate[r]))             // relay and relay board available and is the relay state changing
  {
    relays[b].relay->toggleRelay(r+1);                                                    // since we know the relay state, toggle the relay when we want to change it
                                                                                          // we could query relay state to check if it has changed but we assume it worked
    relays[b].Relaystate[r] = value;                                                      // update internal state to match
    if(debugon) Serial.printf("SetRelay %d %d %s\n", relay, relays[b].Relaystate[r], value?"ON":"OFF");
  }
  return true;
}

/*
 *  Button information and state
 */
#define COOLBUTTON      0                                                                 // button indexes in ButtonState array
#define EXHAUSTBUTTON   1
#define AIRASSISTBUTTON 2
#define LIGHTSBUTTON    3

#define BUTTONSTATESIZE   (sizeof(ButtonState)/sizeof(ButtonState[0]))
#define BUTTONINDEX(p)    (p-ButtonState)

struct _ButtonState
{
  bool state;                                                                             // current state
  bool oldstate;                                                                          // UI state
  int relay;                                                                              // relay to toggle if configured
  bool relaystate;                                                                        // current relay state
  NexButton *Button;                                                                      // Button for this configuration
} ButtonState[] = { { false, true, RELAY_COOL, false, &CoolOn },
                    { false, true, RELAY_EXHAUST, false, &ExhaustOn },
                    { false, true, RELAY_AIRASSISTPUMP, false, &AirOn },
                    { false, true, RELAY_LIGHTS, false, &LightsOn } };

/*
 *  Button callback handlers, update button state
 */
void ButtonCallback(void *p)
{
  struct _ButtonState *ptr = (struct _ButtonState *)p;
  ptr->state = !ptr->state;                                                               // toggle button state
  if(debugon) Serial.printf("Button %d %s\n", BUTTONINDEX(ptr), ptr->state?"On":"Off"); 
}

/*
 *  Pulse input and air assist input pin interrupt handlers
 */
/*
 *      GRE    GFE    GRE    BFE  BRE GFE   BRE BFE GRE           Good/Bad Rising/Falling Edge
 * c    1CDRU  0CDU   1CDRU  0C   1C  0CDU  1C  0C  1CDRU         cstate, Curtime, DebTime, Rising/Rising Period, Update Debtime
 * p    0   1  1  0   0   1  1    1   1  0  0   0   0   1         pinstate
 */
void dointerrupt(byte p)
{
  byte cstate = digitalRead(pinpin[p]);                                                   // current input value
  unsigned long curtime = micros();                                                       // current time
  if((pinstate[p] != cstate) && ((curtime - debtime[p]) > debounceDelay))                 // pin change, ignore changes unless debounce period from last change
  {                                                                                       // note will ignore frequencies > 1/debounce
    if((cstate == HIGH) && (p != AIRASSISTPIN))                                           // if pulse input is high calculate period of time from last rising edge except for air assist pin
    {
      averriseperiodtemp[p] = (averriseperiodtemp[p]*(PULSELOWPASSFILTER-1) + (curtime - risetime[p]))/PULSELOWPASSFILTER;
      risetime[p] = curtime;                                                              // time of this rising edge for next time
      if(averriseperiodtemp[p])
        pulseperiod[p] = (1000000L*PULSE100)/averriseperiodtemp[p];                       // calculate pulses/sec * 100
    }
    debtime[p] = curtime;                                                                 // save time for debouncing next change - should be > debounce period from this change
    pinstate[p] = cstate;                                                                 // save debounced pin state
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
  pinMode(DEBUGON_PIN, INPUT_PULLUP);                                                     // input pin to override debug mode
  if(!digitalRead(DEBUGON_PIN)) debugon = true;                                           // if low make sure debug mode is on, otherwise the default in the code
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
  clearState;                                                                             // LED off
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
      errorState;                                                                         // set the error led, we have a problem
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

  analogReadResolution(ANALOGRESOLUTION);                                                 // setup temperature analog pin

  for(int i = 0; i < PINPINSIZE; i++)
  {
    pinMode(pinpin[i], INPUT_PULLUP);                                                     // pulse in pins and air assist input pin
    pinstate[i] = digitalRead(pinpin[i]);                                                 // initialize pin state
    debtime[i] = micros();                                                                // initialize timers
    if(i != AIRASSISTPIN)
      risetime[i] = micros();                                                             // initialize timers
    attachInterrupt(digitalPinToInterrupt(pinpin[i]), int_table[i], CHANGE);              // interrupts on each pin
  }
}

/*
 *  Initialize relay board
 */
void initRelay(int r)
{
  if(!(relays[r].relayavailable = relays[r].relay->begin()))                              // init relay board and check its available
  {
    if(debugon) Serial.printf("Check connections to Qwiic Relay%d\n", r+1);
    errorState;                                                                           // set the error led, we failed to find one of the relay boards
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

void setupRelays()
{
  if(debugon) Serial.printf("SetupRelays\n");
  initRelay(0);
  initRelay(1);
}

/*
 *  Setup everything
 */
void setup()
{
  setupDebug();                                                                           // check if debug is on and init debug serial port

  setupLED();                                                                             // error led

  setupDisplay();                                                                         // display

  setupI2C();                                                                             // I2C bus for relay boards

  setupSensors();                                                                         // sensors

  setupRelays();                                                                          // relays
}

/*
 *  Monitor timing of main loop and error if too large. Loop times are < 8-9ms, most are < 1ms
 *   Most of the delay is when updating the display, non display times < 1ms
 */
#define TIMEOVERALL       0
#define TIMEUPDATEINPUTS  1
#define TIMEUPDATETEMP    2
#define TIMECHECKCOOL     3
#define TIMEAIRASSIST     4
#define TIMERELAYUPDATE   5
#define TIMEDISPLAYNOTIF  6
#define TIMEDISPLAYUPDATE 7

#define NOLOOPTIME      (TIMEDISPLAYUPDATE+1)
unsigned long looptime[NOLOOPTIME];                                                       // timing of main loop and parts of loop
#define MAXLOOPTIME     15000                                                             // max loop time in microseconds
#define SKIPCOUNT       10                                                                // lots of initialization the first few times round loop

void MonitorTimingLoop()
{
  static int startc = SKIPCOUNT, maxcount[NOLOOPTIME];
  static unsigned long avertime[NOLOOPTIME], mintime[NOLOOPTIME], maxtime[NOLOOPTIME];

  if(startc)                                                                              // skip average/min/max during initialization
  {
    if(!(--startc))
      for(int i = 0; i < NOLOOPTIME; i++)
        avertime[i] = mintime[i] = maxtime[i] = looptime[i];                              // initialize everything on last skip
  }
  else
  {
    for(int i = 0; i < NOLOOPTIME; i++)
    {
      avertime[i] = (avertime[i] + looptime[i])/2;                                        // track average, min and max times for parts of loop
      if(looptime[i] < mintime[i]) mintime[i] = looptime[i];
      if(looptime[i] > maxtime[i]) maxtime[i] = looptime[i];
      if(looptime[i] > MAXLOOPTIME) maxcount[i]++;
      looptime[i] = 0;
    }

    if(debugon)
    {
      static unsigned long lastdistime = 0;
      if(DELAYTIME(lastdistime, DEBUGDELAY))
      {
        UPDATETIME(lastdistime);
        Serial.printf("Loop times%s: aver %lu, min %lu, max %lu maxcount %d\n",
          (maxtime[0] > MAXLOOPTIME)?" too large":"", avertime[TIMEOVERALL], mintime[TIMEOVERALL], maxtime[TIMEOVERALL], maxcount[TIMEOVERALL]);
        for(int i = 1; i < NOLOOPTIME; i++)
          if(maxtime[i] > MAXLOOPTIME)
            Serial.printf("loop times%d: aver %lu, min %lu, max %lu maxcount %d\n", i, avertime[i], mintime[i], maxtime[i], maxcount[i]);
      }
    }
  }
}

/*
 *  get sensor values from pulse, air assist input interrupt state and translate them into useful values we can act on
 */
void UpdatePinInputs()
{
  static unsigned long distime;

  for(int i = 0;i < NOPULSEINPUTS;i++)                                                    // note using size of pulseperiod because that is what will be reset
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

  if(debugon && DELAYTIME(distime, DEBUGDELAY))
  {
    static bool aa;
    static short pp[NOPULSEINPUTS];
    UPDATETIME(distime);
    if(((aa != AirAssistValveInput) || (pp[0] != pulseperiod[0]) || (pp[1] != pulseperiod[1]) || (pp[2] != pulseperiod[2]) || (pp[3] != pulseperiod[3])))
    {
      Serial.printf("Sensor %d %d %d %d %d\n", pulseperiod[0], pulseperiod[1], pulseperiod[2], pulseperiod[3], AirAssistValveInput);
      aa = AirAssistValveInput; pp[0] = pulseperiod[0]; pp[1] = pulseperiod[1]; pp[2] = pulseperiod[2]; pp[3] = pulseperiod[3];
    }
  }
}

/*
 *  get temp sensor value and translate it into actual temperatures C and F
 */
#define MINEBLOCKTEMP   (-22*TEMP10)
#define MAXEBLOCKTEMP   (56*TEMP10)

void UpdateTempInputs()
{
  static unsigned long distime;

  short Currenttemp = (Currenttemp*(TEMPLOWPASSFILTER-1) + analogRead(TEMP_PIN))/TEMPLOWPASSFILTER; // analogRead should take ~9.5us, low pass on temp
  intemp[TEMPINDEXC] = map(Currenttemp, 0, ANALOGRANGEMAX, MINEBLOCKTEMP, MAXEBLOCKTEMP);           // linear map of eblock/ADC to temp C * 10
  intemp[TEMPINDEXF] = (intemp[TEMPINDEXC] * 9)/ 5 + 32*TEMP10;                                     // standard translation of C to F, except temp * 10

  if(debugon && DELAYTIME(distime, DEBUGDELAY))
  {
    static short T[NOTEMP];
    UPDATETIME(distime);
    if((T[TEMPINDEXF] != intemp[TEMPINDEXF]) || (T[TEMPINDEXC] != intemp[TEMPINDEXC]))
    {
      Serial.printf("Temp Sensor %d %d\n", intemp[TEMPINDEXC], intemp[TEMPINDEXF]);
      T[TEMPINDEXF] = intemp[TEMPINDEXF]; T[TEMPINDEXC] = intemp[TEMPINDEXC];
    }
  }
}

/*
 *  Check whether we are cooling laser correctly, update cooling state and disable laser if there is a problem
 */
#define TEMPOUTOFRANGE           ((intemp[TEMPINDEXC] < MINTEMP) || (intemp[TEMPINDEXC] > MAXTEMP))
#define PULSEOUTOFRANGE(i)       ((pulseperiod[i] < MINPULSEFREQ) || (pulseperiod[i] > MAXPULSEFREQ))

void ValidateCoolingInputs()
{
                                                                                          // if one of the cooling fans, pump, water flow sensor is bad
                                                                                          // or water temp is out of range we have a problem -> disable laser water protect
  bool newState = (TEMPOUTOFRANGE || PULSEOUTOFRANGE(0) || PULSEOUTOFRANGE(1) || PULSEOUTOFRANGE(2) || PULSEOUTOFRANGE(3));

  if (Current_Cool_State != newState)                                                     // cooling state has changed
  {
    if(SetRelay(RELAY_DISABLELASER, newState))                                            // update whether laser enabled or disabled
    {
      Current_Cool_State = newState;
      if(debugon) Serial.printf("Laser %s\n", Current_Cool_State?"Disabled":"Enabled");
    }
  }
}

/*
 *  Calculate air assist valve state and update relay for air assist valve
 */
void UpdateAirAssistValve()
{
  bool newState = ButtonState[AIRASSISTBUTTON].state?AirAssistValveInput:true;            // If AirAssist pump is on then follow AirAssist input otherwise valve should be open
  if(Air_Valve != newState)                                                               // Air valve state needs to change
  {
    if(SetRelay(RELAY_AIRASSISTVALVE, newState))                                          // update Air assist valve relay
    {
      Air_Valve = newState;
      if(debugon) Serial.printf("Air Assist Valve %s\n", Air_Valve?"Open":"Closed");
    }
  }
}

/*
 *  Update relay state to match button state, if relay assigned to a button
 */
void UpdateRelayState()
{
  for(int i = 0; i < BUTTONSTATESIZE; i++)
    if(ButtonState[i].relaystate != ButtonState[i].state)                                 // need to change relay state
      if(SetRelay(ButtonState[i].relay, ButtonState[i].relaystate))                       // SetRelay checks a relays current state and changes it if required, 0 is no relay
        ButtonState[i].relaystate = ButtonState[i].state;
}

/*
 *  Updating Display routines
 */
#define UpdateDelay   1000
#define ERRORBIT(x)   (1<<(x))
unsigned long errorbits[] = {ERRORBIT(0), ERRORBIT(2), ERRORBIT(4), ERRORBIT(6), ERRORBIT(8), ERRORBIT(10), ERRORBIT(12), ERRORBIT(13), ERRORBIT(14), ERRORBIT(15), ERRORBIT(16) };
char *tsdebug[] = { "TempF", "TempC", "Pulse 0", "Pulse 1", "Pulse 2", "Pulse 3" };
NexText *TextUI[] = { &TempF, &TempC, &c1, &c2, &c3, &c4 };
char displaybuffer[15];                                                                   // int to text buffer used to update text in display

/*
 * Update temp display and pulse display values
 */
bool UpdateTPDisplay(int index, unsigned long sret, unsigned long *rets)
{
  static unsigned int lasttime[6] = {0, 0, 0, 0, 0, 0};
  bool ret = false;
  bool color;
  if(DELAYTIME(lasttime[index], UpdateDelay))                                             // slow down display updates
  {
    if(index <= TEMPINDEXC)
    {
      ret = (oldintemp[index] != intemp[index]);                                          // temperature changed
      sprintf(displaybuffer, "%d.%d", intemp[index]/TEMP10, abs(intemp[index]%TEMP10));   // format for temperature
      color = TEMPOUTOFRANGE;
    }
    else
    {
      ret = (oldpulseperiod[index-NOTEMP] != pulseperiod[index-NOTEMP]);                  // pps changed
      if(pulseperiod[index-NOTEMP])
        sprintf(displaybuffer, "%d.%d", pulseperiod[index-NOTEMP]/PULSE100, pulseperiod[index-NOTEMP]%PULSE100); // format for pulses
      else
        strcpy(displaybuffer, "stopped");                                                 // no pulses
      color = PULSEOUTOFRANGE(index-NOTEMP);
    }
    if(ret)                                                                               // we have an update to do
    {
      if(displayavailable)
      {
        if(!TextUI[index]->setText(displaybuffer)) *rets |= sret;
        if(!TextUI[index]->Set_background_color_bco(color?RED:WHITE)) *rets |= (sret<<1);
      }
      if(!*rets)
      {
        if(index < NOTEMP)
          oldintemp[index] = intemp[index];                                               // save state if updated display temp
        else
          oldpulseperiod[index-NOTEMP] = pulseperiod[index-NOTEMP];                       // save state if updated display pulse
        UPDATETIME(lasttime[index]);                                                      // update successful so wait UpdateDelay before checking again
      }
      if(debugon) Serial.printf("Display %s '%s' %lx\n", tsdebug[index], displaybuffer, *rets);
    }
    else
      UPDATETIME(lasttime[index]);                                                        // no change so wait UpdateDelay before checking again
  }
  return ret;
}

/*
 * Update cooling ok display state
 */
bool UpdateCoolDisplay(unsigned long sret, unsigned long *rets)
{
  bool ret;
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

/*
 * Updare button display state to match our button state
 */
bool UpdateDisplayButton(int index, unsigned long sret, unsigned long *rets)
{
  bool ret;
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
 *    Dont update state unless display acks correctly, will retry later since state hasn't been updated
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
    case 0: case 1: case 2: case 3: case 4: case 5:                                       // 0: TEMPINDEXF, 1: TEMPINDEXC, 2-5: pulse index 0-3
      updateddisplay = UpdateTPDisplay(part, errorbits[part], &rets);                     // update temp/pulse if changed
      break;
    case 6:
      updateddisplay = UpdateCoolDisplay(errorbits[part], &rets);                         // update cooling state if changed
      break;
    case 7: case 8: case 9: case 10:
      updateddisplay = UpdateDisplayButton(part-7, errorbits[part], &rets);               // update display buttons if changed
      break;
    }
    ++part %= 11;                                                                         // round robin the 11 parts of display
  } while(!updateddisplay && (startpart != part));                                        // loop until we have tried updating 1 UI part or none needing updating
  if(rets && debugon) Serial.printf("Display update failures %lx\n", rets);               // did we have any errors when updating display
}

/*
 *  Main loop
 */
#define SensorDelay   100                                                                 // Sensor update delay

#define STARTTIMEL    temptime[1] = micros()                                              // start time of measurement
#define ENDTIMEL(i)   looptime[i] = micros() - temptime[1]                                // end time of measurement
#define STARTTIMES    temptime[0] = micros()                                              // start time of measurement
#define ENDTIMES(i)   looptime[i] = micros() - temptime[0]                                // end time of measurement

void loop()
{
  static unsigned long lastSensorTime;
  unsigned long temptime[2];                                                              // timing start times

  MonitorTimingLoop();                                                                    // Monitor speed of this loop

  STARTTIMEL;                                                                             // time main loop
  if(DELAYTIME(lastSensorTime, SensorDelay))                                              // update sensors and controller state every SensorDelay milliseconds
  {
    UPDATETIME(lastSensorTime);
    STARTTIMES;   UpdatePinInputs();        ENDTIMES(TIMEUPDATEINPUTS);                   // Update digital pin inputs
    STARTTIMES;   UpdateTempInputs();       ENDTIMES(TIMEUPDATETEMP);                     // Update temp input
    STARTTIMES;   ValidateCoolingInputs();  ENDTIMES(TIMECHECKCOOL);                      // Check if we have a cooling problem
    STARTTIMES;   UpdateAirAssistValve();   ENDTIMES(TIMEAIRASSIST);                      // Enable air assist valve if requested
    STARTTIMES;   UpdateRelayState();       ENDTIMES(TIMERELAYUPDATE);                    // Update relay state for buttons
  }
  STARTTIMES;     nexLoop(nex_listen_list); ENDTIMES(TIMEDISPLAYNOTIF);                   // Get any button events from display
  STARTTIMES;     UpdateDisplay();          ENDTIMES(TIMEDISPLAYUPDATE);                  // Update display for current state
  ENDTIMEL(TIMEOVERALL);                                                                  // time main loop
}
