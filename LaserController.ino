/*
 * 4 input pins for water, fan, etc. should be pulsing > 5/sec < 20/sec
 * 1 input pin from Main Controller, high when wanting Air Assist - output 1 from AWC608 low run air assist, high stop air assist
 * 1 analog pin for water temperature
 * 
 * 1 Nextion display connected via serial port - need 1K resistor in Tx from Nextion
 *    Shows - water temp in C and F
 *    Shows - speeds of water/fan/etc. sensors
 *    Shows 4 buttons, see above
 *    Shows state of Enable/Disable laser firing
 * 
 * 4 Buttons
 *    Turn on/off Water Cooling
 *    Turn on/off Exhaust fan
 *    Turn on/off Air Assist pump
 *    Turn on/off internal light
 *    
 * 6 Relays connected via I2C to 2 boards (0x8, 0x6D)
 *    Turn on/off Water Cooling   - Connected to button, 110V AC
 *    Turn on/off Exhaust fan     - Connected to button, 110V AC
 *    Turn on/off Air Assist pump - Connected to button, 110V AC
 *    Turn on/off internal lights - Connected to button, 12V DC
 *    Open/close Air Assist valve - Open when pump off or pump on and main controller wants air assist, 24V DC
 *    Enable/Disable laser firing - Disable if Temp under/over and water/fan/etc pulse rate is under/over, 24V DC
 *
 *  D0-3  pulse input pins      To water flow, fan, etc sensors
 *  D4,5  I2C                   To relay boards
 *  D6,7  Serial1               To NexTion Display
 *  D8    Debug mode            Enable debug if low
 *  D9    Air assist input      To Main Controller output 1 (low active)
 *  A10   Temp input            Thermistor Temperature sensor via eblock
 *  D13   Error state led       Internal board LED used to show error state
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

#define LOWPASSFILTER 8
/* 
 *  digital input and temperature input values 
 */
bool AirAssistValveInput = true;
short intempF, intempC;
unsigned short pulseperiod[4];

/* 
 *  last UI values - save updating UI except when needed 
 */
unsigned short oldpulseperiod[4];
short oldintempF, oldintempC;
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

const unsigned long debounceDelay = 10000;                                                // the debounce time (us), max freq 100/sec
const unsigned long stoppedDelay = 1000000;                                               // after 1sec without any pin state changes, set to 0 pulses/sec
const byte pinpin[] = { TIME_PIN0, TIME_PIN1, TIME_PIN2, TIME_PIN3, INPUT_PIN };

#define PINPINSIZE              (sizeof(pinpin)/sizeof(pinpin[0]))
#define AIRASSISTPIN            4                                                         // pinstate index for Air Assist pin state
                                                                                          // these are read/updated in interrupt handlers
volatile byte lastpinstate[PINPINSIZE];                                                   // non-debounced pin state
volatile byte pinstate[PINPINSIZE];                                                       // debounced pin state
volatile unsigned long debtime[PINPINSIZE];                                               // time of last pin change for debounce
volatile unsigned long risetime[PINPINSIZE];                                              // time of last debounced rising edge
volatile unsigned long averriseperiodtemp[PINPINSIZE];                                    // time of last rising to rising edge period

/*
 *  Display information
 */
bool displayavailable = false;                                                            // whether we found display correctly
const int displayStartupDelay = 1000;                                                     // delay for display to initialize

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
 *  Pulse input and air assist input pin interrupt handlers
 */
void dointerrupt(int p)
{
  byte cstate = digitalRead(pinpin[p]);                                                   // current input value
  unsigned long curtime = micros();                                                       // current time
  if(lastpinstate[p] != cstate)                                                           // pin change, start debounce, shouldn't really need this test
  {
    unsigned long dtime = debtime[p];                                                     // last time pin changed
    if ((curtime - dtime) > debounceDelay)                                                // pulse is valid
    {
      if (lastpinstate[p] != pinstate[p])                                                 // debounced input has changed
      {
        pinstate[p] = lastpinstate[p];                                                    // saved debounced pin state
        if((lastpinstate[p] == HIGH) && (p != AIRASSISTPIN))                              // pulse inputs calculate period of pulse
        {
          averriseperiodtemp[p] = (averriseperiodtemp[p]*(LOWPASSFILTER-1) + (dtime - risetime[p]))/LOWPASSFILTER;
          risetime[p] = dtime;                                                            // time of last rising edge
          pulseperiod[p] = (1000000L*PULSE100)/averriseperiodtemp[p];                     // calculate pulses/sec * 100
        }
      }
    }
    debtime[p] = curtime;                                                                 // save time and input state for debouncing
    lastpinstate[p] = cstate;
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
  if(!nexInit())                                                                          // initialize connection to display and display
  {                                                                                       // sometimes the 1st command to the display fails
    if(debugon) Serial.printf("Display1\n");
    Serial1.end();
    if(!(displayavailable = nexInit()))                                                   // retry init display
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
    lastpinstate[i] = pinstate[i] = digitalRead(pinpin[i]);                               // init pin states
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

#define MAXLOOPTIME     15000
#define MAXTIMEDELAY    1000
#define SKIPCOUNT       5
/*
 * Monitor timing of main loop and error if too large. Loop times are < 7.5ms and most are < 1ms
 */
void MonitorTimingLoop()
{
  static int startc = SKIPCOUNT;
  static unsigned long maxtimetaken;
  static bool takentiming = false;
  static unsigned long avertime = 0, mintime = 0xffffffff, maxtime = 0, lastime = 0;
  unsigned long curtime = micros();
  unsigned long delta = curtime - lastime;
  lastime = curtime;

  if(startc)                                                                              // skip average/min/max during initialization
  {
    startc--;
    avertime = delta;
  }
  else
  {
    if(maxtime > MAXLOOPTIME)
    {
      if(!takentiming)
      {
        maxtimetaken = millis();
        takentiming = true;
      }
      else if((maxtimetaken - millis()) > MAXTIMEDELAY)
      {
        takentiming = false;
        maxtime = (maxtime + delta)/2;
        clearState();
        if(debugon) Serial.printf("Loop times reset: aver %lu, min %lu, max %lu\n", avertime, mintime, maxtime);
      }
    }
    avertime = (avertime + delta)/2;
    if(delta < mintime) mintime = delta;
    if(delta > maxtime) maxtime = delta;
    if((avertime > MAXLOOPTIME) || (maxtime > MAXLOOPTIME))
      errorState();                                                                       // set the error led, if average or max are too large
  }
  if(debugon)
  {
    static unsigned long lastdistime = 0;
    if((millis() - lastdistime) > DEBUGDELAY)
    {
      lastdistime = millis();
      Serial.printf("Loop times%s: aver %lu, min %lu, max %lu\n",
        (!startc && ((avertime > MAXLOOPTIME) || (maxtime > MAXLOOPTIME)))?" too large":"", avertime, mintime, maxtime);
    }
  }
}

/*
 *  get sensor values from pulse input sensor, air assist input sensor and temp sensor and translate them into usable values
 */
void UpdatePinInputs()
{
  static unsigned long distime;

  for(int i = 0;i < PINPINSIZE;i++)
  {
    if ((micros() - debtime[i]) > stoppedDelay)                                           // long delay since last change
      pulseperiod[i] = 0;                                                                 // reset pulseperiod
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
  static short Currenttemp;

  Currenttemp = (Currenttemp*(LOWPASSFILTER-1) + analogRead(TEMP_PIN))/LOWPASSFILTER;     // low pass on temp
  intempC = map(Currenttemp, 0, ANALOGRANGEMAX, MINEBLOCKTEMP, MAXEBLOCKTEMP);            // linear map of eblock/ADC to temp C * 10
  intempF = (intempC * 9)/ 5 + 32*TEMP10;                                                 // standard translation of C to F, except temp * 10

  if(debugon && ((millis() - distime) > DEBUGDELAY))
  {
    static byte F, C;
    distime = millis();
    if(((F != intempF) || (C != intempC)))
    {
      Serial.printf("Temp Sensor %d %d\n", intempC, intempF);
      F = intempF; C = intempC;
    }
  }
}

#define TEMPOUTOFRANGE           ((intempC < MINTEMP) || (intempC > MAXTEMP))
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

/*
 *  Updating Display routines
 */
#define CHECKTEMPDELTA(x, y)    (abs(x - y) > (2*TEMP10))                                 // 2 degrees

bool UpdateTempDisplay(short *oldintemp, short intemp, NexText *Temp, char *s, unsigned long sret, unsigned long *rets)
{
  bool ret = false;

  if(ret = CHECKTEMPDELTA(*oldintemp, intemp))
  {
    sprintf(buffer, "%d.%d", intemp/TEMP10, abs(intemp%TEMP10));
    if(displayavailable)
    {
      if(!Temp->setText(buffer)) *rets |= sret;
      if(!Temp->Set_background_color_bco(TEMPOUTOFRANGE?RED:WHITE)) *rets |= (sret<<1);
    }
    if(!*rets)
      *oldintemp = intemp;                                                                // save state if updated display
    if(debugon) Serial.printf("Display Temp%s '%s' %lx\n", s, buffer, *rets);
  }
  return ret;
}

#define CHECKPULSEDELTA(x, y)    (abs(x - y) > 50)                                        // 0.5 pulse/sec

unsigned long UpdatePulseDisplay(int index, NexText *c, unsigned long sret, unsigned long *rets)
{
  bool ret = false;

  if(ret = CHECKPULSEDELTA(oldpulseperiod[index], pulseperiod[index]))                    // update pulse freq if changed
  {
    if(pulseperiod[index])
      sprintf(buffer, "%d.%d", pulseperiod[index]/PULSE100, pulseperiod[index]%PULSE100);
    else
      strcpy(buffer, "stopped");
    if(displayavailable)
    {
      if(!c->setText(buffer)) *rets |= sret;
      if(!c->Set_background_color_bco(PULSEOUTOFRANGE(index)?RED:WHITE)) *rets |= (sret<<1);
    }
    if(!*rets)
      oldpulseperiod[index] = pulseperiod[index];                                         // save state if updated display
    if(debugon) Serial.printf("Display Pulse %d '%s' %lx\n", index, buffer, *rets);
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

#define ERRORBIT(x)   (1<<(x))
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
    case 0:
      updateddisplay = UpdateTempDisplay(&oldintempF, intempF, &TempF, "F", ERRORBIT(0), &rets);// update temps if changed
      break;
    case 1:
      updateddisplay = UpdateTempDisplay(&oldintempC, intempC, &TempC, "C", ERRORBIT(2), &rets);
      break;
    case 2:
      updateddisplay = UpdatePulseDisplay(0, &c1, ERRORBIT(4), &rets);                    // update pulse freq if changed
      break;
    case 3:
      updateddisplay = UpdatePulseDisplay(1, &c2, ERRORBIT(6), &rets);
      break;
    case 4:
      updateddisplay = UpdatePulseDisplay(2, &c3, ERRORBIT(8), &rets);
      break;
    case 5:
      updateddisplay = UpdatePulseDisplay(3, &c4, ERRORBIT(10), &rets);
      break;
    case 6:
      updateddisplay = UpdateCoolDisplay(ERRORBIT(12), &rets);                            // update cooling state if changed
      break;
    case 7:
      updateddisplay = UpdateDisplayButton(COOLBUTTON, ERRORBIT(13), &rets);              // update display buttons if changed
      break;
    case 8:
      updateddisplay = UpdateDisplayButton(EXHAUSTBUTTON, ERRORBIT(14), &rets);
      break;
    case 9:
      updateddisplay = UpdateDisplayButton(AIRASSISTBUTTON, ERRORBIT(15), &rets);
      break;
    case 10:
      updateddisplay = UpdateDisplayButton(LIGHTSBUTTON, ERRORBIT(16), &rets);
      break;
    }
    ++part %= 11;
  } while(0); //while(!updateddisplay && (startpart != part));                                        // loop until we have updated 1 UI part or tried them all
  if(rets && debugon) Serial.printf("Display update failures %lx\n", rets);
}

#define SensorDelay   100

void loop()
{
  MonitorTimingLoop();                                                                    // Monitor speed of this loop
  
  static unsigned long lastSensorTime;
  if((millis() - lastSensorTime) > SensorDelay)                                           // update sensors and controller state every SensorDelay milliseconds
  {
    UpdatePinInputs();                                                                    // Update digital pin inputs
    UpdateTempInputs();                                                                   // Update temp input

    ValidateCoolingInputs();                                                              // Check if we have a cooling problem
    UpdateAirAssistValve();                                                               // Enable air assist valve if requested
    UpdateRelayState();                                                                   // Update relay state for buttons
  }

  nexLoop(nex_listen_list);                                                               // Get any button events from display
  UpdateDisplay();                                                                        // Update display for current state
}
