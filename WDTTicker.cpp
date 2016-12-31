#define F_CPU 4800000                           //T13: use fuses  high:0xFF low:0x75  
//#define F_CPU 6400000                         //T85: use fuses  high:0xDF low:0xD3  
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>    
#include <avr/eeprom.h>
#include <avr/sleep.h>

// {{{ Definitions

#define STAR2PIN   PB0
#define STAR3PIN   PB4
#define SWITCHPIN  PB3                  // what pin the switch is connected to, which is Star 1
#define PWMPIN     PB1
#define VOLTAGEPIN PB2

#define PWM_LVL     OCR0B               // OCR0B is the output compare register for PB1


#define SWITCHDEBOUNCEDURATION 0b00001111 // time before we consider the switch released after	 
                                          // each bit of 1 from the right equals 16ms, so 0x0f = 64ms

#define SWITCHLONGPRESSDURATION     64    // How many WDT ticks until we consider a press a long press
                                          // 32 is roughly 0.512 s
                                          // 64 is roughly 1.024 s

#define SWITCHSHORTRELEASETIMERMAX  32    // This is to detect double short releases.
                                          // 32 * 0.16s = 0.512s
                                          // After this period, short release is process & counted.


#define LEDPOWEROFFTIMERMAX        100    // 100 * 0.1s = 10s,
                                          // After this period, flashlight enters sleep mode.
#define LEDMODEHIGHTIMERMAX        900    // 900 * 0.1s = 90s, after which flashlight exit high mode
                                          // and enter the lowest normal mode.
#define LEDPOWERLOWVOLTAGETIMERMAX  40    // 40 * 0.1s =  4s
                                          // After this period, flashlight enter lower mode
                                          // than current mode.

#define ADC_LOW                 130     // When do we start ramping
#define ADC_CRIT                120	// When do we shut the light off
#define ADC_DELAY               188     // Delay in ticks between low-bat rampdowns (188 ~= 3s)

#define ADC_CHANNEL             0x01    // MUX 01 corresponds with PB2
#define ADC_DIDR                ADC1D   // Digital input disable bit corresponding with PB2
#define ADC_PRSCL               0x06    // clk/64


// - These values (eg: PWM) will be different between different
//   types of flashlight hosts/LEDs.
// - The values defined below are for M6 (with Nanjg only driver).
#define MODESLOW         6,12            // May consist of multiple PWM values.
#define MODESLOWSIZE     2
#define MODESLOWINDEX    0

#define MODESNORMAL      22,54,128       // May consist of multiple PWM values.
#define MODESNORMALSIZE  3
#define MODESNORMALINDEX 2

#define MODESHIGH        255             // Must consist of one PWM value only.
#define MODESHIGHSIZE    1
#define MODESHIGHINDEX   5

#define MODESSOS         1               // Must consist of one magic value only
#define MODESSOSSIZE     1
#define MODESSOSINDEX    6

#define MODESBLINK       2               // Must consist of one magic value only.
#define MODESBLINKSIZE   1
#define MODESBLINKINDEX  7


#define LEDMODEBLINKTIMERMAX 2           // 2*0.1 ms = 0.2ms

const uint8_t LedModes[]={MODESLOW, MODESNORMAL, MODESHIGH, MODESSOS, MODESBLINK};   // Must begin with low modes, then normal modes.

// }}}

// Global variables.
// - are also static variables?
// volatile uint16_t WdtTicker        = 0;
// volatile uint8_t  SwitchIsPressed = 0;
volatile uint8_t  LedModeIndex     = MODESLOWSIZE+MODESNORMALSIZE-1; // Begins with the brightest in normal mode.
volatile uint8_t  SwitchEvent      = 0;
static   uint8_t  LedPowerStatus   = 1;

// Forward declarations
// - should be in header file.
static inline void SwitchProcess();

// {{{ Adc 

void AdcOn() 
{
    ADMUX  = (1 << REFS0) | (1 << ADLAR) | ADC_CHANNEL; // 1.1v reference, left-adjust, ADC1/PB2
    DIDR0 |= (1 << ADC_DIDR);                           // disable digital input on ADC pin to reduce power consumption
    ADCSRA = (1 << ADEN ) | (1 << ADSC ) | ADC_PRSCL;   // enable, start, prescale
}

void AdcOff() 
{
    ADCSRA &= ~(1<<7); //ADC off
}

// }}}

// {{{ Pci

static inline void PciOn()
// Enable pin change interrupts.
{
  GIMSK |= (1 << PCIE);
}

static inline void PciOff()
// Disable pin change interrupts.
{
  GIMSK &= ~(1 << PCIE);
}

// Need an interrupt for when pin change is enabled to ONLY wake us from sleep.
// All logic of what to do when we wake up will be handled in the main loop.
EMPTY_INTERRUPT(PCINT0_vect);

// }}} 

// {{{ Wdt

void WdtOn() {
    // Setup watchdog timer to only interrupt, not reset, every 16ms.
    cli();                          // Disable interrupts
    wdt_reset();                    // Reset the WDT
    WDTCR |= (1<<WDCE) | (1<<WDE);  // Start timed sequence
    //WDTCR = (1<<WDIE);             // T85: Enable interrupt every 16ms
    WDTCR = (1<<WDTIE);             // T13: Enable interrupt every 16ms    
    // WDTCR = (1<<WDTIE) | (1 << WDP1) | (1 << WDP0);  // Enable interrupt every 125ms
    sei();                          // Enable interrupts
}

void WdtOff()
{
    cli();                          // Disable interrupts
    wdt_reset();                    // Reset the WDT
    MCUSR &= ~(1<<WDRF);            // Clear Watchdog reset flag
    WDTCR |= (1<<WDCE) | (1<<WDE);  // Start timed sequence
    WDTCR = 0x00;                   // Disable WDT
    sei();                          // Enable interrupts
}

ISR(WDT_vect) 
{
  // ++WdtTicker;
  // SwitchDetect();
  SwitchProcess();
}

// }}}

// {{{ Switch

#define SWITCHEVENTLONGPRESS   1
#define SWITCHEVENTSHORTPRESS  2 // Must begin with 2
#define SWITCHEVENTSHORTPRESS2 3
#define SWITCHEVENTSHORTPRESS3 4

// Debounced switch press value
// static inline void SwitchDetect()
static inline int SwitchIsPressed()
{
  // Keep track of last switch values polled
  static uint8_t buffer = 0x00;
  // Shift over and tack on the latest value, 0 being low for pressed, 1 for pulled-up for released
  buffer = (buffer << 1) | ((PINB & (1 << SWITCHPIN)) == 0);
  return (buffer & SWITCHDEBOUNCEDURATION);
  // SwitchIsPressed = (buffer & SWITCHDEBOUNCEDURATION);
}

static inline void SwitchProcess()
{
  static uint8_t  switchStatusPress         = 0;
  static uint8_t  switchPressDuration       = 0;
  static uint8_t  switchStatusLongPress     = 0;  
  static uint8_t  switchShortReleaseTimer   = 0;
  static uint8_t  switchShortReleaseCounter = 0;  

  if (switchShortReleaseTimer) // Count up if the timer is enable.
  {
    // Disable the timer if it exceeds its period.
    if (++switchShortReleaseTimer > SWITCHSHORTRELEASETIMERMAX) // Times out.
    {
      switchShortReleaseTimer = 0; // Disable the timer.
    }
  }

  if (SwitchIsPressed()) 
  //if (SwitchIsPressed) 
  { // Press
    switchStatusPress   = 1;
    switchPressDuration++;

    if (switchPressDuration == SWITCHLONGPRESSDURATION) 
    { // Switch is pressed over long period.
      switchPressDuration   = 0;
      switchStatusLongPress = 1;

      // LedModeGoNext();

      SwitchEvent = SWITCHEVENTLONGPRESS;
    }
  } 
  // If not, then switch is not pressed, or..
  // just released ! (<- this is important)  

  // If there is a prior switch press.
  else if (switchStatusPress)
  { // This means that switch is just released.

    if (switchStatusLongPress) 
    { // Switch is released after long press.
      
      // Do nothing.
      switchStatusLongPress = 0; // We are not in the long press anymore.
    }
    else
    { // Switch is released after short press.

      // If there is not an ongoing timer,
      if (switchShortReleaseTimer == 0) 
      { // This is the first short release.
        switchShortReleaseCounter = 1;
      }
      else // If there is an ongoing timer,
      { // Then this is a second short release.
        switchShortReleaseCounter++;
      }
      switchShortReleaseTimer = 1; // Start the timer.
      
      // // If there is an ongoing timer,
      // if (switchShortReleaseTimer) 
      // { // Then this is a second short release.
      //   switchShortReleaseTimer = 0; // Reset & disable the timer.
      //   SwitchEvent = SWITCHEVENTSHORTPRESS2;
      // }
      // else
      // { // This is the first short release.
      //   switchShortReleaseTimer = 1; // Start the timer.
      //   SwitchEvent = SWITCHEVENTSHORTPRESS;
      // }
    }
    
    switchStatusPress   = 0;
    switchPressDuration = 0;
  } 
  else
  { // Switch is not pressed.
    switchPressDuration = 0;

    if (switchShortReleaseTimer && 
        ++switchShortReleaseTimer > SWITCHSHORTRELEASETIMERMAX) 
    {
      // switch(switchShortReleaseCounter) 
      // {
      // case 1:
      //   SwitchEvent = SWITCHEVENTSHORTPRESS;
      //   break;
      // case 2:
      //   SwitchEvent = SWITCHEVENTSHORTPRESS2;
      //   break;
      // case 2:
      //   SwitchEvent = SWITCHEVENTSHORTPRESS2;
      //   break;
      // default:
      //   ; // do nothing
      // }

      SwitchEvent = switchShortReleaseCounter + 1; 
      
      switchShortReleaseTimer = 0;
    }
  }
}

// }}}

// {{{ Led Mode

void LedModeGoNext()  
{
  // TODO: Detect normal mode or not.
  if (LedModeIndex < MODESLOWSIZE+MODESNORMALSIZE)
  { // Modes: Low & Normal
    if (++LedModeIndex >= MODESLOWSIZE+MODESNORMALSIZE) 
    {
      // Normal mode starts after low mode.
      LedModeIndex = MODESLOWSIZE; 
    }   
  }
  else
  { // Modes: High, SOS & Blink
    if (++LedModeIndex >= MODESLOWSIZE+MODESNORMALSIZE+MODESHIGHSIZE+MODESSOSSIZE+MODESBLINKSIZE) 
    {
      LedModeIndex = MODESLOWSIZE; 
    }
  }
  PWM_LVL = LedModes[LedModeIndex];
}

void LedModeNormalEnter()
{
  LedPowerStatus = 1;
  // Normal mode starts after low mode.
  LedModeIndex = MODESLOWSIZE;
  PWM_LVL = LedModes[LedModeIndex];
}

void LedModeLowEnter()
{
  LedPowerStatus = 1;
  // Start in the lowest low mode.
  LedModeIndex   = 0;  
  PWM_LVL = LedModes[LedModeIndex];
}

void LedModeHighAutoExit()
{
  static uint8_t  LedModeHighTimer = 0;

  // If flashlight is currently on and in high mode,
  if (LedPowerStatus == 1 && 
      // LedModeIndex   >= MODESLOWSIZE + MODESNORMALSIZE) 
      // PWM_LVL == MODESHIGH)  // TODO: Detect with LedModeIndex
      LedModeIndex == MODESHIGHINDEX) 
  { // High mode
    
    // If flashlight is in high mode long enough,
    if (++LedModeHighTimer >= LEDMODEHIGHTIMERMAX) //DEBUG
    { 
      LedModeNormalEnter();
      LedModeHighTimer = 0; // Disable timer.
    }
  }
  else 
  { // Not high mode
    LedModeHighTimer = 0; // Disable timer.
  }
}

void LedModeHighEnter()
{
  LedPowerStatus   = 1;
  // Start auto exit timer
  // LedModeHighTimer = 1;
  // High mode is after normal mode.
  LedModeIndex   = MODESLOWSIZE+MODESNORMALSIZE; 
  PWM_LVL        = LedModes[LedModeIndex];
}

// void LedModeBlink()
// {
//   static uint8_t LedModeBlinkTimer = 0;

//   if (LedModeIndex == MODESBLINKINDEX)
//   {
//     LedModeBlinkTimer++;
    
//     if (LedModeBlinkTimer < LEDMODEBLINKTIMERMAX) 
//     {
//       PWM_LVL =   0;
//     } 
//     else if (LedModeBlinkTimer < 2*LEDMODEBLINKTIMERMAX) 
//     {
//       PWM_LVL = 255;
//     }
//     else // if (LedModeBlinkTimer >= 2*LEDMODEBLINKTIMERMAX) 
//     {
//       LedModeBlinkTimer = 0;
//     }
//   }
//   else
//   {
//     LedModeBlinkTimer = 0;
//   }
// }

// void LedModeProgramEnter()
// {
//   uint8_t blink = 3;
//   while (blink-- > 0)
//   {
//     PWM_LVL = 255;
//     _delay_ms(250);
//     PWM_LVL = 0;
//     _delay_ms(250);
//   }

//   // PWM_LVL = 
// }

// }}}

// {{{ Led Power

// Will sleep until button is pressed.
void LedPowerDoSleep() 
{
  WdtOff();

  PciOn();

  sleep_mode();

  PciOff();

  WdtOn();  
}

void LedPowerAutoSleep()
{
  static uint8_t  LedPowerOffTimer = 0;

  // Sleeps when led is off for some times.
  if (LedPowerStatus)
  { // Led is on
    LedPowerOffTimer = 0;
  }
  else
  { // Led is off

    // If the led is off more than a certain period of time,
    if (++LedPowerOffTimer >= LEDPOWEROFFTIMERMAX)
    { // Make a request so that the system will be put into sleep.
      LedPowerOffTimer        = 0;
      // switchShortReleaseTimer = 0;
      // sysPower                = 0; // The system sleep request.
      LedPowerDoSleep();
    }
  }
}

void LedPowerToggle()
{
  if (LedPowerStatus) // Now is on.
  { // Turn it off
    PWM_LVL = 0;
    LedPowerStatus = 0;
    if (LedModeIndex >= MODESHIGHINDEX) 
    {
      LedModeIndex = MODESNORMALINDEX;
    }
  }
  else                // Now is off.
  { // Turn it on
    PWM_LVL = LedModes[LedModeIndex];
    LedPowerStatus = 1;
  }
}

void LedPowerLowVoltageMonitor()
{
  static uint8_t   LedPowerLowVoltageTimer = 0;  
  
  // Voltage monitor
  if (ADCSRA & (1 << ADIF))  // See if conversion is done
  {
    // See if voltage is lower than what we were looking for
    if (ADCH < ((LedModeIndex == 0) ? ADC_CRIT : ADC_LOW)) 
    {
      ++LedPowerLowVoltageTimer;
    }
    else
    {
      // voltageLowCounter = 0;
      LedPowerLowVoltageTimer = 0;
    }
  }
      
  if (LedPowerLowVoltageTimer >= LEDPOWERLOWVOLTAGETIMERMAX)
  {
    if (LedModeIndex == 0) 
    { // Already in the lowest mode,
      // we can't make the mode step down further,
      // it is time to switch off.
      PWM_LVL        = 0;
      LedPowerStatus = 0;
    }
    else
    { // Step down the mode.
      PWM_LVL = LedModes[--LedModeIndex];
    }

    // if (LedModeIndex > 0) 
    // { // Step down the mode.
    //   PWM_LVL = LedModes[--LedModeIndex];
    // }
    // else


// { // Already in the lowest mode,
    //   // we can't make the mode step down further,
    //   // it is time to switch off.

    //   // ledPowerOffTimer        = 0;
    //   // switchShortReleaseTimer = 0;
    //   // sysPower                = 0; // The system sleep request.

    //   PWM_LVL        = 0;
    //   LedPowerStatus = 0;
    // }

    LedPowerLowVoltageTimer = 0;
  }
  // Make sure conversion is running for next time through
  ADCSRA |= (1 << ADSC);
}

// }}}

// {{{ Main

int main(void)
{       
    // Set all ports to input, and turn pull-up resistors on for the inputs we are using
    DDRB = 0x00;
    PORTB = (1 << SWITCHPIN) | (1 << STAR2PIN) | (1 << STAR3PIN);

    // Set the switch as an interrupt for when we turn pin change interrupts on
    PCMSK = (1 << SWITCHPIN);
        
    // Set PWM pin to output
    DDRB = (1 << PWMPIN);

    // Set timer to do PWM for correct output pin and set prescaler timing
    TCCR0A = 0x23; // phase corrected PWM is 0x21 for PB1, fast-PWM is 0x23
    TCCR0B = 0x01; // pre-scaler for timer (1 => 1, 2 => 8, 3 => 64, 4 => 256, 5 => 1024)
        
    // Turn features on or off as needed
    ACSR   |=  (1<<7); //AC off
        
    // Enable sleep mode set to Power Down that will be triggered by the sleep_mode() command.
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        
    PWM_LVL = 255;
    _delay_ms(750);
    PWM_LVL = 0;
    _delay_ms(750);

    PWM_LVL = LedModes[LedModeIndex];

    PciOff();
    AdcOn();
    WdtOn();

    while (1) 
    {
      cli();

      switch (SwitchEvent) 
      {
      case SWITCHEVENTSHORTPRESS:
        LedPowerToggle();
        break;
      case SWITCHEVENTSHORTPRESS2:
        LedModeHighEnter();
        break;
      // case SWITCHEVENTSHORTPRESS3:
      //   // LedModeIndex = MODESBLINKINDEX;
      //   if (LedPowerStatus == 1)
      //   { // Only allow programming mode,
      //     // when the flashlight is on,
      //     // and at certain mode (low or normal).
      //     LedModeProgramEnter();
      //   }
      //   else
      //   {
      //     ;
      //   }
      //   break;
      case SWITCHEVENTLONGPRESS:
        if (LedPowerStatus == 1) 
        { // Long press while flashlight is on.
          LedModeGoNext();
        }
        else
        { // Long press while flashlight is off.
          LedModeLowEnter();
        }
        break;
      }
      SwitchEvent = 0;
      LedModeHighAutoExit();
      LedPowerLowVoltageMonitor();
      LedPowerAutoSleep();
      // LedModeSos();
      // LedModeBlink();
      sei();
      _delay_ms(100);
    }
}

// }}}

// {{{ ELV

// Local Variables:
//     compile-command: "make" 
//     indent-tabs-mode:nil
//     folded:file:1
//     c-basic-offset:2
// End:
// 

// }}}


