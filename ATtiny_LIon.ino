/* ***************************************************
 *  ATtiny85 Li-Ion Battery Control -  This program implements an ATtiny85 based Lithium Battery Monitor
 *  -------------------------------    and protection circuit.  The battery and output voltages can be
 *                                     monitored with an optionally attached 16x2 LCD display (with I2C
 *                                     interface module).  The low voltage warning LED will begin flashing
 *                                     when the battery voltage approaches the cutoff voltage.  Once the
 *                                     battery voltage dips below the cutoff voltage, the ATtiny will cut
 *                                     the output voltage via the MOSFET switch.
 *                                     
 *  Notable Features:                  This code implements the Low Noise ADC mode of the ATtiny85 
 *                                     allowing for very accurate ADC readings.  Additionally, this code
 *                                     implements a power-down delay function (puts the microcontroller to
 *                                     sleep) to reduce the power usage of the monitor/protection circuit.
 *  
 *  ATtiny85 Pin Configuration
 *  -----------------------------------------------------
 *    ATtiny Pin 1 = n/c (Reset)
 *    ATtiny Pin 2 = ADC3 - Output Voltage 
 *    ATtiny Pin 3 = D4   - Output Enable/Disable
 *    ATtiny Pin 4 = GND  - Battery -
 *    ATtiny Pin 5 = SDA  - I2C Data (for LCD) 
 *    ATtiny Pin 6 = D1   - Low Voltage Warning LED
 *    ATtiny Pin 7 = SCK  - I2C Clock (for LCD)
 *    ATtiny Pin 8 = VCC  - Battery +
 *  -----------------------------------------------------
 *  
 *  This code is copyright (c) 2018 by Jeff Wahaus.  For non-comercial use only.
 */

#include <TinyWire.h>
#include <Tinytwi.h>
#include <LiquidCrystal_attiny.h>
#include <avr/sleep.h>

#define POWER_CUTOFF_VOLTAGE  2700    // Power cutoff at 2700mV (2.7V)
#define HYSTERESIS_VOLTAGE    300     // Voltage must exceed Cutoff + Hysteresis to Enable Power

#define BATTERY_VOLTAGE_CORR  (-40)   // in millivolts (Calibration adjustment - battery voltage)
#define OUTPUT_VOLTAGE_CORR   (-80)   // in millivolts (Calibration adjustment - output voltage)

#define USE_LCD_DISPLAY   1           // Change to 0 to not use LCD functionality

#define MOSFET_GATE_PIN   4
#define VOUT_SENSE_PIN    3
#define RED_LED_PIN       1
#define I2C_SCL_PIN       2
#define I2C_SDA_PIN       0

#define LOW_BATT_COUNT    5           // Low Battery count maximum allowed
int  low_battery_count    = 0;        // Low Battery Detected this many times (in a row)
bool output_is_off        = false;
bool Startup_Seq;

#if (USE_LCD_DISPLAY)
  LiquidCrystal_I2C lcd(0x27, 16, 2);  // set address, 16 x 2 lines
  bool Update_LCD = false;

// DEBUG
int i2c_error_ret = 0;
// DEBUG

  void LCD_Init() {
    pinMode(I2C_SCL_PIN, OUTPUT);
    pinMode(I2C_SDA_PIN, OUTPUT);
    TinyWire.begin();
    lcd.begin(16, 2);                 // initialize the lcd
    lcd.backlight();                  // turn on the backlight
    lcd.print("Tiny LIon - v1.1");
    delay(1500);
  }
#endif


// when ADC completed, take an interrupt but do nothing
EMPTY_INTERRUPT (ADC_vect);

// Read the ADC in Low Noise Mode (SLEEP_MODE_ADC)
//     - Note: The ADCMUX must be set prior to calling this function
int AnalogReadLowNoise( void )
{
  uint8_t adc_low;
  int adc_value;
  
  noInterrupts();
  // Enable Noise Reduction Sleep Mode
  set_sleep_mode(SLEEP_MODE_ADC);
  sleep_enable();

  // Enable ADC interrupts
  ADCSRA |= bit (ADIE);
  interrupts();
  sleep_cpu();                            // Sleep, ADC Start happens automatically
  sleep_disable();

  while (bit_is_set(ADCSRA, ADSC)) ;      // Make sure ADC conversion is done

  adc_low = ADCL;
  adc_value = (ADCH << 8) | adc_low;      // Read the ADC
 
  // Return the conversion result
  return(adc_value);
}


// Read and return the Vcc voltage in millivolts
int GetVcc() {
  volatile unsigned char save_ADMUX;
  uint8_t low;
  int i;
  unsigned int val, adc_sum;
    
  save_ADMUX = ADMUX;

  ADCSRA = 0x00;                                    // Turn off ADC to switch MUX
  ADMUX = _BV(MUX3) | _BV(MUX2);                    // Read internal 1V1 BandGap with Vcc Reference
  
  #if (F_CPU == 1000000L)                           // 1MHz CPU Clock
    ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0);   // Turn ON ADC with Divide by 8 Prescaler (ADC Clock = 125KHz)
  #endif
  #if (F_CPU == 8000000L)                           // 8MHz CPU Clock
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1);   // Turn ON ADC with Divide by 64 Prescaler (ADC Clock = 125KHz)
  #endif
  #if (F_CPU == 16000000L)                          // 16MHz CPU Clock
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);   // Turn ON ADC with Divide by 128 Prescaler (ADC Clock = 125KHz)
  #endif
  delay(4); // Wait for Vref to settle

  val = AnalogReadLowNoise();               // discard first reading

  adc_sum = 0;
  for (i = 0; i < 10; i++)                  // Average 10 readings
  {  
    adc_sum += AnalogReadLowNoise();        // get reading
    delay(1);
  }
  val = adc_sum / 10;                       // Average of 10 Readings
  
  ADMUX = save_ADMUX;

  return (((long)1024 * 1100) / val);  
}


unsigned int Read_ADC3_VccRef()
{
  volatile unsigned char save_ADMUX;
  int i;
  unsigned int val, adc_sum;

  save_ADMUX = ADMUX;

  ADCSRA = 0x00;                                    // Turn off ADC in order to switch MUX
  ADMUX = _BV(MUX1) | _BV(MUX0);                    // Select ADC3 with Vcc reference

  #if (F_CPU == 1000000L)                           // 1MHz CPU Clock
    ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0);   // Turn ON ADC with Divide by 8 Prescaler (ADC Clock = 125KHz)
  #endif
  #if (F_CPU == 8000000L)                           // 8MHz CPU Clock
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1);   // Turn ON ADC with Divide by 64 Prescaler (ADC Clock = 125KHz)
  #endif
  #if (F_CPU == 16000000L)                          // 16MHz CPU Clock
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);   // Turn ON ADC with Divide by 128 Prescaler (ADC Clock = 125KHz)
  #endif
  delay(4);                                         // Wait for Vref to settle

  val = AnalogReadLowNoise();                       // discard first reading

  adc_sum = 0;
  for (i = 0; i < 10; i++)                          // Average 10 readings
  {  
    adc_sum += AnalogReadLowNoise();                // get reading
    delay(1);
  }
  val = adc_sum / 10;
  ADMUX = save_ADMUX;                               // Restore ADMUX to previous state
  return(val);
}


bool Display_Initialized = false;

void Initialize_Display()
{
  #if (USE_LCD_DISPLAY)
    LCD_Init();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Battery: ");
    lcd.setCursor(0, 1);
    lcd.print("Output : ");
  #endif

  Display_Initialized = true;
}


// Enable_Watchdog(timeout) - Timeout in milliseconds.
// -----------------------    Supports timeout between 32 and 8000 ms (rounded up)
void Enable_Watchdog(unsigned int timeout) 
{
  unsigned int wd_time;
  byte bitmask;

  if (timeout <= 32)          // 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms
    wd_time = 1;              // 5=500ms, 6=1s, 7=2s, 8=4s, 9=8s
  else if (timeout <= 64)
    wd_time = 2;
  else if (timeout <= 125)
    wd_time = 3;
  else if (timeout <= 250)
    wd_time = 4;
  else if (timeout <= 500)
    wd_time = 5;
  else if (timeout <= 1000)
    wd_time = 6;
  else if (timeout <= 2000)
    wd_time = 7;
  else if (timeout <= 4000)
    wd_time = 8;
  else
    wd_time = 9;    // Use 8000ms

  bitmask = wd_time & 7;
  if (wd_time > 7)
    bitmask |= (1<<5);
  
  bitmask |= (1<<WDCE);

  noInterrupts();
  MCUSR &= ~(1<<WDRF);

  // start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);

  // set new watchdog wd_time value
  WDTCR = bitmask;
  WDTCR |= _BV(WDIE);
  interrupts();
}

void Disable_Watchdog()
{
  noInterrupts();
  __asm__ volatile ( "wdr" );
  /* Clear WDRF in MCUSR */
  MCUSR = 0x00;
  /* Write logical one to WDCE and WDE */
  WDTCR |= (1<<WDCE) | (1<<WDE);
  /* Turn off WDT */
  WDTCR = 0x00;
  interrupts();
}


volatile int wd_int = 0;  // set global flag

// Watchdog ISR
ISR(WDT_vect) {
  wd_int = 1;
}


//  Sleep_Delay() - Put CPU to sleep and turn off ADC (power save)
//  -------------   for delay_time milliseconds
void Sleep_Delay(unsigned int delay_time)    
{
  ADCSRA = 0x00;                        // Turn off the ADC

  Enable_Watchdog(delay_time);
  noInterrupts();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Set Power Down Mode
  sleep_enable();
  interrupts();
  sleep_cpu();                          // Go to Sleep

  sleep_disable();                      // Wake up happens here
  Disable_Watchdog();
}


// Its_a_good_time_to_die() - Shut down the CPU and don't wake up.
// ----------------------     Minimum power drain.

void Its_a_good_time_to_die()
{
  while (1)
  {
    noInterrupts();
    ADCSRA = 0x00;                        // Turn off the ADC
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Set Power Down Mode
    sleep_enable();
    sleep_cpu();                          // Go to Sleep
  }
}


void setup() {
  Startup_Seq = true;                     // Indicate Starting up
  pinMode(VOUT_SENSE_PIN, INPUT);
  pinMode(MOSFET_GATE_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(MOSFET_GATE_PIN, LOW);
  digitalWrite(RED_LED_PIN, HIGH);        // Flash the LED on Startup

  #if (USE_LCD_DISPLAY)
    pinMode(I2C_SCL_PIN, OUTPUT);
    pinMode(I2C_SDA_PIN, OUTPUT);
    TinyWire.begin();
  #else
    pinMode(I2C_SCL_PIN, INPUT);        // Not used so switch to high impedance inputs
    pinMode(I2C_SDA_PIN, INPUT);
  #endif    
}


void loop() {
  long Vcc_Voltage, Out_Voltage;
  long volts, mvolts;
  static bool Gate_Is_On = false;
  static unsigned int lcd_reset_count = 0;

  // Read the Vcc Voltage
  Vcc_Voltage = GetVcc();

  // Read the Output Voltage (8.1K + 2K Ohm Voltage Divider)  V=V*5.1
  Out_Voltage = ((((long)Read_ADC3_VccRef() * Vcc_Voltage) / 1023) * 51) / 10;  // Read and Scale Output (ADC3) Voltage

  // Adjust Voltage Readings verses actual measured values
  Vcc_Voltage += BATTERY_VOLTAGE_CORR;
  Out_Voltage += OUTPUT_VOLTAGE_CORR;

  #if (USE_LCD_DISPLAY)
    if (Update_LCD)
    {
      if (! Display_Initialized)
        Initialize_Display();

// DEBUG
#if 0   // used for debugging I2C errors
    if (i2c_error_ret != 0)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("I2Ce: ");
      lcd.print(i2c_error_ret);
      delay(5000);
      i2c_error_ret = 0;
    }
#endif    
// DEBUG
    

      // Format and display Battery Voltage (in Volts)
      lcd.setCursor(9, 0);
      volts = (Vcc_Voltage + 5) / 1000;   // Round up to Volta
      lcd.print(volts);
      lcd.print(".");
      mvolts = ((Vcc_Voltage - (volts * 1000)) + 5) / 10;
      if (mvolts == 100)
      {
        lcd.print("00");
      }
      else
      { 
        if (mvolts < 10)    // print leading zero if less than 10
          lcd.print("0");
        lcd.print(mvolts);
      }
      lcd.print("V ");
    
      // Format and display Output Voltage (in Volts)
      lcd.setCursor(9, 1);
      volts = (Out_Voltage + 5) / 1000;   // Round up to Volta
      lcd.print(volts);
      lcd.print(".");
      mvolts = ((Out_Voltage - (volts * 1000)) + 5) / 10;
      if (mvolts == 100)
      {
        lcd.print("00");
      }
      else
      { 
        if (mvolts < 10)    // print leading zero if less than 10
          lcd.print("0");
        lcd.print(mvolts);
      }
      lcd.print("V ");
    } // end if (Update_LCD)
  #endif

  #if (F_CPU == 1000000L)         // 1MHz CPU Clock delay (I2C takes longer)
    delay(400);                   // Give the LCD I2C time to transmit
  #else
    delay(100);                   // 8MHz CPU Clock delay
  #endif

  if (output_is_off && (Vcc_Voltage < (POWER_CUTOFF_VOLTAGE - 100)))
  {
    // Battery is 100mv below cutoff voltage, time to shut down for good, no LED blinks.
    // Note:  The voltage will jump up once the output is shut off.  LED flahing will indicate
    // low voltage.  The LED can likely flash for days (or weeks) after output is shut off.
    // This case is to use the absolute minimun current possible (i.e. Power down the Micro and
    // don't wake back up (no led flash) to keep from draining the battery further.
    
    Its_a_good_time_to_die();
  }
  else if (Vcc_Voltage < POWER_CUTOFF_VOLTAGE)  // Low Voltage - Shut off power
  {
    low_battery_count++;
    if ((low_battery_count >= LOW_BATT_COUNT) && Gate_Is_On)
    {
      output_is_off = true;
      digitalWrite(MOSFET_GATE_PIN, LOW);
      Gate_Is_On = false;
      #if (USE_LCD_DISPLAY)
        // Stop updating the LCD
        Update_LCD = false;
        // Prevent voltage leak through I2C pins
        pinMode(I2C_SCL_PIN, INPUT);
        pinMode(I2C_SDA_PIN, INPUT);
      #endif
    }
    // Flash the LED once every ~4 seconds
    digitalWrite(RED_LED_PIN, HIGH);
    delay(300);
    digitalWrite(RED_LED_PIN, LOW);
    Sleep_Delay(4000);
  }
  else if (Vcc_Voltage >= (POWER_CUTOFF_VOLTAGE + HYSTERESIS_VOLTAGE))  // Good Voltage - Power On
  {
    if (!output_is_off)     // Keep the output off, Voltage will jump up after shutdown
    {
      if (!Gate_Is_On)      // Turn the Output ON (if it's off)
      {
        digitalWrite(MOSFET_GATE_PIN, HIGH);
        Gate_Is_On = true;
        digitalWrite(RED_LED_PIN, LOW);
        delay(10);
        #if (USE_LCD_DISPLAY)
          Update_LCD = true;
        #endif
      }
    }
    else
    {
      // Slow Flash - Battery was switched off
      digitalWrite(RED_LED_PIN, HIGH);
      delay(300);
      digitalWrite(RED_LED_PIN, LOW);
      Sleep_Delay(4000);
    }
  }
  else // Between Power-cut and Power-Good (Battery Low), Blink LED
  {
    digitalWrite(RED_LED_PIN, HIGH);
    delay(400);
    digitalWrite(RED_LED_PIN, LOW);
  }

  // Limit ADC Reading / Read and Display at most once per 2 seconds
  //  except for the first time trough (faster startup)
  if (!Startup_Seq)
    Sleep_Delay(2000);
  else
    Startup_Seq = false;
    
  // Re-init the LCD every ~1 min (it ocassionally gets in a bad state - I2C errors)
  #if (USE_LCD_DISPLAY)
    lcd_reset_count++;
    if (lcd_reset_count > 30)
    {
      Display_Initialized = false;
      lcd_reset_count = 0;
    }
  #endif
  
} // end loop()
