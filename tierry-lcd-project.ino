//change charge pulse to 10 degrees from 30
// last change 4/21/2019
// 8/17 added code for pulse out width
// 7/14/2017 added correct math and setting for position of pickup
// 10 10/28/2016 adds charging pulse with Timer 2
// lowest RPM = 615 with charge pulse to keep OCR2A <= 255
// CDI Tester Pulse Generator Serial Output Arduino Code
// replaced on 7/22/18 mod on 4/18/2019 changed to go simple transformer control of pos / neg trigger
// update 7/14/2017 added correct math and setting for position of pickup
// revision 10 10/28/2016 adds charging pulse with Timer 2
// lowest RPM = 615 with charge pulse to keep OCR2A <= 255
// CDI Tester Pulse Generator Serial Output Arduino Code
// 8/17 added code for pulse out width

#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h> // include i/o class header
hd44780_I2Cexp lcd; // declare lcd object: auto locate & config display for hd44780 chip

int pot1 = A1; // select the input pin for the pot for rpm
int pot2 = A2; // select the input pin for the pot for pickup location in degrees
int potValue = 0; // variable to store the value coming from the sensor
int pickupValue = 0; // position of pickup in degrees
int timerTopValue = 12500; // changed from timerTopValue = 0
int outputPin = 4; // select the pin for the output for trigger pulse, changed from 8
int output2Pin = 5;
int chargePin = 7; // select the pin for the output for charge pulses

volatile boolean trigger = false;
volatile unsigned long delayPeriod;
unsigned long copy_delayPeriod;
volatile unsigned long delayPeriodStart;
float delayDegrees; // changed from int to float for decimal place display
int RPM;
int pickup;
int pulseWidth;
volatile boolean interruptFlag;
unsigned long analogReadInterval = 500; // read pots and map
unsigned long lastAnalogRead;
const byte setChargePulseSwitch = 6;
boolean chargePulse = false ;  // default dc powered mode
volatile byte timeSliceCount = 0; // TDC 0 degrees

void setup() {
// Serial.begin(115200);
// Serial.println("starting...");
  lcd.begin(20, 4); // initialize the lcd for 20 chars 4 lines, turn on backlight
  lcd.setCursor(1, 0); // lcd display setup of unchanging headings
  lcd.print("RPM:"); // print fixed characters
  lcd.setCursor(12, 0);
  lcd.print("Deg");
  lcd.setCursor(1, 1);
  lcd.print("Pos:");
  lcd.setCursor(1, 2);
  lcd.print("Us Delay:");
  lcd.setCursor(1, 3);
  lcd.print("Deg Advance:");

  pinMode(outputPin, OUTPUT); // declare the outputPin as an OUTPUT
  pinMode(output2Pin, OUTPUT);
  pinMode(chargePin, OUTPUT); // declare the chargePin as an OUTPUT
  pinMode(setChargePulseSwitch, INPUT_PULLUP);

  if (digitalRead(setChargePulseSwitch) == LOW)
    chargePulse = true; // AC CDI

 // Set up Timer1 (16bits) to give a .5ms trigger pulse every 50 ms
 // https://fr.wikiversity.org/wiki/Micro_contr%C3%B4leurs_AVR/Le_Timer_1
 // Mode PWM rapide sur ICR1 = mode N°14
 // TIMER1 est coupé en 2 registres TCCR1A & TCCR1B
  TCCR1A = (1<< WGM11) | (0 << WGM10);  // Set lower two bits of WGM to 2.
  TCCR1B = (1 << WGM13) | (1 << WGM12); // Set higher two bits of WGM to 8 + 4. Total WGM = 8+4+2=14.

  // La valeur MAXI de TIMER1 peut etre definie soit dans OCR1A soit dans ICR1
  // Si on utilise OCR1A comme TOP, on ne peut plus l'utiliser en mode PWM
  //OCR1A = timerTopValue;
  ICR1 = timerTopValue;   // TOP value in millisecond

  // Enable Timer overflow Interrupt
  TIMSK1 |= (1 << OCIE1A); // enable interrupt when counter reaches OCR1A value
  TIMSK1 |= (1 << OCIE1B); // enable interrupt when counter reaches OCR1B value?
  TIMSK1 |= (1<< TOIE1);//overflow interrupt at top

  // Section 15 du datasheet
  // Lorsque le timer TCNT1 attent les valeurs stockees dans OCR1A, une int est declenchée (car OCIE1A is set) et le flag OCF1A is set. 
  // Le resultat de la comparaison est utilisé egalement pour faire basculer la sortie PWM
  // OCR1A = duty cycle.
  OCR1B = 250; // sets trigger pulse width, 500 = 2Ms, 250 = 1Ms, 125 = .5Ms, set to 1250 for 5013 *********************************************
  // vitesse de TIMER1 = clock/prescaler = 16000000/64 = 250KHz
  // periode = 1/250000= 4usec
  // pour attendre 2ms, il faut attendre 2000us/4us= 500 ticks
  // OCR1A = clock/(prescaler * freq voulue) -1 = 16000000/64 x frq = frq = 1000Hz
  OCR1A = OCR1B +500;                     //add default
  TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10); // prescaler=64 ce qui fait 4us/tick
  // frequence PWM fast = Clock/prescaler/steps = 16000000/64/256 = 976Hz*256

// Timer2 default setup charge pulse 36 periods 10 degrees each; only 5 get turned on
// charge pulse timing interval = Timer1 trigger period/12 = timerTopValue/96
// le timer/compteur TCNT2 est comparé avec les registres OCR2A,B pour faire du PWM
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A = 1 << WGM20; // lsb of mode 7 pwm to OCR2A (Waveform Generation Mode)
  TCCR2B = 1 << WGM22; // msb of mode 7 pwm to OCR2A;
  //OCR2A = timerTopValue / 96; // 96 = 12*4*2 (#periods, prescaler difference, up/down timer mode )
  OCR2A = timerTopValue / 288; //  = 36*4*2 (#periods, prescaler difference, up/down timer mode )

  // Timer2 Interrupt MaSK
  TIMSK2 = 0;
  TIMSK2 = 1 << TOIE2; // enable overflow interrupt
// actually start timer in ISR(TIMER1_COMPA_vect
// to prevent out of sync charge pulse, no prescaler set here

  attachInterrupt(digitalPinToInterrupt(3), delayPeriodTiming, FALLING);

}

void loop()
{
  unsigned long analogReadInterval = 500; // read pots and map
  if (millis() - lastAnalogRead >= analogReadInterval)
  {
    lastAnalogRead += analogReadInterval;
    potValue = analogRead(pot1); // rpm
    pickupValue = analogRead(pot2); // pickup position

    if (digitalRead(setChargePulseSwitch) == LOW)
    chargePulse = true; // AC CDI
    else chargePulse = false;
   
// Timer2 OCR2A requires max value 255 => 615 lowest RPM
// if (chargePulse)
    {
      RPM = map(potValue, 0, 1023, 615, 10000);
      pickup = map(pickupValue, 0, 1023, 10, 75); // position so advance will read based on delay, 57 for yamaha 350 2 pu, 74 for 1 pu, 72 for tw200, 25 for 5013
      pulseWidth = (60000000/RPM)/360; // time for 1° in uS
    }

    timerTopValue = 15000000UL / RPM;
    OCR1B = pulseWidth;
    OCR1A = OCR1B + pulseWidth;

    lcd.setCursor(6, 0);
    lcd.print("     "); // print blank spaces to clear old data
    lcd.setCursor(6, 0);
    lcd.print(RPM);  // print rpm

    lcd.setCursor(16, 0);
    lcd.print("  ");
    lcd.setCursor(16, 0);
    lcd.print(pulseWidth, 1); // print us per degree
   
    lcd.setCursor(6, 1);
    lcd.print("   ");
    lcd.setCursor(6, 1);
    lcd.print(pickup); // print pickup coil position

    lcd.setCursor(11, 2);
    lcd.print("       ");
    lcd.setCursor(11, 2);
    lcd.print(copy_delayPeriod);
   
    lcd.setCursor(14, 3);
    lcd.print("      ");
    lcd.setCursor(14, 3);
    lcd.print(delayDegrees, 1);   // delayDegrees, 1);
  }
 
  if (trigger == true && interruptFlag == true )
  {
    trigger = false;
    interruptFlag = false;
    noInterrupts();
    copy_delayPeriod = delayPeriod;
    interrupts();

    delayDegrees = pickup - (360.0 * (copy_delayPeriod) / (timerTopValue * 4.0)); // for decimal place in deg display.
    }
}

//ISR(TIMER1_COMPA_vect) {
ISR(TIMER1_OVF_vect){
  //OCR1A = (timerTopValue); // value to set delay between pulses to trigger cdi
  ICR1 = (timerTopValue);
  digitalWrite(chargePin, LOW); // guarantee off charge pin at trigger
  digitalWrite(outputPin, HIGH); // turn on pin  trigger

  {
    delayPeriodStart = micros(); // start looking for response as pulse rises
    trigger = true;
  }
// start Timer 2 for charge pulses
  if (chargePulse)
  {
    timeSliceCount = 0;
    TCNT2 = 0;
    //OCR2A = timerTopValue/96; // set 12 periods
    OCR2A = timerTopValue/288;//set 36 periods
    TCCR2B |=  1 << CS22 | 1 << CS21; //prescaler 256 16us/tick
  }
}

ISR(TIMER1_COMPB_vect) {
  digitalWrite(outputPin, LOW);
  //could add short delayMiocroseconds here for gap
  digitalWrite(output2Pin,HIGH);

  //{
  //  delayPeriodStart = micros(); // start looking for response as pulse falls
  //  trigger = true;
  //}
}

ISR(TIMER1_COMPA_vect) {
  digitalWrite(output2Pin, LOW);
}

void delayPeriodTiming()
{
  delayPeriod = micros() - delayPeriodStart;
  interruptFlag = true;
}

ISR(TIMER2_OVF_vect)
// 5 pulses of 10 degrees starting at 60 degrees
// ON at 60,120,180,240,300 = slice count 6,12,18,24,30
// OFF at 70,130,190,250,310
{
  if (timeSliceCount != 0 && timeSliceCount % 6 == 0)
  {
    digitalWrite (chargePin, HIGH);
  }
  else
  {
    digitalWrite(chargePin, LOW);
  }
  timeSliceCount++;

  if (timeSliceCount == 36)
  {
    timeSliceCount = 0;
// stop Timer2 by clearing prescaler bits
    TCCR2B &= ~1<< CS22;
    TCCR2B &= ~1<< CS21;
  }
}
