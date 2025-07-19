//This code is written by OE3SDE, Simon Dorrer!

//Define CPU-Clock-Speed (16MHz internal clock)
//#define F_CPU 16000000UL

//Define Baudrate for UART
//#define BAUD 9600UL

//Include Libraries
#include <avr/io.h>
#include <stdio.h>
#include <inttypes.h>
#include <avr/interrupt.h>

//Libs. for SH1106 OLED-Display
#include <Wire.h>
#include <U8g2lib.h>

//Libs. for Keypad
#include <Keypad.h>

//Libs. for Si5351 Breakout Board
#include <si5351.h>
//----------------------------------------------------------------------------

//Defines

//I/O Declaration
//#define TX        0     //PD0 / 0
//#define RX        1     //PD1 / 1
#define K_C1        2     //PD2 / 2
#define K_C2        3     //PD3 / 3
#define K_C3        4     //PD4 / 4
#define K_C4        5     //PD5 / 5
#define K_R1        6     //PD6 / 6
#define K_R2        7     //PD7 / 7
#define K_R3        8     //PB0 / 8
#define K_R4        9     //PB1 / 9
#define K_R5        10    //PB2 / 10
#define PTT_TXPWR   3     //PB3 / 11
#define PTT_TXRX    4     //PB4 / 12
#define PTT_IN      5     //PB5 / 13
#define TX_LED      0     //PC0 / 14 (A0) - PCINT5
#define DIR         1     //PC1 / 15 (A1)
#define STEP        2     //PC2 / 16 (A2)
#define POTI_F      3     //PC3 / 17 (A3)
//#define SDA       4     //PC4 / 18 (A4)
//#define SCK       5     //PC5 / 19 (A5)

//Defines for SH1106 Display
U8X8_SH1106_128X64_NONAME_HW_I2C SH1106(/* reset=*/ U8X8_PIN_NONE);

//Defines for Keypad
const byte ROWS = 5; // five rows
const byte COLS = 4; // four columns

char keys[ROWS][COLS] =
{
{'X','Y','#', '*' },       //X=F1, Y=F2
{'1','2','3', 'U' },       //U=up
{'4','5','6', 'D' },       //D=down
{'7','8','9', 'E' },       //E=Esc
{'L','0','R', 'O' }        //L=left, R=right, O=Ent
};

byte rowPins[ROWS] = {K_R5, K_R4, K_R3, K_R2, K_R1};
byte colPins[COLS] = {K_C1, K_C2, K_C3, K_C4};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

char KEY[4] = {'1','5','0','5'};
char attempt[4] = {0,0,0,0};
int z=0;
int keystate = 0;                     // 0 = wrong key, 1 = right key

//Defines for Si5351 Breakout Board
Si5351 si5351;
//----------------------------------------------------------------------------

//Global variables
uint8_t firstLoop = 1;

//OLED Display variables
uint8_t updateDirOnDisplay = 1;
uint8_t updateDataOnDisplay = 1;
uint8_t updateFrequencyOnDisplay = 1;
uint8_t updateStepOnDisplay = 1;
uint8_t updatePTTOnDisplay = 1;
uint8_t USB = 1;                              //USB = 1... Upper Side Band is used!, USB = 0... Lower Side Band is used!

//UART variables
char readData = '0';                          //character for receiving data;

//ADC variables
int POTI_F_Value_new = 0;
int POTI_F_Value_old = 0;

//PTT variables
uint8_t PTT_IN_state = 2;                     //No state at the beginning, 1 = TX, 0 = RX

//Si5351 variables
uint8_t setSteps = 0;                         //set Frequency with Steps
uint8_t stepDir = 1;                          //1... up, 0... down;
uint8_t setPreciseFrequency = 0;              //set Frequency precise with Numpad

unsigned long f_cor = 63000;                  //63000
unsigned long f_step = 50;                    // set this to your wanted tuning rate in Hz.
unsigned long f_bandStart = 14000000;         // start of 20m (14MHz - 14.070MHz CW, 14.112MHz - 14.350MHz USB Voice)
unsigned long f_bandEnd = 14350000;           // end of 20m
unsigned long f_bandInit = 14200000;          // where to initially set the frequency
unsigned long f_BFOTX = 8998600;              // TX BFO frequency
unsigned long f_BFORX = 8999300;              // RX BFO frequency
unsigned long f_VFORX = f_bandInit - f_BFORX; // RX VFO frequency
unsigned long f_VFOTX = f_bandInit - f_BFOTX; // TX VFO frequency
unsigned long f_old = f_bandInit;
unsigned long f_new = f_bandInit;

//Stepper variables
unsigned long f_stepper = f_bandInit;
unsigned long f_perStep = 10000;         //Antenna changes XYZkHz per 1/16 step!
unsigned long f_difStepper = f_new - f_bandInit;
uint16_t motorSteps = 0;
//----------------------------------------------------------------------------

//Subprograms
//IO Subprograms
void IO_Init()                  //initialize IO
{
  //Define INPUTs
  DDRB &= ~(1 << PTT_IN);       //set PB5 (PTT_IN) as Input
  PORTB |= (1 << PTT_IN);       //activate Pull-Up-R at PB5 (PTT_IN)     
  
  DDRC &= ~(1 << POTI_F);       //set PA3 (POTI_F) as Input
  
  //Define OUTPUTs
  DDRB |= (1 << PTT_TXPWR);     //set PB3 (PTT_TXPWR) as Output
  //PORTB &= ~(1 << PTT_TXPWR);   //set PB3 (PTT_TXPWR) LOW at the beginning
  PORTB |= (1 << PTT_TXPWR);    //set PB3 (PTT_TXPWR) HIGH (inverted Logic) at the beginning --> PA off

  DDRB |= (1 << PTT_TXRX);      //set PB4 (PTT_TXRX) as Output
  PORTB &= ~(1 << PTT_TXRX);    //set PB4 (PTT_TXRX) LOW at the beginning

  DDRC |= (1 << TX_LED);        //set PA0 (TX_LED) as Output
  PORTC &= ~(1 << TX_LED);      //set PA0 (TX_LED) LOW at the beginning

  DDRC |= (1 << DIR);           //set PA1 (DIR) as Output
  PORTC &= ~(1 << DIR);         //set PA1 (DIR) LOW at the beginning

  DDRC |= (1 << STEP);          //set PA2 (STEP) as Output
  PORTC &= ~(1 << STEP);        //set PA2 (STEP) LOW at the beginning
}

void PCINT5_Init() //PCINT0-PCINT7
{
  PCICR |= (1 << PCIE0);    //Enable PCINT0-PCINT7
  PCMSK0 |= (1 << PCINT5);  //Enable PCINT5 --> PB5
}

//Timer1 OCR1A Interrupt
void Timer1_OCR1A_Init()
{
  //Control Registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;
  
  // set CTC mode
  //TCCR1A |= (1 << WGM10);
  //TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12);
  
  // Set Prescaler 102
  TCCR1B |= (1 << CS10);
  //TCCR1B |= (1 << CS11);
  TCCR1B |= (1 << CS12);
  
  // initialize compare value (INT. every 200ms)
  OCR1A = 3125;
  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}

//ADC Subprograms
void ADC_Init(void)
{
  //ADC Setup
  //Set Reference Voltage (VCC = VREF)
  ADMUX &= ~(1<<REFS1);
  ADMUX |= (1<<REFS0);
  
  //Prescaler --> 128 (50kHz - 200kHz)
  ADCSRA |= (1<<ADPS0);
  ADCSRA |= (1<<ADPS1);
  ADCSRA |= (1<<ADPS2);
  
  ADCSRA |= (1<<ADEN); //Enable ADC
}
int ReadAnalogValue(int Pin)            // Read ADC Value
{
  ADMUX = (1<<REFS0) | (Pin & 0x0f);    //select input and ref
  ADCSRA |= (1<<ADSC);                  //Start ADC conclusion
  while (ADCSRA & (1<<ADSC)){}          //wait until ADC is ready
  return ADC;                           //return the ADC value
}

//UART Subprograms
void UART_Init()                  //initialize UART
{
  //initial state
  UCSR0A = 0;
  UCSR0B = 0;
  UCSR0C = 0;
  
  //Set Baudrate --> UBRR0_VALUE = ((F_CPU/(16*BAUD)) - 1) --> Baudrate = 9600
  UBRR0H = 0x00;
  UBRR0L = 103;                 //Baudrate calculation on page 173 of ATmega328 datasheet
    
  //UCSR0B Register
  //Enable USART RX-Interrupt
  UCSR0B |= (1 << RXCIE0);
    
  //Enable USART Receive and Transmit
  UCSR0B |= (1 << TXEN0);
  UCSR0B |= (1 << RXEN0);
    
  //UCSR0A Register
  //Asynchronous Mode, Parity Disabled, 8 Bit character size, Transmitted Data Changed (rising edge), Received Data Sampled (falling edge)
  UCSR0C |= (1 << UCSZ00);
  UCSR0C |= (1 << UCSZ01);
}
int UART_putchar(char c, FILE *stream)
{
  while(!((UCSR0A) & (1 << (UDRE0))));      //loop until bit UDRE0 is set in register UCSR0A (If UDRE0 is one, the buffer is empty, and therefore ready to be written.)
  UDR0 = c;
  return(0);
}
int UART_getchar(FILE *stream)
{
  while(!((UCSR0A) & (1 << (RXC0))));       //loop until bit RXC0 is set in register UCSR0A (If RXC0 is one, the receiver buffer contains unread data.)
  return(UDR0);
}

//SH1106 OLED-Display Subprograms
void SH1106_setMenu(unsigned long f, unsigned long fstep, char BT)
{
  SH1106.clear();
  SH1106.drawString(1, 1, "Frequency: ");
  if(stepDir == 1)
  {
    SH1106.drawString(12, 1, "UP");
  }
  else if(stepDir == 0)
  {
    SH1106.drawString(12, 1, "DOWN");
  }

  SH1106.setCursor(1, 3);
  SH1106.print(f);  
  SH1106.drawString(9, 3, "Hz");

  SH1106.drawString(13, 3, "20m");
  
  SH1106.drawString(1, 5, "f-Step:  ");
  SH1106.setCursor(9, 5);
  SH1106.print(fstep);  
  SH1106.drawString(11, 5, "Hz");
  
  SH1106.drawString(1, 7, "BT:");
  SH1106.setCursor(5, 7);
  SH1106.print(BT); 

  if(USB == 1)
  {
    SH1106.drawString(8, 7, "USB");
  }
  else
  {
    SH1106.drawString(8, 7, "LSB"); 
  }

  if(PTT_IN_state == 1)
  {
    SH1106.drawString(13, 7, "TX");
  }
  else if(PTT_IN_state == 0 || PTT_IN_state == 2)
  {
    SH1106.drawString(13, 7, "RX");
  }
}

void SH1106_update(unsigned long f, unsigned long fstep, char BT)
{
  if(updateDirOnDisplay == 1)
  {
    if(stepDir == 1)
    {
      SH1106.drawString(12, 1, "     ");
      SH1106.drawString(12, 1, "UP");
    }
    else if(stepDir == 0)
    {
      SH1106.drawString(12, 1, "     ");
      SH1106.drawString(12, 1, "DOWN");
    } 
    updateDirOnDisplay = 0;
  }

  if(updateFrequencyOnDisplay == 1 && setSteps == 0)
  {
    SH1106.drawString(1, 3, "        ");
    SH1106.drawString(8, 5, " ");
    SH1106.setCursor(1, 3);
    SH1106.print(f);  
    SH1106.drawString(11, 3, "-");
    updateFrequencyOnDisplay = 0;
  }

  if(updateStepOnDisplay == 1 && setSteps == 1)
  {
    SH1106.drawString(8, 5, "-");
    SH1106.drawString(11, 3, " ");
    SH1106.drawString(9, 5, "       ");
    SH1106.setCursor(9, 5);
    SH1106.print(fstep);
    switch(fstep)
    {
      case 50:
        SH1106.drawString(11, 5, "Hz");
      break;
      case 100:
        SH1106.drawString(12, 5, "Hz");
      break;
      case 500:
        SH1106.drawString(12, 5, "Hz");
      break;
      case 1000:
        SH1106.drawString(13, 5, "Hz");
      break;
        case 5000:
        SH1106.drawString(13, 5, "Hz");
      break;
        case 10000:
        SH1106.drawString(14, 5, "Hz");
      break;
      default:
      break;
    }
    updateStepOnDisplay = 0;
  }

  if(updateDataOnDisplay == 1)
  {
    SH1106.drawString(7, 7, " ");
    SH1106.setCursor(5, 7);
    SH1106.print(BT); 
    updateDataOnDisplay = 0;
  }
  if(updatePTTOnDisplay == 1)
  {
    if(PTT_IN_state == 1)
    {
      SH1106.drawString(13, 7, "TX");
    }
    else if(PTT_IN_state == 0)
    {
      SH1106.drawString(13, 7, "RX");
    }
    updatePTTOnDisplay = 0;
  }
}
    
//Keypad Subprogams
void noPW()
{ 
  delay(1000);
  SH1106.clear();
  SH1106.drawString(2, 4, "KEY ACCEPTED!");
  printf("KEY ACCEPTED!\n\r");
  delay(2000);
  //ClearDisplay();
  SH1106.clear();
  keystate = 1;
}
void checkKEY()
{
  int correct=0;
  int i;
  for ( i = 0; i < 4 ; i++ )
  {
    if (attempt[i] == KEY[i])
    {
       correct++;
    }
  }
  if (correct == 4)
  {
    SH1106.clear();
    SH1106.drawString(2, 4, "KEY ACCEPTED!");
    printf("KEY ACCEPTED!\n\r");
    delay(2000);
    SH1106.clear();
    //ClearDisplay();
    keystate = 1;
    firstLoop = 1;
  }
  else
  {
    SH1106.clear();
    SH1106.drawString(2, 4, "KEY REJECTED!");
    printf("KEY REJECTED!\n\r");
    delay(2000);
    
    SH1106.clear();
    SH1106.drawString(0, 3, "Enter Password:");
    keystate = 0;
  }
  for (int zz = 0; zz < 4; zz++) // clear previous key input
  {
    attempt[zz] = 0;
  }
}
void readKeypad()
{
  char key = keypad.getKey();
  //printf("%c \n\r", key);
   
  if (key != NO_KEY)
  {
    if(keystate == 0)
    {
      switch(key)
      {
      case '*':
        z = 0;
        SH1106.drawString(2, 5, ".");
      break;
      case '#':
        if(z == 4)
        {
          SH1106.drawString(2, 5, ". * * * * .");
        }
        SH1106.drawString(2, 5, ".");
        delay(100); // added debounce
        checkKEY();
      break;
      case 'E':
        SH1106.drawString(2, 5, ". * * * * .");
        noPW();
      break;
      default:
        attempt[z] = key;
        z++;
        switch(z)
        {
          case 1:
            SH1106.drawString(2, 5, ". *");
          break;
          case 2:
            SH1106.drawString(2, 5, ". * *");
          break;
          case 3:
            SH1106.drawString(2, 5, ". * * *");
          break;
          case 4:
            SH1106.drawString(2, 5, ". * * * *");
          break;
        }    
      }
    }
    
    if(keystate == 1)
    {
      switch(key)
      {
      case 'O':
        f_old = f_new;
      break;
      case 'U':
        stepDir = 1;
        updateDirOnDisplay = 1;
      break;
      case 'D':
        stepDir = 0;
        updateDirOnDisplay = 1;
      break;
      case 'X':
        if(setSteps == 1)
        {
          setSteps = 0;
        }
        else if(setSteps == 0)
        {
          setSteps = 1;
        }
        setPreciseFrequency = 0;
      break;
      case 'Y':
        setPreciseFrequency = 1;
      break;
      /*case '*':
      break;
      case '#':
      break;*/
      case 'E':
        f_new = f_bandInit;
        f_old = f_bandInit;
      break;
      default:
        printf("Default!");
      }
    }
  }
}

//Si5351 + Stepper Driver Subprograms
void setFrequencySteps(int POTI_F_Value)
{
  if(setSteps == 1)
  {
    f_old = f_new;
    if(POTI_F_Value >= 0 && POTI_F_Value < 180)
    {
      f_step = 50;
      printf("Step: 50Hz");
    }
    else if(POTI_F_Value >= 180 && POTI_F_Value < 360)
    {
      f_step = 100;
      printf("Step: 100Hz");
    }
    else if(POTI_F_Value >= 360 && POTI_F_Value < 540)
    {
      f_step = 500;
      printf("Step: 500Hz");
    }
    else if(POTI_F_Value >= 540 && POTI_F_Value < 720)
    {
      f_step = 1000;
      printf("Step: 1kHz");
    }
    else if(POTI_F_Value >= 720 && POTI_F_Value < 900)
    {
      f_step = 5000;
      printf("Step: 5kHz");
    }
    else if(POTI_F_Value >= 900 && POTI_F_Value < 1024)
    {
      f_step = 10000;
      printf("Step: 10kHz");
    }
  }
  else if(setSteps == 0)
  {
    if(stepDir == 1)
    {
      f_new = f_old + (f_step*(POTI_F_Value/20));
    }
    else if(stepDir == 0)
    {
      f_new = f_old - (f_step*(POTI_F_Value/20));
    }
    
    if(f_new < f_bandStart)
    {
      f_new = f_bandStart;
    }
    if(f_new > f_bandEnd)
    {
      f_new = f_bandEnd;
    }
    /*printf("f_step: %lu Hz \n\r", f_step);
    printf("f_old: %lu Hz \n\r", f_old);
    printf("f_VFORX: %lu Hz \n\r", f_new-f_BFORX);
    printf("f_VFOTX: %lu Hz \n\r", f_new-f_BFOTX);
    printf("f_new: %lu Hz \n\r", f_new);*/
  }
}

void setStepperDriver(unsigned long f)  //Driver operates in "sixteenth step mode"!
{
  //Example: f = 14MHz, f_stepper = 14.2MHz --> f_difStepper = 200kHz, --> motorSteps = 20 @ f_perStep = 10kHz
  if(f_stepper > f)
  {
    PORTC |= (1 << DIR);            //set PA1 (DIR) HIGH (clockwise)
    f_difStepper = f_stepper - f;
  }
  else if(f_stepper < f)
  {
    PORTC &= ~(1 << DIR);           //set PA1 (DIR) LOW (counter clockwise)
    f_difStepper = f - f_stepper;
  }

  motorSteps = f_difStepper / f_perStep;

  /*printf("f: %lu Hz \n\r", f);
  printf("f_stepper: %lu Hz \n\r", f_stepper);
  printf("f_difStepper: %lu Hz \n\r", f_difStepper);
  printf("f_stepper: %lu Hz \n\r", f_perStep);
  printf("motorSteps: %d \n\r", motorSteps);*/
  
  for(int m = 0; m < motorSteps; m++) 
  { 
    PORTC |= (1 << STEP);        //set PA2 (STEP) HIGH
    delayMicroseconds(250);  
    PORTC &= ~(1 << STEP);        //set PA2 (STEP) LOW
    delayMicroseconds(250); 
  }
  
  f_stepper = f;
  f_difStepper = 0;
}

void setFrequency(unsigned long f)
{
  //configure Stepper Driver (Capacitor on Magnetic Loop Antenna)
  setStepperDriver(f);
  
  //configure Si5351 CLKs
  f_VFORX = f - f_BFORX;
  f_VFOTX = f - f_BFOTX;

  if(PTT_IN_state == 1) //TX
  {
    si5351.set_clock_pwr(SI5351_CLK1, 0);
    si5351.set_clock_pwr(SI5351_CLK0, 1);
    si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);   // 280mV amplitude for SA612
    si5351.set_freq((f_VFOTX * 100ULL), SI5351_CLK0);
    si5351.set_freq((f_BFOTX * 100ULL), SI5351_CLK2);
  }
  else //RX
  {
    si5351.set_clock_pwr(SI5351_CLK0, 0);
    si5351.set_clock_pwr(SI5351_CLK1, 1);
    si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);   // 280mV amplitude for SA612
    si5351.set_freq((f_VFORX * 100ULL), SI5351_CLK1);
    si5351.set_freq((f_BFORX * 100ULL), SI5351_CLK2);
  }
  
}

//PTT Subprograms
void setPTT()
{
  if(PTT_IN_state == 1)
  {
    PORTC |= (1 << TX_LED);           //set PA0 (TX_LED) HIGH
    PORTB |= (1 << PTT_TXRX);         //set PB4 (PTT_TXRX) HIGH (RX to GND, TX to ANT.)
    delay(500);                       //wait 100ms (let TXRX Relais settle)
    //PORTB |= (1 << PTT_TXPWR);      //set PB3 (PTT_TXPWR) HIGH --> turn on PA
    PORTB &= ~(1 << PTT_TXPWR);       //set PB3 (PTT_TXPWR) LOW (inverted Logic) --> turn on PA
  }
  else if(PTT_IN_state == 0)
  {
    //PORTB &= ~(1 << PTT_TXPWR);     //set PB3 (PTT_TXPWR) LOW --> turn off PA
    PORTB |= (1 << PTT_TXPWR);        //set PB3 (PTT_TXPWR) HIGH (inverted Logic) --> turn off PA
    delay(500);                       //wait 100ms (let TX_PWR Relais settle)
    PORTB &= ~(1 << PTT_TXRX);        //set PB4 (PTT_TXRX) LOW (RX to ANT., TX to GND)
    PORTC &= ~(1 << TX_LED);          //set PA0 (TX_LED) LOW
  }
}
//----------------------------------------------------------------------------

//Setup (Initialize hardware)
void setup()
{
  //Init IO
  IO_Init();

  //Init Timer1
  Timer1_OCR1A_Init();

  //Init PCINT5
  PCINT5_Init();
  
  //Init UART
  UART_Init();
  fdevopen(UART_putchar, UART_getchar);          //define printf
  
  //Init ADC
  ADC_Init();

  //Init I2C
  Wire.begin();
  
  //enable global interrupts
  sei();

  //Init SH1106 OLED-Display
  SH1106.begin();
  SH1106.clear();
  SH1106.setFont(u8x8_font_amstrad_cpc_extended_r);
  
  //Set start Text
  SH1106.drawString(1, 1, "14MHz SSB TRX");
  SH1106.drawString(1, 4, "by OE3SDE");
  SH1106.drawString(1, 7, "Simon Dorrer");
  printf("14MHz SSB TRX\n\r");
  printf("by OE3SDE\n\r");
  printf("Simon Dorrer\n\r");

  delay(2000);
  
  //Init Si5351 Breakout Board
  //https://github.com/etherkit/Si5351Arduino
  bool i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  if(i2c_found == false)
  {
    SH1106.clear();
    SH1106.drawString(3, 4, "No Si5351!");
    printf("No Si5351!\n\r");
  }
  else
  {
    printf("Si5351 found!\n\r");

    //Set Up Si5351 Settings
    si5351.set_correction(f_cor, SI5351_PLL_INPUT_XO);      // Set to specific Si5351 calibration number
    si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    
    // 8mA=880mV, 6mA=680mV, 4mA=480mV, 2mA=280mV
    si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);   // 280mV amplitude for SA612
    si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);   // 280mV amplitude for SA612
    si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_4MA);   // 480mV amplitude for MC1496

    //Query a status update and wait a bit to let the Si5351 populate the status flags correctly.
    si5351.update_status();
    
    //Set Password
    delay(2000);
    SH1106.clear();
    SH1106.drawString(0, 3, "Enter Password:");
    printf("Enter Password!\n\r");
  }
}
//----------------------------------------------------------------------------

//Loop
void loop()
{
  if(keystate == 0)
  {
    readKeypad();
  }
  else if(keystate == 1)
  {
    if(firstLoop == 1)
    {
      SH1106_setMenu(f_bandInit, f_step, readData);
    }
    else
    {
      SH1106_update(f_new, f_step, readData);
      setPTT();
      setFrequency(f_new);
    }
    firstLoop = 0;
  }
  delay(100);
}
//----------------------------------------------------------------------------

//ISP (Interrupt Service Routine) 
//Timer1 COMPA ISR
ISR(TIMER1_COMPA_vect)
{
  if(keystate == 1)
  {
    readKeypad();
    
    POTI_F_Value_new = ReadAnalogValue(POTI_F);
    if((POTI_F_Value_new >= (POTI_F_Value_old + 5)) || (POTI_F_Value_new <= (POTI_F_Value_old - 5)))
    {
      updateFrequencyOnDisplay = 1;
      updateStepOnDisplay = 1;
    }
    POTI_F_Value_old = POTI_F_Value_new;
    
    setFrequencySteps(POTI_F_Value_new);
  }
}

//UART receive ISR
ISR(USART_RX_vect)                      //If data from PC is received completely of UART interface, an interrupt occurs.
{
  updateDataOnDisplay = 1;
  if(keystate == 1)
  {
    readData = UDR0;
   
    switch(readData)                    //evaluate the received data
    {
      case 'a':
        printf("a\n\r");
      break;
      case 'b':
        printf("b\n\r");
      break;
      default:
        //printf("Keine richtige Anweisung!\n\r");
      break;
    }
  }
  else
  {
    printf("No Password is entered!\n\r");
  }
}

//PTT
//PCINT0-7 ISR
ISR(PCINT0_vect)
{
  if(keystate == 1)
  {
    updatePTTOnDisplay = 1;
    if(!(PINB & (1 << PTT_IN)))         //PTT_IN changed to LOW
    {
      printf("PTT_IN_LOW\n\r"); 
      PTT_IN_state = 1;
    }
    else if (PINB & (1 << PTT_IN))      //PTT_IN changed to HIGH
    {
      printf("PTT_IN_HIGH\n\r");
      PTT_IN_state = 0;
    }
  }
}
//----------------------------------------------------------------------------
