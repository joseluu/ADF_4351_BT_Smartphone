#include <rom/rtc.h>
#include <SPI.h>
#include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`
#include "EEPROM.h"
#include "BluetoothSerial.h"
#include <JsonParser.h>
#include <driver/dac.h>

SSD1306Wire display(0x3c, 4, 15);
char message[100];
BluetoothSerial SerialBT;


#define USE_SERIAL Serial

#define ADF4351_LE_PIN 21  // load enable pin 3 does not work
#define ADF4351_LE ADF4351_LE_PIN
#define ADF4351_PLL_LOCKED_PIN 2
#define SYNC_PIN 22 // for timing debug
#define SYNC2_PIN 12 // for timing debug
#define SYNC3_PIN 27 // for timing debug

#define REF_CLK   10 // local clock or extern OCXO MHz
static int referenceFrequency = REF_CLK;

/* SPI on ESP32
 *  SCK GPIO18   -> CLOCK 4351
 *  MISO  GPIO19
 *  MOSI  GPIO23  -> DATA 4351
 *  SS  GPIO5
 */
class CSweepParameters {
  public:
  char name[64];
  bool sweepOn;
  int refClk;
  int current;
  int start;
  int stop;
  int step;
  int timeStep;
};

CSweepParameters g_sweepParameters;

EEPROMClass  parametersStorage("eeprom", 0x500);

static unsigned long Register_Buf[6];
static unsigned long Register_Previous[6];
static unsigned int Fraction, Integer;
static unsigned char Reg_Buf[6];
static int RF_Fre_Value;
static unsigned long Start_Fre_value;
static unsigned long Stop_Fre_value;
static unsigned long Delta_Fre_value;
static unsigned long Sweep_Time_value;
static unsigned long SweepCurrentFreq;
bool sweepFlag;
SemaphoreHandle_t xSweepTimerSemaphore;
double percentSweep = 0;
double percentSweepIncrement = 0;

#define _SYNC(val)   digitalWrite(SYNC_PIN, val)
#define _SYNC2(val)   digitalWrite(SYNC2_PIN, val)
#define _SYNC3(val)   digitalWrite(SYNC3_PIN, val)

void IRAM_ATTR onTimer(){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR( xSweepTimerSemaphore, &xHigherPriorityTaskWoken );
}
hw_timer_t * timer = NULL;
void setupTimer()
{
    // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);
}

void SweepTimerProcessing( void * parameter )
{
  Serial.println((char *)parameter);
  /* loop forever */
  for(;;){
    /* task move to Block state to wait for interrupt event */
    xSemaphoreTake( xSweepTimerSemaphore, portMAX_DELAY ); // wait forever when INCLUDE_vTaskSuspend==1
    SweepTimerTick();
  }
  vTaskDelete( NULL );
}

void setup() {
  /*
   * Initialize display
   */
  pinMode(16,OUTPUT); digitalWrite(16,LOW); delay(50); digitalWrite(16,HIGH); 
  display.init();
  display.clear();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  /*
   * Initialize serial and BT
   */
  Serial.begin(115200);
  delay(2000);

  Serial.println("CPU0 reset reason: ");
  print_reset_reason(rtc_get_reset_reason(0));

  Serial.println("CPU1 reset reason: ");
  print_reset_reason(rtc_get_reset_reason(1));

  if (!parametersStorage.begin(parametersStorage.length())) {
    Serial.println("Failed to initialise parametersStorage");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }

  parametersStorage.get(0,g_sweepParameters);
  if (g_sweepParameters.current < 35000 || g_sweepParameters.current > 4500000) {
    initializeSweepParameters(); // restore defaults
    parametersStorage.put(0,g_sweepParameters);
    parametersStorage.commit();
  }
  
  SerialBT.begin(g_sweepParameters.name); //Bluetooth device name
  Serial.println("Device started, now you can pair it with bluetooth!");

  pinMode(SYNC_PIN, OUTPUT);  
  pinMode(SYNC2_PIN, OUTPUT);  
  pinMode(SYNC3_PIN, OUTPUT);  

  dac_output_enable(DAC_CHANNEL_1); // pin 25
  dac_output_voltage(DAC_CHANNEL_1, 0);

 
  ADF4351_Init(referenceFrequency);

  xSweepTimerSemaphore = xSemaphoreCreateBinary();
  /* this task will process the interrupt event 
  which is forwarded by interrupt callback function */
  xTaskCreate(
    SweepTimerProcessing,           /* Task function. */
    "SweepTimerProcessing",        /* name of task. */
    1000,                     /* Stack size of task */
    NULL,                     /* parameter of the task */
    4,                        /* priority of the task */
    NULL);  
  setupTimer();

  delayMicroseconds(2);
  _SYNC2(0);
  SetCurrentFrequency(g_sweepParameters.current);
  RF_OUT();
  if (g_sweepParameters.sweepOn) {
    StartSweep(  g_sweepParameters.start ,g_sweepParameters.stop ,g_sweepParameters.timeStep,g_sweepParameters.step);
  }
}

void doCommand(CSweepParameters sweepParameters){
  strcpy(sweepParameters.name,g_sweepParameters.name);
  parametersStorage.put(0,sweepParameters);
  parametersStorage.commit();
  if (sweepFlag) {
    SweepTimerStop();
  }
  dac_output_voltage(DAC_CHANNEL_1, 0);
  SetCurrentFrequency(sweepParameters.current);
  RF_OUT();
  if (sweepParameters.sweepOn) {
    StartSweep(  sweepParameters.start ,sweepParameters.stop ,sweepParameters.timeStep,sweepParameters.step);
  }
}

char * read_from_BT() {
  #define BT_BUFFER_SIZE 200
  static char buffer[BT_BUFFER_SIZE];
  bool go=true;
  int i=0;
  char c;
  while (go){
    if (SerialBT.available()){
      c=SerialBT.read();
      buffer[i++]=c;
    }
    if (c==0) {
      return buffer;
    }
    if (c=='\n' && i>0) {
      buffer[i-1]='\0';
      return buffer;
    }
    delay(2);
  }
}

JsonParser<100> parser;

void loop() {
  displayStatus();
  delay(100);
  if (SerialBT.available()) {
    char * command=read_from_BT();
    Serial.println("Received from bluetooth");
    Serial.println(command);
    //char * nl=strchr(command,'\n');
    //if (NULL != nl && nl < command+BT_BUFFER_SIZE){
    //  *nl='\0';
    //}
    JsonHashTable parametersHash = parser.parseHashTable(command);
    if (!parametersHash.success())
    {
           Serial.println("parsing failed");
    } else {
      CSweepParameters sweepParameters;
      strcpy(sweepParameters.name,parametersHash.getString("Name"));
      sweepParameters.sweepOn=parametersHash.getBool("sweep");
      sweepParameters.refClk=parametersHash.getLong("refClk");
      sweepParameters.current=parametersHash.getDouble("current")*1000;
      sweepParameters.start=parametersHash.getDouble("start")*1000;
      sweepParameters.stop=parametersHash.getDouble("stop")*1000;
      sweepParameters.step=parametersHash.getLong("step");
      sweepParameters.timeStep=parametersHash.getLong("timeStep");
      doCommand(sweepParameters);
        Serial.println("command executed");
    }
  }
  return;
  for (int i=0;i<6;i++) {
    Register_Previous[i]=0;
  }
  RF_OUT();
  delayMicroseconds(100);
}

void displayStatus()
{
// this function takes 3ms
    display.clear();
    sprintf(message,"Freq: %4d.%03d MHz  %s",RF_Fre_Value/1000,RF_Fre_Value%1000,(digitalRead(ADF4351_PLL_LOCKED_PIN)?"LOCK":"____"));
            _SYNC2(1);
            display.drawString(0,0,message);
    sprintf(message,"Sweep %s",(sweepFlag?"ON":"OFF"));
    display.drawString(0, 10, message);
    sprintf(message,"From %7u to %7u",Start_Fre_value,Stop_Fre_value);
    display.drawString(0, 20, message);
    sprintf(message,"Rate %u kHz per %u ms",Delta_Fre_value,Sweep_Time_value);
    display.drawString(0, 30, message);

    display.drawProgressBar(0, 43, 120, 10, percentSweep);

    display.display();  //takes 150 us
    _SYNC2(0);
}
void SweepProgress(int percentSweep){
   //display.drawProgressBar(0, 40, 120, 10, percentSweep);
   //display.display();
}

void SweepTimerStart(void)
{
  sweepFlag=true;
    // Set alarm to call onTimer function every second (value in microseconds).

  setupTimer();
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, Sweep_Time_value*1000LL, true);

  // Start an alarm
  timerAlarmEnable(timer);
}
void SweepTimerStop(void)
{
  sweepFlag=false;
  timerEnd(timer);
}

void initializeSweepParameters()
{
  strcpy(g_sweepParameters.name,"ADF4351 Synthetizer sweeper");
  g_sweepParameters.refClk=10;
  g_sweepParameters.sweepOn = false;
  g_sweepParameters.current = 144300;
  g_sweepParameters.start = 144300;
  g_sweepParameters.stop = 146000;
  g_sweepParameters.step = 10; //kHz
  g_sweepParameters.timeStep = 1; // millis seconds
}

void initPorts() 
{
  pinMode(ADF4351_PLL_LOCKED_PIN, INPUT);  // PIN 2 en entree pour lock
  pinMode(ADF4351_LE, OUTPUT);          // Setup pins
  digitalWrite(ADF4351_LE, HIGH);
  
  SPI.begin();                          // Init SPI bus
  SPI.setFrequency(8000000);
  SPI.setBitOrder(MSBFIRST);            // poids forts en tête
}
//*********************************************************************
void ADF4351_Init(int referenceFrequency)
{
  initPorts();
  reset_all_reg(referenceFrequency);
// referenceFrequency is in MHz, currentFrequency is in kHz
  SetCurrentFrequency(referenceFrequency *1000 * 10);
}


int SetCurrentFrequency(int newFrequencyKHz)
{
  int value = RF_Fre_Value;
  RF_Fre_Value = newFrequencyKHz;
  return value;
}

static unsigned long Sweep_Time_Counter;
static unsigned char Sweep_DIR_Flag;

void StartSweep(unsigned long Start, 
        unsigned long  Stop, 
        unsigned long  SweepDeltaTime, 
        unsigned long  SweepDeltaFrequency)
{
  if (Start < Stop) {
// Initialize global variables
    Start_Fre_value = Start;
    Stop_Fre_value = Stop;
    Sweep_Time_value = SweepDeltaTime;
    Delta_Fre_value = SweepDeltaFrequency;
    Sweep_Time_Counter = 0;
    percentSweepIncrement = 100.0/((Stop_Fre_value-Start_Fre_value)/Delta_Fre_value);
    percentSweep = 0;

    SweepCurrentFreq = Start_Fre_value;
    Sweep_DIR_Flag = 0;
    RF_Fre_Value = Start_Fre_value;
    RF_OUT();
    
    dac_output_voltage(DAC_CHANNEL_1, 0);

    setFastLockTimer(20,g_sweepParameters.refClk);
    SweepTimerStart();
  }
}


void SweepTimerTick(void){ // interrupt processing routine, every usec
  if (Sweep_Time_Counter++ >= Sweep_Time_value) { // wait until the counter reaches Sweep_Time_Value
    Sweep_Time_Counter = 0;
    if (Sweep_DIR_Flag == 0) {
      SweepCurrentFreq += Delta_Fre_value;

      percentSweep += percentSweepIncrement;
      if (percentSweep >= 100){
        percentSweep = 100;
      }   
      
      if (SweepCurrentFreq >= (Stop_Fre_value - Delta_Fre_value/10.0)) {
        SweepCurrentFreq = Stop_Fre_value;
        percentSweep = 100;
        Sweep_DIR_Flag = 1;
      }
    } else {
      SweepCurrentFreq -= Delta_Fre_value;
      
      percentSweep -= percentSweepIncrement;
      if (percentSweep <= 0) {
        percentSweep = 0;
      }   
      if (SweepCurrentFreq <= (Start_Fre_value + Delta_Fre_value/10.0)) {
        SweepCurrentFreq = Start_Fre_value;
        percentSweep=0;
        Sweep_DIR_Flag = 0;
      }
    }
    SweepProgress(percentSweep);
    dac_output_voltage(DAC_CHANNEL_1, percentSweep*2.55);

    RF_Fre_Value =  SweepCurrentFreq;
    RF_OUT();
  }
}


void RF_OUT(void)
{
  unsigned int mod_v[10]={5,25,50,125,250,500,1000,2000,4000};
  unsigned char div,flag;
  unsigned int frac_value;
  unsigned int mod_value;
  double n_value,op,y;
  double ref_clk=REF_CLK;
  unsigned long temp;
  unsigned int i,x;
  
    if(RF_Fre_Value<69000)  {
        Register_Buf[4] &= 0x000FFFFF;
        Register_Buf[4] |= 0x00E00000;
        div=64;
    }
    else if(RF_Fre_Value<138000)
    {
        Register_Buf[4] &= 0x000FFFFF;
        Register_Buf[4] |= 0x00D00000;
        div=32;
    }
    else if(RF_Fre_Value<275000)
    {
        Register_Buf[4] &= 0x000FFFFF;
        Register_Buf[4] |= 0x00C00000;
        div=16;
    }
    else if(RF_Fre_Value<550000)
    {
        Register_Buf[4] &= 0x000FFFFF;
        Register_Buf[4] |= 0x00B00000;
        div=8;
    }
    else if(RF_Fre_Value<1100000)
    {
        Register_Buf[4] &= 0x000FFFFF;
        Register_Buf[4] |= 0x00A00000;
        div=4;
    }
    else if(RF_Fre_Value<2200000)
    {
        Register_Buf[4] &= 0x000FFFFF;
        Register_Buf[4] |= 0x00900000;
        div=2;
    }
    else
    {
        Register_Buf[4] &= 0x000FFFFF;
        Register_Buf[4] |= 0x00800000;
        div=1;
    }
    
  unsigned long oscillatorFrequency = RF_Fre_Value * div;
  Register_Buf[0] = (oscillatorFrequency / 1000) << 15 | (oscillatorFrequency % 1000) << 3;
    
  _SYNC(1); // measure time to write to ADF
// 400us to send all registers on EPS32 with SPI 1MHz  200us with SPI 8MHz
  for (int i=5;i>=0;i--){
    bool mustWrite = false;
    if (mustWrite || (Register_Buf[i] != Register_Previous[i])) {
      mustWrite = true; // we must finish up to register 0 once we start writing into registers
      WriteRegister32(&Register_Buf[i]);
      Register_Previous[i] = Register_Buf[i];// remember value so that we can skip it next time
    }
  }
  _SYNC(0);
}

void reset_all_reg(int referenceFrequency)
{
//[DB23:DB22] = 01 digital lock detect
#define LDPIN_DIGITAL_LOCK_DETECT 1
//[DB20:DB19] = 11 reserved
#define RESERVED_11 0x3
  Register_Buf[5]= LDPIN_DIGITAL_LOCK_DETECT << 22 | RESERVED_11 << 19 | 0x00000005;

//(DB23=1)The signal is taken from the VCO directly;
//(DB22-20:4H) the RF divider is 8;
//(DB19-12:50H) band select clock divider: 08
//(DB11=0)VCO powerd up;
//(DB5=1)RF output is enabled;
#define RF_ENABLE 1
//(DB4-3=3H)Output power level is 5
#define OUTPUT_LEVEL_5 0x3
  Register_Buf[4]=0x00c08038 | 0x00000004;

// Register 3
//(DB23=1) Band Select Clock Mode for fast lock and > 125KHz PFD freq
//[DB16:DB15] = 01 to activate fast lock;
#define CLOCK_DIVIDER 1000 // KHz resolution
  Register_Buf[3]= (1<<23)  | CLOCK_DIVIDER << 3 | 0x00000003;

// Register 2
#define R_COUNTER referenceFrequency  // 1 MHz PFD
#define DOUBLE_BUFFER 1
//[DB30:DB29] = 11 low spur mode;
//[DB28:DB26] 101= analog lock detect  110= digital lock detect
#define ANALOG_LOCK_DETECT 0x5
#define DIGITAL_LOCK_DETECT 0x6
#define LOW_SPUR 0x3
//(DB6=1)set PD polarity is positive;
#define PDP 1
//(DB7=0)LDP is 10nS  (DB8,DB7)=(0,0) recommended for fractional-N;
//(DB8=0)enable fractional-N digital lock detect;
//(DB12-9:7H)set Icp 2.50 mA;
#define CP25 0x7
  Register_Buf[2] = (LOW_SPUR<<29) | DIGITAL_LOCK_DETECT<< 26 | CP25 << 9 | PDP << 6 | DOUBLE_BUFFER << 13 | R_COUNTER << 14 | 0x00000002;

#define PRESCALER_8_9 1
#define PHASE_VALUE 1
#define MODULUS_VALUE 1000 // so that we get KHz resolution
  Register_Buf[1]= PRESCALER_8_9 << 27 | PHASE_VALUE << 15 | MODULUS_VALUE << 3 | 0x00000001;

#define INTEGER_VALUE 100 // MHz
#define FRACTIONAL_VALUE 0 // KHz
  Register_Buf[0]= INTEGER_VALUE << 15 | FRACTIONAL_VALUE << 3;
  
  for (int i=0;i<6;i++){
    Register_Previous[i] = 0;
  }
}

void setFastLockTimer(unsigned int microSeconds, unsigned int refClk)
{ // see page 22 of datasheet
  //[DB16:DB15] = 01 to setup fast lock timer;
  unsigned int timerValue=(20+microSeconds)*(refClk * 1000000)/MODULUS_VALUE;
    Register_Buf[3]= (1<<15) | 0x00000003 | timerValue << 3;
}

void WriteRegister32(const unsigned long *value)   //Programme un registre 32bits
{
  // this takes 35us with SPI 8MHz and 67us with SPI 1MHz
  digitalWrite(ADF4351_LE, LOW);
  _SYNC3(1);
  #if 0
  SPI.transfer32(*value);
  #else
  for (int i = 3; i >=0; i--) {         // boucle sur 4 x 8bits MSB first
    SPI.transfer((*value >> (8 * i)) & 0xFF); // MSB first as per SPI initialization
  }
  #endif
  digitalWrite(ADF4351_LE, HIGH); // load data
  delayMicroseconds(1);
  digitalWrite(ADF4351_LE, LOW);
  _SYNC3(0);
}

void print_reset_reason(RESET_REASON reason)
{
  switch ( reason)
  {
    case 1 : Serial.println ("POWERON_RESET");break;          /**<1, Vbat power on reset*/
    case 3 : Serial.println ("SW_RESET");break;               /**<3, Software reset digital core*/
    case 4 : Serial.println ("OWDT_RESET");break;             /**<4, Legacy watch dog reset digital core*/
    case 5 : Serial.println ("DEEPSLEEP_RESET");break;        /**<5, Deep Sleep reset digital core*/
    case 6 : Serial.println ("SDIO_RESET");break;             /**<6, Reset by SLC module, reset digital core*/
    case 7 : Serial.println ("TG0WDT_SYS_RESET");break;       /**<7, Timer Group0 Watch dog reset digital core*/
    case 8 : Serial.println ("TG1WDT_SYS_RESET");break;       /**<8, Timer Group1 Watch dog reset digital core*/
    case 9 : Serial.println ("RTCWDT_SYS_RESET");break;       /**<9, RTC Watch dog Reset digital core*/
    case 10 : Serial.println ("INTRUSION_RESET");break;       /**<10, Instrusion tested to reset CPU*/
    case 11 : Serial.println ("TGWDT_CPU_RESET");break;       /**<11, Time Group reset CPU*/
    case 12 : Serial.println ("SW_CPU_RESET");break;          /**<12, Software reset CPU*/
    case 13 : Serial.println ("RTCWDT_CPU_RESET");break;      /**<13, RTC Watch dog Reset CPU*/
    case 14 : Serial.println ("EXT_CPU_RESET");break;         /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : Serial.println ("RTCWDT_BROWN_OUT_RESET");break;/**<15, Reset when the vdd voltage is not stable*/
    case 16 : Serial.println ("RTCWDT_RTC_RESET");break;      /**<16, RTC Watch dog reset digital core and rtc module*/
    default : Serial.println ("NO_MEAN");
  }
}





