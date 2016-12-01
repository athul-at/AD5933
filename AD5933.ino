// AD5933 I2C Interface Master device.
// Author         : Athul Asokan Thulasi
// Created on     : September 1, 2016
// Last modified  : November, 2016

#include <Wire.h>
#include "AD5933.h" 
#include "AD524X.h"
#include <math.h>   
#include "Linduino.h"
#include "LT_I2C.h"
#include "LTC6904.h"
#include <Wire.h>

// Function Declaration 
void LTC_setclock();
boolean configureAD5245(float R);
void configure2M_DPOT(boolean pot_number, unsigned long int resistance); // 0 - U11 calib DPOT, 1- U3 gain DPOT

// Global variables
static uint8_t output_config = LTC6904_CLK_ON_CLK_INV_OFF;  //!< Keeps track of output configuration of LTC Clock .        
/******************************************************************************/
/*****************  Defining the PIN locations         ************************/
/******************************************************************************/
 #define AMUX_ADRS0 2 
 #define AMUX_ADRS1 3 
 #define AMUX_EN 4  

 #define AMUX2_ADRS0 30 
 #define AMUX2_ADRS1 32 
 #define AMUX2_EN 31  
 /******************************************************************************/
/*****************  EIS system configuration settings  ************************/
/******************************************************************************/
# define AD5245_ADDR  0x2D
# define RANGE  AD5933_RANGE_200mVpp
# define GAIN   AD5933_GAIN_X1
# define SETLE_MULTIPLIER AD5933_SETTLE_4X
# define DPOT_CALIB 1   // Comment this line to not used DPOT for calib
 
unsigned long   start_freq                 = 100;
unsigned long   freq_step                  = 0;
unsigned short  increment_number           = 3;
unsigned long   settling_cycles            = 1;   // Actual settling cycle count = settling_cycles * SETTLE_MULTIPLIER
unsigned long   external_clock_freq        = 100000;
unsigned long calib_impedance              = 18060; //18K Ohm 
unsigned int wait_minutes                  = 0;    // Actual value = 13;
float AD5245_resistance                    = 10000;
unsigned long int calib_DPOT_resistance                  = calib_impedance;
unsigned long int gain_DPOT_resistance               = 168000;

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
unsigned short      temperature              = 0;
double              impedance                = 0.0;
double              gainFactor               = 1.0;
double              gain_delta_change        = 0.0;
double              baseline_impedance       = 0.0;
double              sweat_impedance          = 0.0;
double              delta_impedance_percent  = 0.0;
double              glucose_concentration    = 0.0;
double              impedance_phase          = 0.0;
double              impedance_sum            = 0.0;
double              phase_sum                = 0.0; 
extern signed short GrealData;
extern signed short GimagData;
unsigned long start_time;
AD524X AD00(0x2F);  // 0 - U11 calib DPOT
AD524X AD01(0x2C);  // 1 - U3 gain DPOT
/******************************************************************************/
/*! Function that reads a number from serial port*/
unsigned long read_number()
{
      unsigned long num = 0;  
      while(Serial.available() == 0);
      while(Serial.available() > 0)
       {
        num = num*10 + Serial.parseInt();
       }
      return num;
}

/******************************************************************************/
/*! Function that selects the channel of the mux*/
void set_mux( int channel)
{
  if(channel == 0) // POT_CHANNEL
  {
  digitalWrite(AMUX_EN,HIGH);
  digitalWrite(AMUX_ADRS0,LOW);
  digitalWrite(AMUX_ADRS1,LOW); 
  }
  else if(channel == 1) // CALIB_CHANNEL1
  {
  digitalWrite(AMUX_EN,HIGH);
  digitalWrite(AMUX_ADRS0,HIGH);
  digitalWrite(AMUX_ADRS1,LOW);
  }
  else if (channel == 2) // CLIB_CHANNEL2
  {
  digitalWrite(AMUX_EN,HIGH);
  digitalWrite(AMUX_ADRS0,LOW);
  digitalWrite(AMUX_ADRS1,HIGH);
  }
  else if ( channel == 3) //IMPEDANCE_CHANNEL
  {
  digitalWrite(AMUX_EN,HIGH);
  digitalWrite(AMUX_ADRS0,HIGH);
  digitalWrite(AMUX_ADRS1,HIGH);
  }
  else
  {
    Serial.println("Invalid mux channel: 0-3 are valid options"); 
  }
}

// Setup routine runs once when you press reset
void setup() 
{  
  Serial.begin(9600); 
  Serial.println("##############################################################################");
  Serial.println("#                Sweat Glucose Measurement Program                           #");
  Serial.println("##############################################################################");
  Serial.println("");
  Serial.println("Starting device initialization sequence . .");
  pinMode(AMUX_EN,OUTPUT);
  pinMode(AMUX_ADRS0,OUTPUT);
  pinMode(AMUX_ADRS1,OUTPUT);
  pinMode(AMUX2_EN,OUTPUT);
  pinMode(AMUX2_ADRS0,OUTPUT);
  pinMode(AMUX2_ADRS1,OUTPUT);
  /*Select the AD5245 (100K D.POT resistance) for first stage gain */
  boolean status_val = configureAD5245(AD5245_resistance);
  TWBR = 12;  // 400 KHz  IIC speed for AD5245 1M Ohm dual D.POT
  configure2M_DPOT(0,calib_DPOT_resistance);
  configure2M_DPOT(1,gain_DPOT_resistance);
  #ifndef DPOT_CALIB
  set_mux(1);  //Selecting fixed 18K resitance for calibration
  #else
   set_mux(0); // Selecting DPOT for calibartion
  #endif     
  Wire.begin(); 
  /* Set the clock frequecny in LTC6904 clock source */
  LTC_setclock();
  /* Reset the device. */
  AD5933_Reset();
  Serial.println("Reset completed. .");
  /* Select the source of the AD5933 system clock. */
  AD5933_SetSystemClk(AD5933_CONTROL_EXT_SYSCLK, external_clock_freq);
  Serial.println("Clock Setup completed");
  /* Set range and gain. */
  AD5933_SetRangeAndGain(RANGE,GAIN);
  Serial.println("Setting range and gain done. .");
  /* Read the temperature. */
  temperature = AD5933_GetTemperature();
  Serial.print("Temperature: ");Serial.print(temperature);Serial.println(" C");
  /* Configuring the settling cycles */
  AD5933_settling_time(settling_cycles,SETLE_MULTIPLIER);
  Serial.println("Setting settling cycles to done. .");
  /*Configure the sweep parameters */
  AD5933_ConfigSweep(start_freq, freq_step, increment_number);
  Serial.println("Setting the sweep settings completed. . ");
   /* Starting frequency sweep*/
  AD5933_StartSweep();
  Serial.print("Calibration resistance value: ");
  Serial.println(calib_impedance);
  gainFactor = AD5933_CalculateGainFactor(calib_impedance,AD5933_FUNCTION_REPEAT_FREQ);
  Serial.print("Calculated Calibration Gain: ");
  Serial.println(gainFactor);
  
  Serial.println("Device initialization sequence completed. .");
  Serial.println("");
  Serial.println("Add baseline sample and wait for it to be ready for measurement");
  Serial.println("Enter any key to continue when ready.");
  AD5933_standby();
  while(Serial.available() == 0);
  while(Serial.available() >0)
  {
      Serial.read();
  }
  AD5933_Reset();
  AD5933_StartSweep();
  set_mux(3);  // Impedance channel
 Serial.println("Measuring baseline impedance. . .");
 Serial.println("Sl No., Time, Frequency, Baseline Impedance(Ohms), Phase(degrees),R,I");
 
 impedance_sum = 0.0;
 phase_sum = 0.0;
 start_time = millis();
  for(int i = 1; i<=increment_number; i++)
  {
    impedance = AD5933_CalculateImpedance(gainFactor, AD5933_FUNCTION_INC_FREQ);
    baseline_impedance = impedance;
    Serial.print(i);
    Serial.print(",");
    Serial.print((millis()-start_time)/1000.0);
    Serial.print(",");
    Serial.print(start_freq + freq_step*i);
    Serial.print(",");
    Serial.print(baseline_impedance);
    Serial.print(",");
    Serial.print(impedance_phase);
    Serial.print(",");
    Serial.print(GrealData);
    Serial.print(",");
    Serial.println(GimagData);
    impedance_sum = impedance_sum + baseline_impedance;
    phase_sum = phase_sum + impedance_phase;
  }
  set_mux(2); // Avoiding the DC voltage to be applied to the glucose sensor
  baseline_impedance = impedance_sum/increment_number;
  Serial.print("Average Baseline impedance : ");
  Serial.println(baseline_impedance);
  Serial.print("Average Baseline phase : ");
  Serial.println(phase_sum/increment_number);
  
  //AD5933_power_down();                
}

// Loop routine runs over and over again forever
void loop()
{
  Serial.println("");
  Serial.println("");
  AD5933_standby();
  Serial.println("Measuring sweat impedance. . .");
  Serial.println("Add Sweat sample");
  Serial.print("Enter any key to start ");
  Serial.print(wait_minutes);
  Serial.println(" mts timer");
  while(Serial.available() == 0);
  while(Serial.available() >0)
  {
      Serial.read();
  }
  start_time = millis();
  for (unsigned long sec = 0; sec < wait_minutes*60;)
  {
    sec = (millis() - start_time)/1000;
    if(sec % 60 == 0)
    {
      Serial.print("Time remaining to start measurement: ");
      Serial.print(wait_minutes - (sec/60));
      Serial.println(" mts");
    }
    delay (1000);
  }
   /* Reset the device. */
  // Impedance channel
  AD5933_Reset();
  Serial.println("Reset completed. .");
  Serial.println("");
 Serial.println("Sl No., Time, Frequency, Impedance(Ohms), Phase(degrees),R,I");
  /* Starting frequency sweep*/
  AD5933_StartSweep();
    set_mux(3);
 impedance_sum = 0.0;
 phase_sum = 0.0;
  start_time = millis();
  for(int i = 1; i<=increment_number; i++)
  {
    impedance = AD5933_CalculateImpedance(gainFactor, AD5933_FUNCTION_INC_FREQ);
    Serial.print(i);
    Serial.print(",");
    Serial.print((millis()-start_time)/1000.0);
    Serial.print(",");
    Serial.print(start_freq + freq_step*i);
    Serial.print(",");
    Serial.print(impedance);
    Serial.print(",");
    Serial.print(impedance_phase);
    Serial.print(",");
    Serial.print(GrealData);
    Serial.print(",");
    Serial.println(GimagData);
    impedance_sum = impedance_sum + impedance;
    phase_sum = phase_sum + impedance_phase;
  }
  set_mux(2);
  sweat_impedance = impedance_sum/increment_number;
  Serial.print("Average Sweat impedance : ");
  Serial.println(sweat_impedance);
  Serial.print("Average Sweat phase : ");
  Serial.println(phase_sum/increment_number);
  delta_impedance_percent = ((baseline_impedance - sweat_impedance)*100.0)/baseline_impedance;
  Serial.print("Impedance change percent: ");
  Serial.println(delta_impedance_percent);
  glucose_concentration = calculate_concentration(delta_impedance_percent);
  Serial.print("Calculated Sweat Glucose Concentration : ");
  Serial.print(glucose_concentration);
  Serial.println(" ug/ml");
}

void LTC_setclock()
{
  float freq = (float)external_clock_freq;
  uint16_t clock_code;
  uint8_t ack;
  quikeval_I2C_connect();
  clock_code = LTC6904_frequency_to_code((freq/1000000)+0.00072, output_config); // 0.72 is error in generation
  //clock_code = LTC6904_frequency_to_code((freq/1000000), output_config);
  Serial.print("Set LTC clock frequency: ");
  Serial.print((freq/1000));
  Serial.println(" KHz");
  ack = LTC6904_write(LTC6904_ADDRESS, (uint16_t)clock_code);
}


boolean configureAD5245(float R) 
{  
  
    Serial.print("Setting resistance of AD5245 to ");
    Serial.print(R);
    Serial.println(" Ohms.");
  
   int i2cStatus;
  
   int Rw = 50; // wiper resistance
   float Rab = 100*pow(10,3); // max resistance
   
   if (R > Rab) {
     return false;
   } else {
  
     int D = int(( 256 * (R + 2*Rw) ) / Rab);
     
     Wire.beginTransmission(AD5245_ADDR); // master sends a 7-bit slave address
     Wire.write(0x00); // instruction byte
     Wire.write(D);    // data byte
     i2cStatus = Wire.endTransmission();
     
     if (i2cStatus)
       return false;
     else
       return true;
   }  
}


void configure2M_DPOT(boolean pot_number, unsigned long int resistance)
{
  //Serial.print("Resistance :");Serial.println(resistance);
  int wiper_pos = (resistance/4000);
  //Serial.print("Wiper pos sum : "); Serial.println(wiper_pos);
  int rem = wiper_pos%2;
  //Serial.print("Wiper pos rem : "); Serial.println(rem);
  wiper_pos = wiper_pos/2;
  //Serial.print("Wiper pos : "); Serial.println(wiper_pos);
  //Serial.print("Reverse calculated resistance :"); Serial.println(( (unsigned long)4000*((wiper_pos*2)+rem)));
  if(pot_number)
  {
    AD01.write(1, 256-(wiper_pos+rem));AD01.write(0, 256-(wiper_pos));
  }
  else
  {
    AD00.write(1, 256-(wiper_pos+rem));AD00.write(0, 256-(wiper_pos));
    #ifdef DPOT_CALIB
    calib_impedance = (unsigned long)4000*((wiper_pos*2)+rem);
    Serial.print("Adjusted DPOT calibration impedance to : ");Serial.println(calib_impedance);
    #endif
  }
}

