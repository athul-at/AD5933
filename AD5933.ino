// AD5933 I2C Interface Master device.
// Author         : Athul Asokan Thulasi
// Created on     : September 1, 2016
// Last modified  : November, 2016

#include <Wire.h>
#include "AD5933.h" 
#include <math.h>   
#include "Linduino.h"
#include "LT_I2C.h"
#include "LTC6904.h"
#include <Wire.h>

// Function Declaration 
void LTC_setclock();


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
 
# define RANGE  AD5933_RANGE_200mVpp
# define GAIN   AD5933_GAIN_X1
# define SETLE_MULTIPLIER AD5933_SETTLE_4X

unsigned long   start_freq                 = 100;
unsigned long   freq_step                  = 0;
unsigned short  increment_number           = 3;
unsigned long   settling_cycles            = 100;   // Actual settling cycle count = settling_cycles * SETTLE_MULTIPLIER
unsigned long   external_clock_freq        = 100000;
unsigned long calib_impedance = 18000; //18K Ohm 
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
  pinMode(AMUX_EN,OUTPUT);
  pinMode(AMUX_ADRS0,OUTPUT);
  pinMode(AMUX_ADRS1,OUTPUT);
  pinMode(AMUX2_EN,OUTPUT);
  pinMode(AMUX2_ADRS0,OUTPUT);
  pinMode(AMUX2_ADRS1,OUTPUT);
  set_mux(1);
//  unsigned char in_address = 0;;
//  unsigned long  in_data = 0;
//  unsigned long read_value = 0;
//  byte index;
//  char adrs_buf[2];
//  char data_buf[2];          
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
  //calib_impedance = read_number();
  Serial.println(calib_impedance);
  gainFactor = AD5933_CalculateGainFactor(calib_impedance,AD5933_FUNCTION_REPEAT_FREQ);
  Serial.print("Calculated Calibration Gain: ");
  Serial.println(gainFactor);
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
    if(i==1)
    {
        /* Configuring the settling cycles */
      AD5933_settling_time(1,SETLE_MULTIPLIER); //Reducing it for a fast measurement
    }
  }
  baseline_impedance = impedance_sum/increment_number;
  Serial.print("Average Baseline impedance : ");
  Serial.println(baseline_impedance);
  Serial.print("Average Baseline phase : ");
  Serial.println(phase_sum/increment_number);
  Serial.println("");
  Serial.println("");
  
  //AD5933_power_down();                
}

// Loop routine runs over and over again forever
void loop()
{
  AD5933_standby();
  Serial.println("Measuring sweat impedance. . .");
  Serial.println("Add Sweat sample and wait for it to be ready for measurement");
  Serial.println(" Enter any key to continue when ready.");
  while(Serial.available() == 0);
  while(Serial.available() >0)
  {
      Serial.read();
  }
  Serial.println("");
  Serial.println("");
   /* Reset the device. */
  set_mux(3);  // Impedance channel
  AD5933_Reset();
  Serial.println("Reset completed. .");
  /* Set range and gain. */
  AD5933_SetRangeAndGain(RANGE,GAIN);
  Serial.println("Setting range and gain done. .");
     /*Configure the sweep parameters */
  AD5933_ConfigSweep(start_freq,       // 10 KHz
                       freq_step,        // 1 KHz increments
                       increment_number);        // 500 increments
  Serial.println("Setting the sweep settings completed. . ");
  Serial.println("");
 Serial.println("Sl No., Time, Frequency, Impedance(Ohms), Phase(degrees),R,I");
  /* Starting frequency sweep*/
  AD5933_StartSweep();
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
    if(i==1)
    {
        /* Configuring the settling cycles */
      AD5933_settling_time(1,SETLE_MULTIPLIER); //Reducing it for a fast measurement
    }
  }
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



