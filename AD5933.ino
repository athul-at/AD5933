// AD5933 I2C Interface Master device.
// Author         : Athul Asokan Thulasi
// Created on     : September 1, 2016
// Last modified  : November, 2016

#include <Wire.h>
#include "AD5933.h" 
#include <math.h>           

/******************************************************************************/
/*****************  EIS system configuration settings  ************************/
/******************************************************************************/
 
# define RANGE  AD5933_RANGE_200mVpp
# define GAIN   AD5933_GAIN_X1
# define SETLE_MULTIPLIER AD5933_SETTLE_4X

unsigned long   start_freq                 = 100;
unsigned long   freq_step                  = 0;
unsigned short  increment_number           = 20;
unsigned long   settling_cycles            = 100;
unsigned long   external_clock_freq        = 100000;

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
unsigned short      temperature              = 0;
double              impedance                = 0.0;
double              gainFactor               = 1.0;
double              gain_delta_change        = 0.0;
double              baseline_impedance       = 0.0;
double              sweat_impedance          = 0.0;
double              delta_impedance          = 0.0;
double              glucose_concentration    = 0.0;
double              impedance_phase          = 0.0;
extern signed short GrealData;
extern signed short GimagData;
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

// Setup routine runs once when you press reset
void setup() 
{  
  unsigned char in_address = 0;;
  unsigned long  in_data = 0;
  unsigned long read_value = 0;
  byte index;
  char adrs_buf[2];
  char data_buf[2];
  unsigned long calib_impedance = 75400; //75.4K Ohm  
  double system_phase = 0.0;
  double impedance_phase = 0.0;          
  Serial.begin(9600); 
  Serial.println("##############################################################################");
  Serial.println("#                Sweat Glucose Measurement Program                           #");
  Serial.println("##############################################################################");
  Serial.println("");
  Wire.begin(); 
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
  Serial.println("");
  Serial.print("Temperature: ");Serial.print(temperature);Serial.println(" C");
  Serial.println("");
  /* Configuring the settling cycles */
  AD5933_settling_time(settling_cycles,SETLE_MULTIPLIER);
  Serial.println("Setting settling cycles to done. .");
   /*Configure the sweep parameters */
  AD5933_ConfigSweep(start_freq, freq_step, increment_number);
  Serial.println("Setting the sweep settings completed. . ");
  Serial.println("");
  /* Starting frequency sweep*/
  AD5933_StartSweep();
  Serial.print("Enter the Calibration resistance value: ");
  calib_impedance = read_number();
  Serial.println(calib_impedance);
  gainFactor = AD5933_CalculateGainFactor(calib_impedance,AD5933_FUNCTION_REPEAT_FREQ);
  Serial.print("Calculated Calibration Gain: ");
  Serial.println(gainFactor);
  Serial.println("Measuring baseline impedance. . .");
  Serial.println(" Enter any key to continue..");
  while(Serial.available() == 0);
  while(Serial.available() >0)
  {
      Serial.read();
  }
  Serial.println("");
  Serial.println("");
 Serial.println("Sl No., Time, Frequency, Baseline Impedance(Ohms), Phase(degrees),R,I");
  for(int i = 0; i<=increment_number; i++)
  {
    impedance = AD5933_CalculateImpedance(gainFactor, AD5933_FUNCTION_INC_FREQ);
    baseline_impedance = impedance;
    Serial.print(i);
    Serial.print(",");
    Serial.print(millis());
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
  }
  Serial.println("");
  Serial.println("");
  /* Reset the device. */
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
  /* Starting frequency sweep*/
  AD5933_StartSweep();
  Serial.println("Measuring sweat impedance. . .");
  Serial.println(" Enter any key to continue.."); 
  while(Serial.available() == 0);
  while(Serial.available() >0)
  {
      Serial.read();
  }  
  Serial.println("");
  Serial.println("");
 Serial.println("Sl No., Time, Frequency, Impedance(Ohms), Phase(degrees),R,I");
  for(int i = 0; i<increment_number; i++)
  {
    impedance = AD5933_CalculateImpedance(gainFactor, AD5933_FUNCTION_INC_FREQ);
    Serial.print(i);
    Serial.print(",");
    Serial.print(millis());
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
  }
  AD5933_power_down();                
}

// Loop routine runs over and over again forever
void loop()
{
}



