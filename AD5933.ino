
// AD5933 I2C Interface Master device.
// Author         : Athul Asokan Thulasi
// Created on     : September 1, 2016
// Last modified  : September 21, 2016

#include <Wire.h>
#include "AD5933.h" 
#include <math.h>           


# define RANGE AD5933_RANGE_200mVpp
# define GAIN AD5933_GAIN_X1
 unsigned int increment_number = 20;
 unsigned int start_freq = 90;
 unsigned int freq_step = 1;
extern double impedance_phase;
 
/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
unsigned short  temperature = 0;
double          impedanceK  = 0.0;
double          impedance   = 0.0;
double          gainFactor  = 1.0;
double          baseline_impedance = 0.0;
double          sweat_impedance = 0.0;
double          delta_impedance = 0.0;
double          glucose_concentration = 0.0;
/******************************************************************************/

// Setup routine runs once when you press reset:
void setup() 
{                
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
  AD5933_SetSystemClk(AD5933_CONTROL_EXT_SYSCLK, 100000ul);
  Serial.println("Clock Setup completed");
  /* Set range and gain. */
  AD5933_SetRangeAndGain(AD5933_RANGE_200mVpp, AD5933_GAIN_X1);
  Serial.println("Setting range and gain done. .");
  /* Read the temperature. */
  temperature = AD5933_GetTemperature();
  Serial.println("");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");
  Serial.println("");
  /* Configuring the settling cycles to 4 cycles*/
  AD5933_settling_time(20,AD5933_SETTLE_4X);
  Serial.println("Setting settling cycles to 80 done. .");
   /*Configure the sweep parameters */
  AD5933_ConfigSweep(start_freq,       // 90 Hz
                       freq_step,        // 1 Hz increments
                       increment_number);        // 20 increments
  Serial.println("Setting the sweep settings completed. . ");
  Serial.println("");
  /* Starting frequency sweep*/
  AD5933_StartSweep();
}

// Loop routine runs over and over again forever
void loop()
{
   //Serial.println("Inside the loop function");
   char usr_sel;
   usr_sel = user_input();
   execute_user_function(usr_sel);
}


/*! Plot the impedance spectrum in the frequency range configured by the config Sweep function */
void Plot_impedance_spectrum()
{
 Serial.println("Sl No., Frequency, Impedance (Ohms), Phase (degrees)");
  for(int i = 0; i<increment_number; i++)
  {
    impedance = AD5933_CalculateImpedance(gainFactor, AD5933_FUNCTION_INC_FREQ);
    baseline_impedance = impedance;
    Serial.print(start_freq + freq_step*i);
    Serial.print(",");
    Serial.print(baseline_impedance);
    Serial.print(",");
    Serial.println(impedance_phase);
  }
}


