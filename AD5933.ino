
// AD5933 I2C Interface Master device.
// Author         : Athul Asokan Thulasi
// Created on     : September 1, 2016
// Last modified  : September 13, 2016

#include <Wire.h>
#include "AD5933.h" 
#include <math.h>           



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
 #ifndef DEBUG
 /* Reset the device. */
  AD5933_Reset();
  Serial.println("Reset completed. .");
  /* Select the source of the AD5933 system clock. */
  AD5933_SetSystemClk(AD5933_CONTROL_INT_SYSCLK, 16000000ul);
  Serial.println("Clock Setup completed");
  /* Set range and gain. */
  AD5933_SetRangeAndGain(AD5933_RANGE_2000mVpp, AD5933_GAIN_X1);
  Serial.println("Setting range and gain done. .");
  /* Read the temperature. */
  temperature = AD5933_GetTemperature();
  Serial.println("");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");
  Serial.println("");
   /*Configure the sweep parameters */
  AD5933_ConfigSweep(10000,       // 10 KHz
                       1000,        // 1 KHz increments
                       500);        // 500 increments
  Serial.println("Setting the sweep settings completed. . ");
  Serial.println("");
  /* Starting frequency sweep*/
  AD5933_StartSweep();
 #endif
}

// Loop routine runs over and over again forever
void loop()
{
   //Serial.println("Inside the loop function");
   char usr_sel;
   usr_sel = user_input();
   execute_user_function(usr_sel);
}



