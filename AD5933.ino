
// AD5933 I2C Interface Master device.
// Author         : Athul Asokan Thulasi
// Created on     : September 1, 2016
// Last modified  : September 4, 2016

#include <Wire.h>
#include "AD5933.h" 
#include <math.h>           

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
unsigned short  temperature = 0;
unsigned long   impedanceK  = 0;
double          impedance   = 0;
double          gainFactor  = 0.0;
double          baseline_impedance =0.0;
double          sweat_impedance = 0.0;
double          delta_impedance = 0.0;
double          glucose_concentration = 0.0;
/******************************************************************************/

// Setup routine runs once when you press reset:
void setup() 
{                
  Serial.begin(9600); 
  Serial.println("Impedance measurement program");
  Serial.println("");
  Serial.println("");
  Wire.begin(); 
  /* Reset the device. */
  AD5933_Reset();
  /* Select the source of the AD5933 system clock. */
  AD5933_SetSystemClk(AD5933_CONTROL_INT_SYSCLK, 0);
  /* Set range and gain. */
  AD5933_SetRangeAndGain(AD5933_RANGE_2000mVpp, AD5933_GAIN_X1);
  /* Read the temperature. */
  temperature = AD5933_GetTemperature();
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");
  Serial.println("");
  Serial.println("");
  /* Configure the sweep parameters */
  AD5933_ConfigSweep(100,       // 100 Hz
                       1000,        // 1000 Hz
                       500);        // 500 increments
}

// Loop routine runs over and over again forever
void loop()
{
   int usr_sel;
   usr_sel = user_input();
   execute_user_function(usr_sel);
}


int user_input()
{
 start_menu:
 Serial.println("##############################################################################");
 Serial.println("# Run the menu options sequentially                                          #");
 Serial.println("# 1. Run Calibration                                                         #");
 Serial.println("# 2. Measure Impedance without sweat sample                                  #");
 Serial.println("# 3. Measure Impedance with sweat sample                                     #");
 Serial.println("# 4. Calculate Sweat Concentration                                           #");
 Serial.println("##############################################################################");
 Serial.println("");
 Serial.println("");
 while(Serial.available() == 0);
 int in_byte = Serial.read();
 Serial.print("User selected option:   ");
 Serial.println(in_byte-'0'); 
  if ((in_byte >= '0')&&(in_byte <= '4'))
  {
   Serial.println("Accepted user input");
   Serial.println("");
   Serial.println("");
   return in_byte;
  }
  else
  {
   Serial.println("Invalid Input !!");
   Serial.println("");
   Serial.println("");
   goto start_menu;
  }
}

void execute_user_function(int inByte)
{
 switch(inByte)
 {
   case '1':
       Serial.println("Calibarting the AD5933. . .");
       Serial.println("");
       Serial.println("");
        /* Start the sweep operation. */
        AD5933_StartSweep();
        /* Calculate the gain factor for an impedance of 47kohms. */
        gainFactor = AD5933_CalculateGainFactor(47000,AD5933_FUNCTION_REPEAT_FREQ);
       Serial.print("Calculated Calibration Gain: ");
       Serial.println(gainFactor);
       Serial.println("");
       Serial.println("");
       break;
   case '2':
       Serial.println("Measuring baseline impedance. . .");
       Serial.println("");
       Serial.println("");
       impedance = AD5933_CalculateImpedance(gainFactor, AD5933_FUNCTION_REPEAT_FREQ);
       impedanceK = (unsigned long)impedance;
       impedanceK /= 1000;
       baseline_impedance = impedanceK;
       Serial.print("Baseline Impedance (K Ohms): ");
       Serial.println(baseline_impedance);
       Serial.println("");
       Serial.println("");
       break;
   case '3':
      Serial.println("Measuring Impedance with sweat sample. . .");
      Serial.println("");
      Serial.println("");
      impedance = AD5933_CalculateImpedance(gainFactor, AD5933_FUNCTION_REPEAT_FREQ);
      impedanceK = (unsigned long)impedance;
      impedanceK /= 1000;
      sweat_impedance = impedanceK;
      Serial.print("Baseline Impedance (K Ohms): ");
      Serial.println(sweat_impedance);
      Serial.println("");
      Serial.println("");
      break;
   case '4': 
      Serial.println("Calculating the Sweat Concentration. . .");
      Serial.println("");
      Serial.println("");
      delta_impedance = abs(baseline_impedance - sweat_impedance);
      glucose_concentration = calculate_concentration(delta_impedance);
      Serial.print("Glucose Concentration (microGram/milliLiter: ");
      Serial.println(glucose_concentration);
      Serial.println("");
      Serial.println("");
      break;
   default: 
      Serial.println("Invalid Option !!");
      Serial.println("");
      Serial.println("");
      break;
 }
}

// Function that calculates the sweat glucose concentration from the change in impedance.
// Rewrite this function with a look up table of a poylnomial fit.
double calculate_concentration(double delta_impedance)
{
  return delta_impedance*250.0;
}

