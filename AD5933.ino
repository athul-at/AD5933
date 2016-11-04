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

// Setup routine runs once when you press reset
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
 Serial.println("Sl No., Time, Frequency, Impedance(Ohms), Phase(degrees),R,I");
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
}

/*! Plot the impedance at a given frequncy varying settling time*/
void Plot_impedance_settlingtime(unsigned long freq, unsigned long settling_start, unsigned settling_step_size)
{
  unsigned long settlingTime = 1;
  unsigned long current_settling_time;
  unsigned remainder = 0;
  Serial.println("Sl No., Time, Frequency, Settling_time, Impedance (Ohms), Phase(degrees), R, I");
  for(int i = 0; i<=increment_number; i++)
  {
    current_settling_time = settling_start+(i*settling_step_size);
    if (current_settling_time <=511)
    {
         AD5933_settling_time(current_settling_time,AD5933_SETTLE_1X);
         remainder = 0;
         #ifdef DEBUG4
         Serial.print("Settling Time: "); 
         Serial.print(current_settling_time);
         Serial.println("X1");
         #endif
         
    }
    else if ((current_settling_time >511)&&(current_settling_time <1022))
    {
        AD5933_settling_time(current_settling_time/2,AD5933_SETTLE_2X);
        remainder = current_settling_time%2;
        #ifdef DEBUG4
         Serial.print("Settling Time: "); 
         Serial.print(current_settling_time/2);
         Serial.println("X2");
         #endif
    }
    else
    {
      AD5933_settling_time(current_settling_time/4,AD5933_SETTLE_4X);
      remainder = current_settling_time%4;
      #ifdef DEBUG4
        Serial.print("Settling Time: "); 
        Serial.print(current_settling_time/4);
        Serial.println("X4");
      #endif
    }
    AD5933_ConfigSweep(freq,0,increment_number);
    AD5933_StartSweep();
    impedance = AD5933_CalculateImpedance(gainFactor, AD5933_FUNCTION_REPEAT_FREQ);
    Serial.print(i);
    Serial.print(",");
    Serial.print(millis());
    Serial.print(",");
    Serial.print(start_freq);
    Serial.print(",");
    Serial.print(current_settling_time-remainder);
    Serial.print(",");
    Serial.print(impedance);
    Serial.print(",");
    Serial.print(impedance_phase);
    Serial.print(",");
    Serial.print(GrealData);
    Serial.print(",");
    Serial.println(GimagData);
  }
}



