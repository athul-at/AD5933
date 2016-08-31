
#define LED1 RED_LED
#define LED2 GREEN_LED


int user_input()
{
 start_menu:
 Serial.println("##############################################################################");
 Serial.println("# Run the menu options sequentially                                          #");
 Serial.println("# 1. Run Calibration                                                         #");
 Serial.println("# 2. Measure Impedance without sweat sample                                  #");
 Serial.println("# 3. Measure Impedance with sweat sample                                     #");
 Serial.println("# 4. Calculate Sweat Concentartion                                           #");
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
       break;
   case '2':
       Serial.println("Measuring baseline impedance. . .");
       Serial.println("");
       Serial.println("");
       break;
   case '3':
      Serial.println("Measuring Impedance with sweat sample. . .");
      Serial.println("");
      Serial.println("");
      break;
   case '4': 
      Serial.println("Calculating the Sweat Concentration. . .");
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
 
// the setup routine runs once when you press reset:
void setup() 
{                
  // initialize the digital pin as an output.
  pinMode(LED1, OUTPUT); 
  pinMode(LED2, OUTPUT); 

  // Start serial communication at 9600 baud
  Serial.begin(9600);  
  //while(Serial.available() == 0);
  //Serial.flush(); 
}

// the loop routine runs over and over again forever:
void loop()
{
   int usr_sel;
   usr_sel = user_input();
   execute_user_function(usr_sel);
}
