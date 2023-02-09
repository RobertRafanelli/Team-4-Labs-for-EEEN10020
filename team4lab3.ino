// Group 4 lab 3 Report - Robert Rafanelli, Hugh Fitzpatrick and Amelie Cojocaru
// We focused our efforts in this lab on improving the structural integrity of our robot and writing a working script for the assigned challenge. 
// We used building techniques shown to us at the start of the lab to improve the integrity of our robot and prevent it from breaking as the motors spun.
// In the meantime, we also created a script to count the number of rotations of an output gear that we connected to the drivetrain through a small spur gear.
// A piece of black tape was placed on this gear, and an optical sensor was used to detect when this black tape had completed one full rotation.
// Rising edge detection had to be used to prevent the function from looping and not counting the correct amount of rotations.
// We also cleaned up our code from the previous weeks by creating functions for code that was being used repeatedly.
// Using our new robot design, we then affixed a distance sensor and got to work with creating a script to make use of this new hardware.
// Unfortunately we did not get this fully working in time however the code has since been amended and we believe that this script should work correctly once uploaded.
#include <LCD16x2.h> //allows us to use the little LCD at the top
#include <Wire.h>

LCD16x2 lcd;

int intVal1 = 0, intVal2 = 0;
float floatVal1 = 0.0, floatVal2 = 0.0;

int E1 = 5;     //Port for M1 Speed Control
int E2 = 6;     //Port for M2 Speed Control

int M1 = 4;    //Port for M1 Direction Control
int M2 = 7;    //Port for M2 Direction Control

void setup(){ //this code executes once at start
int b;

  Wire.begin();


pinMode(E1, OUTPUT);    //These pins control the motor H-bridge circuitry, so we need to set them up as outputs
pinMode(E2, OUTPUT);
pinMode(M1, OUTPUT);
pinMode(M2, OUTPUT);
pinMode(A0, INPUT_PULLUP);  //Sets up optical sensor with correct pullup resistor
pinMode(A3, INPUT);  //Sets up the distance sensor

  lcd.lcdClear();
   
  lcd.lcdGoToXY(1,1);
  lcd.lcdWrite("Press white key");

  b = lcd.readButtons();    //Sample the state of the large white buttons

while(b == 15)    // A value of 15 will only be seen if no buttons are pressed
{
  b = lcd.readButtons();    //Sample the state of the large white buttons
}

// Will only proceed as far as here once a button is pressed
// We want execution to get "trapped" in the above loop: otherwise, the programme would turn on the motors straigh away, and your robot might go sailing off the bench!
}

int dist_read = analogRead(A3);    // Set variable for read from distance sensor
int dist_val = 24 //This is the output from the distance sensor when an object is 10cm away

int opt_state = 0, rot_count =0;
int black_val = 250;  // Defines the difference between black and non black colours read from optical sensor

int dist_map = map(dist_read,0,1023,0,100);    // Defines variable for read from distance sensor on a more intuitive scale (0-100)

void forward(int speed){    // Function for forward travel
  analogWrite (E1,speed);     //Set M1 speed
  digitalWrite(M1,HIGH);    //Set M1 direction 
 
  analogWrite (E2,speed);   //Set M2 speed 
  digitalWrite(M2,HIGH) ;   //Set M2 speed
}
  
void reverse(int speed){     // Function for reverse travel
  analogWrite (E1,speed);    //Set M1 speed
  digitalWrite(M1,LOW);     //Set M1 direction 
 
  analogWrite (E2,speed);    //Set M2 speed 
  digitalWrite(M2,LOW);     //Set M2 speed
}

 void stop(){
  forward(0);
  }
  
void lcdwrite(int count){    // Define function for writing text to lcd
    lcd.lcdWrite(count);     // Writes to lcd that black tape detected
    delay(500);           // sets a delay so that the value is readable
    lcd.lcdClear();     // Clears LCD
}


int optval =analogRead(A0);     // Sets variable for optical sensor reading

void rotatCount(int time){//This function spins the motors for a certain period of time to allow us to count the number of rotations.
  
  forward(250);
    
  if(optval > black_val && opt_state == 0){    // Checks if sensor is on black tape and that the state of optical sensor is not currently on black
      rot_count++;    // Adds 1 to value of rotation
      opt_state == 1;     // Sets optical state to 1 to prevent looping
      }
  else if (optval < black_val){    // Checks that sensor is not on black tape
      opt_state == 0;     // Resets optical state back to 0 so that black state can be re-entered
      }
    
  delay(time); 

  stop();
}

int rotatCounted = 0; //Sets up a variable to prevent looping of rotation counting.

int distState = 0;

void distTest(){
  forward(250);
  if(dist_read < dist_val && distState == 0){ //Checks if the sensor has detected an object 10cm away
    reverse(250);
    delay(600);
    stop();
  }else{
    forward(250);
  }
}

int distTested == 0;

void loop(){
  
  if(rotatCounted == 0){ // Checks to ensure the rotations have not been counted yet
    rotatCount(10000); // Counts the number of rotations for 10 seconds
    lcd.lcdClear(); //Clears the LCD
    lcd.lcdGoToXY(1,2);
    lcdwrite(rot_count);  // Displays rotation count
    rotatCounted == 1; //Prevents the rotations from being counted multiple times
  }

  lcd.lcdGoToXY(1,1);
  lcdwrite(optval);

  if(distTested == 0){ //Checks to ensure the distance test has not happened yet
    distTest(); // Runs the distance test
    distTested == 1; //Prevents the test from looping
  }


  }



  

 
