// Group 4 lab 3 Report - Robert Rafanelli, Hugh Fitzpatrick and Amelie Cojocaru
// We focused our efforts in this lab on improving the structural integrity of our robot and writing a working script for the assigned challenge. 
// We started from the ground up, and emphasized accomodation for the PSU, the Arduino, possible Gear Trains, and bracing assemblies. Ensuring the possibility for future modularity took precedence as well.
// We used building techniques shown to us at the start of the lab to improve the integrity of our robot and prevent it from breaking as the motors spun.
// We retained the re-purposed axel sent through the bottom for support, despite advisement against it, due to tests that applied pressure to the Arduino displaying utility in providing structural support across the middle. No functional fixedness with Team 4!
// From our newly supported base, we improved our third wheel assembly with added support and superior positioning relative to its position. Tension was increased for the tracks, and the robot manuevered because of it.
// The motor was a priority in our redesign, and properly securing it to the rest of the machine ensured that the coming gear train functioned properly, and that the robots most integral component was secure.
// We leveled out all of vertical stages of the robot from the ground up, and supported each stage with the fastening technique. This leveled out positioning enabled the Arduino module to sit on a level plane with optimal security.
// In the meantime, we also created a script to count the number of rotations of an output gear that we connected to the primary motor's gear to create a gear train
// This new geartrain was integrated into the front assmebly of the robot, and was secured utilizing a series of axels fed through Technic bricks and was supplemented by additional fastening support along the side assembly. 
// One flaw however in our geartrain design was the lack of technic bricks on the other side of the gears. Although there were technic bricks to support the axels and gears being mounted, the assembly was lodged against the rest of the machine (primarily the PSU).
// As a consequence of this, the smaller gear that was used to connect the motor gear and the large gear (the large gear is the one being used to count rotations) would fall through if the axel came undone. This was a frequent occurence because the gear is not machined to fit to a technic axel, and is instead just rounded.
// A piece of black tape was placed on this gear, and an optical sensor was used to detect when this black tape had completed one full rotation.
// Rising edge detection had to be used to prevent the function from looping and not counting the correct amount of rotations.
// We also cleaned up our code from the previous weeks by creating functions for code that was being used repeatedly.
// In addition to standard code garbage disposal, we organized the code into a comprehensible structure as opposed to having variables and functions declared irrespective of code position.
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

int display_delay = delay(5000) // Gives us time to observe the data displayed on the lcd screen 

int dist_read = analogRead(A3);    // Set variable for read from distance sensor
int dist_val = 24 //This is the output from the distance sensor when an object is 10cm away

int opt_state = 0, rot_count =0;
int black_val = 250;  // Defines the difference between black and non black colours read from optical sensor

int dist_map = map(dist_read,0,1023,0,100);    // Defines variable for read from distance sensor on a more intuitive scale (0-100)

int optval =analogRead(A0);     // Sets variable for optical sensor reading

int rotatCounted = 0; //Sets up a variable to prevent looping of rotation counting.

int distState = 0;

int distTested == 0;

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

void stop(){ // Stops the motors entirely by utilizing the forward function and setting it to zero
  forward(0);
}
  
void lcdwrite(int count){    // Define function for writing text to lcd
    lcd.lcdWrite(count);     // Writes to lcd that black tape detected
    delay(500);           // sets a delay so that the value is readable
    lcd.lcdClear();     // Clears LCD
}

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

void loop(){
  // Goal of this challenge (Challenge #3) is to drive slowly towards the wall, stop when it's at 10cm away, and then reverse 20cm 
  if(rotatCounted == 0){ // Checks to ensure the rotations have not been counted yet
    rotatCount(10000); // Counts the number of rotations for 10 seconds
    lcd.lcdClear(); //Clears the LCD
    lcd.lcdGoToXY(1,2);
    lcdwrite(rot_count);  // Displays rotation count
    display_delay; // 5 seconds of time to observe data
    rotatCounted == 1; //Prevents the rotations from being counted multiple times
  }

  lcd.lcdGoToXY(1,1);
  lcdwrite(optval);

  if(distTested == 0){ //Checks to ensure the distance test has not happened yet
    distTest(); // Runs the distance test
    distTested == 1; //Prevents the test from looping
  }

}



  

 
