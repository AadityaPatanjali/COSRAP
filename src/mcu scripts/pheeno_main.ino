  #include "PheenoV2Basic.h"

// Instantiate Pheeno class
PheenoV2Basic my_robot = PheenoV2Basic(1);

#define pi 3.14


//Motor vaiables

float Linear_vel=40;  // in cm/sec
float angle=2.14;     // in rad/sec
float r = 1.6;        // Wheel radius in cm
float b = 13.37;      // axel length in cm

 
//constant variables
float compare_time;
float compare_time1;
float move_delay = 15000;


/* Encoder Matching PID Vairables */

//PID gains for encoder matching
float kp=0.1;
float ki=0.;
float kd=0;

// PID variables
float prev_error=0;
float error=0;
float integrl_error=0;
float diff_error=0;




float timeStep=100;
 int count_l ,count_r;
int count=0;
 
void setup() {
  Serial.begin(9600);
  my_robot.SetupBasic();
  compare_time = millis();
  
compare_time = millis();                              //Note the current loop time
}


void loop() {
   
  while (millis() - compare_time < move_delay) {      // The loop would execute for only the given time frame
      Serial.print("  Time  ");
      Serial.print(millis()-compare_time);
      
      // Move robot forward. Pheeno moves forward at a given speend (range 0-255) and angle as degrees
      if(millis()-compare_time<7000)
      {
      Pheeno_Move(Linear_vel,0);  
      }
      if(millis()-compare_time>7000 && millis()-compare_time<14000){
        Pheeno_Move(Linear_vel,angle);
      }
      
      Serial.print("  Pos L:  ");
      Serial.print(count_l);
      Serial.print("  Pos R:  ");
      Serial.print(count_r);
      Serial.print("  Linear distance R:  ");
      Serial.print(my_robot.botXPos);
      Serial.print("  Linear distance L:  ");
      Serial.print(my_robot.botYPos);
      Serial.print("  Velocity R:  ");
      Serial.println(my_robot.botVel);

  }

  Pheeno_Move(0,0);
}





