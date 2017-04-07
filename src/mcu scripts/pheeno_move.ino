
// Moves the Pheeno Forward.
// Applying a specific speed value (0-255) and both motors will apply the speed
// without error handling. Be careful!
void Pheeno_Move(int speed , float theta) {

//  
  count_l=my_robot.encoderCountLR;
  count_r=my_robot.encoderCountRL;
    
  //w=r*(count_l
  my_robot.encoderPositionUpdate(timeStep);

int speed_r = (2*speed+theta*b)/(2*r);
int speed_l = (2*speed-theta*b)/(2*r);
    speed_r = speed_r/r;
    speed_l = speed_l/r;
  my_robot.PIDMotorControlLR(speed_l);
  my_robot.PIDMotorControlRL(speed_r);

      
}


