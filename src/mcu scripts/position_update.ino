//if (millis() - positionUpdateTimeStart >= timeStep){
//    timeStep = (millis() - _positionUpdateTimeStart)/1000; //Convert ms to s
//    
//
//    readEncoders();
//    int countL = encoderCountLR;
//    int countR = encoderCountRL;
//
//    float Dl = pi*2*r * (countL - _oldEPUEncoderCountL) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
//    float Dr = pi*2*r * (countR - _oldEPUEncoderCountR) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
//    
//    //Check integer roll over!
//    if (countL < 0 && _oldEPUEncoderCountL > 0 && _oldEPUEncoderCountL > 20000){
//      Dl = 2*pi*r * ((countL - (-32768)) + (32767 - _oldEPUEncoderCountL)) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
//    }
//    if (countR < 0 && _oldEPUEncoderCountR > 0 && _oldEPUEncoderCountL > 20000){
//      Dr = 2*pi*r * ((countR - (-32768)) + (32767 - _oldEPUEncoderCountR)) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
//    }
//    
//    float Dc = (Dr + Dl)/2; //Distance center of bot has moved read by encoders.
//
////    Linear_vr=Dr/timestep;
////    Linear_vl=Dl/timestep;
////    
////    oldEPUEncoderCountR = countR;
////    oldEPUEncoderCountL = countL;
////    
////    botA += (Dr - Dl)/axelLength;
////
////    botXPos += Dc * cos(_botA0 + (botA-botA0)/2);
////    botYPos += Dc * sin(_botA0 + (botA-botA0)/2);
////
////    bot_liner_Vel = (Dr + Dl)/(2*timeStep);
////  
////    botA0 = botA;
////    positionUpdateTimeStart = millis();
//  }  

