void kinematics (int leg, int xIn, int yIn, int zIn, int interOn, int dur) {

    #define length1 150 
    float angle1;
    float angle1a;
    float angle1b;
    float angle1c;
    float angle2;
    float angle2Degrees;
    float angle2aDegrees;
    float angle1Degrees;
    float z3;
    float shoulderAngle2;
    float shoulderAngle2Degrees;

    float shoulderAngleServo;
    float shoulderAngleServo2;
    float kneeAngleServo;

    float pos1;
    float pos2;
    float pos3;
    float pos4;

    // ** INTERPOLATION **
    // use Interpolated values if Interpolation is on
    if (interOn == 1) {

        //Serial.print(zIn);
        //Serial.print(" , ");
            
        if (leg == 1) {        // front right
            z = interpFRZ.go(zIn,dur);
            x = interpFRX.go(xIn,dur);
            y = interpFRY.go(yIn,dur);
        }
      
        else if (leg == 2) {    // front left
            z = interpFLZ.go(zIn,dur);
            x = interpFLX.go(xIn,dur);
            y = interpFLY.go(yIn,dur);           
        }

        //Serial.print(z);
        //Serial.println();

    }

    else {
      z = zIn;
      x = xIn;
      y = yIn;
    }

    // calculate the shoulder joint offset and new leg length based on now far the foot moves forward/backwards
    shoulderAngle2 = atan(x/z);     // calc how much extra to add to the shoulder joint    
    z3 = z/cos(shoulderAngle2);     // calc new leg length to feed to the next bit of code below

    // calculate leg length based on shin/thigh length and knee and shoulder angle
    angle1a = sq(length1) + sq(z3) - sq(length1);
    angle1b = 2 * length1 * z3;
    angle1c = angle1a / angle1b;
    angle1 = acos(angle1c);     // hip angle RADIANS
    angle2 = PI - (angle1 * 2); // knee angle RADIANS

    //calc degrees from angles
    angle2Degrees = angle2 * (180/PI);    // knee angle DEGREES
    angle2aDegrees = angle2Degrees / 2;    // half knee angle for each servo DEGREES
    shoulderAngle2Degrees = shoulderAngle2 * (180/PI);    // front/back shoulder offset DEGREES

    shoulderAngleServo = (angle2aDegrees-45);        // remove defualt angle offset. Multiple degrees to get servo angle
    kneeAngleServo = (angle2aDegrees-45);
    shoulderAngleServo2 = shoulderAngle2Degrees;

    // ***** WRITE TO DYNAMIXELS *****

    if(leg == 1) {      // *** LEFT ***
        pos1 = 180+(shoulderAngleServo - shoulderAngleServo2);
        pos2 = 180-(kneeAngleServo + shoulderAngleServo2);

        pos1 = constrain(pos1, 170,300); 
        pos2 = constrain(pos2, 100,190);

        if (ch8 > 1300) {     // motor enable

          dxl.setGoalPosition(0, pos1, UNIT_DEGREE);  // higher = longer leg      RIGHT SHOULDER
          dxl.setGoalPosition(3, pos2, UNIT_DEGREE);  // lower = longer leg       RIGHT KNEE

        }

    }
    else if (leg == 2) { // *** RIGHT ***
        pos3 = 180-(shoulderAngleServo - shoulderAngleServo2);
        pos4 = 180+(kneeAngleServo + shoulderAngleServo2);

        pos3 = constrain(pos3,100,190); 
        pos4 = constrain(pos4, 170,300);

        if (ch8 > 1300) {     // motor enable
  
          dxl.setGoalPosition(1, pos3, UNIT_DEGREE);  // lower = longer leg      LEFT SHOULDER
          dxl.setGoalPosition(2, pos4, UNIT_DEGREE);  // higher = longer leg     LEFT KNEE

        }

    }













    

  
}

