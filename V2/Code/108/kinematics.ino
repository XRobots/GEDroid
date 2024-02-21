void kinematics (int leg, int xIn, int yIn, int zIn, float rIn, int interOn, int dur) {

    // roll axis
    #define bodyWidth 103      // half the distance from the middle of the body to the hip pivot  
    float legDiffRoll;            // differnece in height for each leg
    float legDiffRoll2;            // differnece in height for each leg
    float bodyDiffRoll;           // how much shorter the 'virtual body' gets
    float footDisplacementRoll;   // where the foot actually is
    float footDisplacementAngleRoll; // smaller angle
    float footWholeAngleRoll;     // whole leg angle
    float hipRollAngle;       // angle for hip when roll axis is in use
    float rollAngle;          // angle in RADIANS that the body rolls
    float zz1a;               // hypotenuse of final triangle
    float zz1;                // new height for leg to pass onto the next bit of code
    float yy1;                // new position for leg to move sideways


    float hipAngle1;
    float hipAngle1Degrees;
    float z2;

    #define length1 150   // thigh
    #define footHeight 50   // foot height to pivot
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

    float zIn2;
    float xIn2;
    float yIn2;
    float rIn2;

    // ** INTERPOLATION **
    // use Interpolated values if Interpolation is on
    if (interOn == 1) {
            
        if (leg == 1) {        // left
            zIn2 = interpFRZ.go(zIn,dur);
            xIn2 = interpFRX.go(xIn,dur);
            yIn2 = interpFRY.go(yIn,dur);
            //rIn2 = interpFRT.go(rIn,dur);
        }
      
        else if (leg == 2) {    // right
            zIn2 = interpFLZ.go(zIn,dur);
            xIn2 = interpFLX.go(xIn,dur);
            yIn2 = interpFLY.go(yIn,dur);
            //rIn2 = interpFLT.go(rIn,dur);           
        }     
    }    

    else {
      zIn2 = zIn;
      xIn2 = xIn;
      yIn2 = yIn;
      //rIn2 = rIn;  // do not interpolate roll here
    }
    
    // *** ROLL AXIS ***     
    
    if (leg == 2) {    // reverse the calcs for each side of the robot
      rIn = rIn *-1;
      yIn2 = yIn2*-1;
    }

    // convert roll angle to radians
    rollAngle = (PI/180) * rIn;    //covert degrees from the stick to radians

    // calc the top triangle sides
    legDiffRoll = sin(rollAngle) * bodyWidth;
    bodyDiffRoll = cos(rollAngle) * bodyWidth;

    // calc actual height from the ground for each side
    legDiffRoll2 = zIn2 + legDiffRoll + footHeight;   // ***  +footheight

    // calc foot displacement
    footDisplacementRoll = ((bodyDiffRoll - bodyWidth)*-1)-yIn2;

    //calc smaller displacement angle
    footDisplacementAngleRoll = atan(footDisplacementRoll/legDiffRoll2);

    //calc distance from the ground at the displacement angle (the hypotenuse of the final triangle)
    zz1a = legDiffRoll2/cos(footDisplacementAngleRoll); 

    // calc the whole angle for the leg
    footWholeAngleRoll = footDisplacementAngleRoll - rollAngle;

    //calc actual leg length - the new Z to pass on
    zz1 = (cos(footWholeAngleRoll) * zz1a) - footHeight;

    //calc new Y to pass on
    yy1 = sin(footWholeAngleRoll) * zz1a;


    if (leg == 2) {   // reverse the calcs for each side of the robot
      yy1 = yy1*-1;
    }
 
  
    // *** TRANSLATION AXIS ***

    // calculate the hip joint and new leg length based on how far the robot moves sideways
    hipAngle1 = atan(yy1/(zz1 + footHeight));    // ***  +footHeight   
    hipAngle1Degrees = ((hipAngle1 * (180/PI)));   // convert to degrees
    z2 = ((zz1)/cos(hipAngle1));     

    // calculate the shoulder joint offset and new leg length based on now far the foot moves forward/backwards
    shoulderAngle2 = atan(xIn2/z2);     // calc how much extra to add to the shoulder joint    
    z3 = z2/cos(shoulderAngle2);     // calc new leg length to feed to the next bit of code below

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
        pos1 = 180-(shoulderAngleServo - shoulderAngleServo2);  // SHOULDER
        pos2 = 180+(kneeAngleServo + shoulderAngleServo2);      // KNEE
        
        pos2 = pos2 + 5;    // servo trim
        pos1 = pos1 - 2;    // servo trim
        
        pos5 = 180 + (hipAngle1Degrees * 2);                    // HIP - there is a 2:1 mechanical reduction on the servo

        // convert to encoder counts

        //pos1 = constrain(pos1, 170,300); 
        //pos2 = constrain(pos2, 100,190);

        pos5 = pos5 + 2;        

        pos1a = int(pos1 * 11.375);
        pos2a = int(pos2 * 11.375);
        pos5a = int(pos5 * 11.375);
    }    
    
    else if (leg == 2) { // *** RIGHT ***
        pos3 = 180+(shoulderAngleServo - shoulderAngleServo2);  // SHOULDER
        pos4 = 180-(kneeAngleServo + shoulderAngleServo2);      // KNEE

        pos3 = pos3 + 1;    // servo trim

        pos4 = pos4 - 5;

        pos6 = 180 + (hipAngle1Degrees * 2);                    // HIP - there is a 2:1 mechanical reduction on the servo

        // convert to encoder counts

        //pos3 = constrain(pos3,100,190);
        //pos4 = constrain(pos4, 170,300);

        pos3a = int(pos3 * 11.375);
        pos4a = int(pos4 * 11.375); 
        pos6a = int(pos6 * 11.375);
    }

    

   

  
}

