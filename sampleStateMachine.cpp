// This is never-compiled code. Expect errors. Use at your own risk.

// in variable declarations
  int autoState;
  float autoEncoderTrigger;
  float autoGyroTrigger;


void AutonomousInit() {
  if (left) {
    if (defense) {
      autoState = 1100
    } else {
      autoState = 2100
    }
  } else if (center) {
    if (defense) {
      autoState = 1200
    } else {
      autoState = 2200
    }
  } else {
     // and so on ...
  }
}

void AutonomousPeriodic() {
  bool encoderTriggered;
  switch(autoState) {
    default: break;
    
    case 1100:
      // go forward
      myRobot.TankDrive(100, 100);
      
      // 10 feet
      encoder.Reset();
      autoEncoderTrigger = 10.0;

      // go to next state
      autoState += 1;
      break;
    
    case 1101:
      // If we're done going forward
      if (encoder.GetDistance() >= autoEncoderTrigger) {
        // Pivot
        myRobot.TankDrive(100, -100);
        // 90 degrees
        gyro.Reset();
        autoEncoderAngle = 90;
        
        // go to next state
        autoState += 1;
      }
      // otherwise, do nothing
      break;
      
    case 1102:
      // If we're done pivoting
      if (gyro.GetAngle() >= autoGyroTrigger) {
        // and so on...
        autoState = 3000;
      }
      break;
      
    case 3000: // pick up and pivot
    
    case 3030:
      if (left
  }
}
