int forward = -1;
int backward =1;
int left = -1;
int right = 1;
int rotdir = -1;
int loopcount;
float da = 18*deg2rad;

//--------------------------------------------------------------------------------------------------
void CircleMove1(int loopCount, float magnitude)
{
    int r;
    float angle, fd, rt;
    
    angle = 0;
    fd = 0;
    rt = 0;

    //Read PID gains from SmartDashboard
    gainP = frc::SmartDashboard::GetNumber("driveP", 0.085);		
	gainI = frc::SmartDashboard::GetNumber("driveI", 0);
	gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

    while (angle <= 90*deg2rad)
    {
        fd = magnitude*cos(angle)*forward;
        rt = magnitude*sin(angle)*right;

        r = 0;
        while(r < loopCount)
        {
            // Calculate Swerve Drive Parameters
	        SwerveDriveMe(-fd*0.8*forward, -rt*0.8*right, 0, 0, true, 0, gainP, gainI, gainD);

	        // Move
	        Locomotion(TeleopDrive);   

            frc::Wait(0.005);

            r++; 
        }

        angle = angle + da;
    }
    
}

void CircleMove2(int loopCount, float magnitude)
{
    int r;
    float angle, fd, rt;
    
    angle = 90*deg2rad;
    fd = 0;
    rt = 0;

    //Read PID gains from SmartDashboard
    gainP = frc::SmartDashboard::GetNumber("driveP", 0.085);		
	gainI = frc::SmartDashboard::GetNumber("driveI", 0);
	gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

    while (angle <= 180*deg2rad)
    {
        fd = magnitude*cos(angle)*forward;
        rt = magnitude*sin(angle)*right;

        r = 0;
        while(r < loopCount)
        {
            // Calculate Swerve Drive Parameters
	        SwerveDriveMe(-fd*0.8*forward, -rt*0.8*right, 0, 0, true, 0, gainP, gainI, gainD);

	        // Move
	        Locomotion(TeleopDrive);   

            frc::Wait(0.005);

            r++; 
        }

        angle = angle + da;
    }
    
}

void CircleMove3(int loopCount, float magnitude)
{
    int r;
    float angle, fd, rt;
    
    angle = 180*deg2rad;
    fd = 0;
    rt = 0;

    //Read PID gains from SmartDashboard
    gainP = frc::SmartDashboard::GetNumber("driveP", 0.085);		
	gainI = frc::SmartDashboard::GetNumber("driveI", 0);
	gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

    while (angle <= 270*deg2rad)
    {
        fd = magnitude*cos(angle)*forward;
        rt = magnitude*sin(angle)*right;

        r = 0;
        while(r < loopCount)
        {
            // Calculate Swerve Drive Parameters
	        SwerveDriveMe(-fd*0.8*forward, -rt*0.8*right, 0, 0, true, 0, gainP, gainI, gainD);

	        // Move
	        Locomotion(TeleopDrive);   

            frc::Wait(0.005);

            r++; 
        }

        angle = angle + da;
    }
    
}

void CircleMove4(int loopCount, float magnitude)
{
    int r;
    float angle, fd, rt;
    
    angle = 270*deg2rad;
    fd = 0;
    rt = 0;

    //Read PID gains from SmartDashboard
    gainP = frc::SmartDashboard::GetNumber("driveP", 0.085);		
	gainI = frc::SmartDashboard::GetNumber("driveI", 0);
	gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

    while (angle <= 360*deg2rad)
    {
        fd = magnitude*cos(angle)*forward;
        rt = magnitude*sin(angle)*right;

        r = 0;
        while(r < loopCount)
        {
            // Calculate Swerve Drive Parameters
	        SwerveDriveMe(-fd*0.8*forward, -rt*0.8*right, 0, 0, true, 0, gainP, gainI, gainD);

	        // Move
	        Locomotion(TeleopDrive);   

            frc::Wait(0.005);

            r++; 
        }

        angle = angle + da;
    }
    
}

void CircleMove5(int loopCount, float magnitude)
{
    int r;
    float angle, fd, rt;
    
    angle = 0;
    fd = 0;
    rt = 0;

    //Read PID gains from SmartDashboard
    gainP = frc::SmartDashboard::GetNumber("driveP", 0.085);		
	gainI = frc::SmartDashboard::GetNumber("driveI", 0);
	gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

    while (angle <= 90*deg2rad)
    {
        fd = magnitude*cos(angle)*forward;
        rt = -magnitude*sin(angle)*right;

        r = 0;
        while(r < loopCount)
        {
            // Calculate Swerve Drive Parameters
	        SwerveDriveMe(-fd*0.8*forward, -rt*0.8*right, 0, 0, true, 0, gainP, gainI, gainD);

	        // Move
	        Locomotion(TeleopDrive);   

            frc::Wait(0.005);

            r++; 
        }

        angle = angle + da;
    }
    
}

void CircleMove6(int loopCount, float magnitude)        // Mirror of CircleMove(2)
{
    int r;
    float angle, fd, rt;
    
    angle = 90*deg2rad;
    fd = 0;
    rt = 0;

    //Read PID gains from SmartDashboard
    gainP = frc::SmartDashboard::GetNumber("driveP", 0.085);		
	gainI = frc::SmartDashboard::GetNumber("driveI", 0);
	gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

    while (angle <= 180*deg2rad)
    {
        fd = magnitude*cos(angle)*forward;
        rt = magnitude*sin(angle)*right;

        r = 0;
        while(r < loopCount)
        {
            // Calculate Swerve Drive Parameters
	        SwerveDriveMe(-fd*0.8*forward, rt*0.8*right, 0, 0, true, 0, gainP, gainI, gainD);

	        // Move
	        Locomotion(TeleopDrive);   

            frc::Wait(0.005);

            r++; 
        }

        angle = angle + da;
    }
    
}

void CircleMove7(int loopCount, float magnitude)        // Mirror of CircleMove3()
{
    int r;
    float angle, fd, rt;
    
    angle = 180*deg2rad;
    fd = 0;
    rt = 0;

    //Read PID gains from SmartDashboard
    gainP = frc::SmartDashboard::GetNumber("driveP", 0.085);		
	gainI = frc::SmartDashboard::GetNumber("driveI", 0);
	gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

    while (angle <= 270*deg2rad)
    {
        fd = magnitude*cos(angle)*forward;
        rt = magnitude*sin(angle)*right;

        r = 0;
        while(r < loopCount)
        {
            // Calculate Swerve Drive Parameters
	        SwerveDriveMe(-fd*0.8*forward, rt*0.8*right, 0, 0, true, 0, gainP, gainI, gainD);

	        // Move
	        Locomotion(TeleopDrive);   

            frc::Wait(0.005);

            r++; 
        }

        angle = angle + da;
    }
    
}


void CircleMove8(int loopCount, float magnitude)    // Mirror of CircleMove4()
{
    int r;
    float angle, fd, rt;
    
    angle = 270*deg2rad;
    fd = 0;
    rt = 0;

    //Read PID gains from SmartDashboard
    gainP = frc::SmartDashboard::GetNumber("driveP", 0.085);		
	gainI = frc::SmartDashboard::GetNumber("driveI", 0);
	gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

    while (angle <= 360*deg2rad)
    {
        fd = magnitude*cos(angle)*forward;
        rt = magnitude*sin(angle)*right;

        r = 0;
        while(r < loopCount)
        {
            // Calculate Swerve Drive Parameters
	        SwerveDriveMe(-fd*0.8*forward, rt*0.8*right, 0, 0, true, 0, gainP, gainI, gainD);

	        // Move
	        Locomotion(TeleopDrive);   

            frc::Wait(0.005);

            r++; 
        }

        angle = angle + da;
    }
    
}
//--------------------------------------------------------------------------------------------------
void CircleAroundRight(float side, int loopCount, float mag)
{
    int q, r;
    float angle, fd, rt;
    
    angle = 0;
    fd = 0;
    rt = 0;

    //Read PID gains from SmartDashboard
    gainP = frc::SmartDashboard::GetNumber("driveP", 0.085);		
	gainI = frc::SmartDashboard::GetNumber("driveI", 0);
	gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

    for (q=0;q<side;q++)
    {
        angle = angle + da;
        fd = mag*cos(angle)*forward;
        rt = mag*sin(angle)*right;

        r = 0;
        while(r < loopCount)
        {
            // Calculate Swerve Drive Parameters
	        SwerveDriveMe(-fd*0.8*forward, -rt*0.8*right, 0, 0, true, 0, gainP, gainI, gainD);

	        // Move
	        Locomotion(TeleopDrive);   

            frc::Wait(0.005);

            r++; 
        }
    }
}

void CircleAroundLeft(float side, int loopCount, float mag)
{
    int q, r;
    float angle, fd, rt;
    
    angle = 0;
    fd = 0;
    rt = 0;

    //Read PID gains from SmartDashboard
    gainP = frc::SmartDashboard::GetNumber("driveP", 0.085);		
	gainI = frc::SmartDashboard::GetNumber("driveI", 0);
	gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

    //for (q=0;q<side;q++)
    for (q=0;q<15;q++)
    {
        angle = angle + da;
        fd = mag*cos(angle)*forward;
        rt = mag*sin(angle)*right;

        r = 0;
        while(r < loopCount)
        {
            // Calculate Swerve Drive Parameters
	        SwerveDriveMe(-fd*0.8*forward, rt*0.8*right, 0, 0, true, 0, gainP, gainI, gainD);

	        // Move
	        Locomotion(TeleopDrive);   

            frc::Wait(0.005);

            r++; 
        }
    }
}
void CircleAroundLeft2(float side, int loopCount, float mag)
{
    int q, r;
    float angle, fd, rt;
    
    angle = 0;
    fd = 0;
    rt = 0;

    //Read PID gains from SmartDashboard
    gainP = frc::SmartDashboard::GetNumber("driveP", 0.085);		
	gainI = frc::SmartDashboard::GetNumber("driveI", 0);
	gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

    //for (q=0;q<side;q++)
    for (q=0;q<10;q++)
    {
        angle = angle + da;
        fd = mag*cos(angle)*forward;
        rt = mag*sin(angle)*right;

        r = 0;
        while(r < loopCount)
        {
            // Calculate Swerve Drive Parameters
	        SwerveDriveMe(-fd*0.8*forward, rt*0.8*right, 0, 0, true, 0, gainP, gainI, gainD);

	        // Move
	        Locomotion(TeleopDrive);   

            frc::Wait(0.005);

            r++; 
        }
    }
}

void TurnToFieldDirection(float angle)
{
    float yaw_angle, err, rot;
    int t;

    // Angles in degrees
    err = 0.25;
    yaw_angle = navxGyro->GetYaw() - angle;   

    while (abs(yaw_angle) > err && t < 3000)
    {
        // adjustment of yaw angle
        rot = FindMax(abs(yaw_angle)/180, 0.1)*FindSign(yaw_angle);

        gainP = frc::SmartDashboard::GetNumber("driveP", 0.7);
        gainI = frc::SmartDashboard::GetNumber("driveI", 0);
        gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

        SwerveDriveMe(0, 0, rot , 0, false, 0, gainP, gainI, gainD);

        // Move
        Locomotion(AutonomousDrive);   
        frc::Wait(0.0001);  

        yaw_angle = navxGyro->GetYaw() - angle;
        t++;
    }

}

void AutonMove(float X, float Y, float W, bool isSetFaceAngle, float faceAngle, int loopCount)   // Just move for a specified amount of loopcounts at a specific throttle speed
{
    float A, waittime;
    int t;
    int y;
    
    y = 0;
    waittime = 0.005;

    //Read PID gains from SmartDashboard
    gainP = frc::SmartDashboard::GetNumber("driveP", 0.07);
    gainI = frc::SmartDashboard::GetNumber("driveI", 0);
    gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

    while(y <= loopCount)
    {
        A = navxGyro->GetYaw(); 

	    // Calculate Swerve Drive Parameters
	    SwerveDriveMe(Y*0.8, -X*0.8, -W*0.4, A, false, 0, gainP, gainI, gainD);

	    // Move
	    Locomotion(AutonomousDrive);         

        y++;
        frc::Wait(waittime);
    } 
	
	// Stop robot once loopcount is over
	//throttleMotor0PIDController.SetReference(0, rev::ControlType::kVelocity); 
	//throttleMotor1PIDController.SetReference(0, rev::ControlType::kVelocity); 
	//throttleMotor2PIDController.SetReference(0, rev::ControlType::kVelocity); 
	//throttleMotor3PIDController.SetReference(0, rev::ControlType::kVelocity);     
}

void AutonMove2(float X, float Y, float W, bool isSetFaceAngle, float faceAngle, int loopCount, int noOfObjects, int LidarSide)   // Just move for a specified amount of loopcounts at a specific throttle speed
{
    float A, waittime;
    int t;
    int y;
    int noOfObjectsPassed = 0;
    bool LidarState = false;
    bool LidarStatePrevious = false;
    bool LidarStatePreviouser = false;
    
    y = 0;
    waittime = 0.005;

    //Read PID gains from SmartDashboard
    gainP = frc::SmartDashboard::GetNumber("driveP", 0.07);
    gainI = frc::SmartDashboard::GetNumber("driveI", 0);
    gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

    while(y <= loopCount && noOfObjectsPassed < noOfObjects)
    //while(y <= loopCount)
    {
        A = navxGyro->GetYaw(); 

	    // Calculate Swerve Drive Parameters
	    SwerveDriveMe(Y*0.8, -X*0.8, -W*0.4, A, false, 0, gainP, gainI, gainD);

	    // Move
	    Locomotion(AutonomousDrive);         

        y++;
        frc::Wait(waittime);
        
        if (LidarSide == LeftLidarID)     // Left Lidar
        {
            LidarStatePrevious = LidarState;
            LidarState = CheckLeftLidar();
        }

        if (LidarSide == RightLidarID)     // Right Lidar
        {
            LidarStatePreviouser = LidarStatePrevious;
            LidarStatePrevious = LidarState;
            LidarState = CheckRightLidar();
        }

        if (LidarStatePrevious == true && LidarState == false)   // meaning we just cleared an object
            noOfObjectsPassed++;

        frc::SmartDashboard::PutNumber("No of Objects", noOfObjectsPassed);
        if (noOfObjectsPassed == noOfObjects)
        {
            loopCount = y + 1;
         }
    } 
	
	// Stop robot once loopcount is over
	throttleMotor0PIDController.SetReference(0, rev::ControlType::kVelocity); 
	throttleMotor1PIDController.SetReference(0, rev::ControlType::kVelocity); 
	throttleMotor2PIDController.SetReference(0, rev::ControlType::kVelocity); 
	throttleMotor3PIDController.SetReference(0, rev::ControlType::kVelocity);     
}

void AutonMoveRotate(float X, float Y, float faceAngle, int loopCount)   // Just move for a specified amount of loopcounts while rotating at a specific throttle speed
{
    float A, waittime;
    int t;
    int y;

    float yaw_angle, err, rot;
 
    // Angles in degrees
    err = 0.25;
    yaw_angle = navxGyro->GetYaw() - faceAngle;  
    
    y = 0;
    waittime = 0.005;

    //Read PID gains from SmartDashboard
    gainP = frc::SmartDashboard::GetNumber("driveP", 0.07);
    gainI = frc::SmartDashboard::GetNumber("driveI", 0);
    gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

    while(y <= loopCount)
    {
        A = navxGyro->GetYaw(); 

        // adjustment of yaw to zero
        rot = FindMax(abs(yaw_angle)/180, 0.1)*FindSign(yaw_angle);

	    // Calculate Swerve Drive Parameters
	    SwerveDriveMe(Y*0.8, -X*0.8, rot, A, false, 0, gainP, gainI, gainD);

	    // Move
	    Locomotion(AutonomousDrive);         

        yaw_angle = navxGyro->GetYaw() - faceAngle;
        y++;
        frc::Wait(waittime);
    } 
	
	// Stop robot once loopcount is over
	throttleMotor0PIDController.SetReference(0, rev::ControlType::kVelocity); 
	throttleMotor1PIDController.SetReference(0, rev::ControlType::kVelocity); 
	throttleMotor2PIDController.SetReference(0, rev::ControlType::kVelocity); 
	throttleMotor3PIDController.SetReference(0, rev::ControlType::kVelocity);     
}

