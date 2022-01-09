void GS_ReturnToZeroYaw(void)
{
    float yaw_angle, err, yaw_prev, rot_dir, rot;
    int t;

    // Angles in degrees
    err = 0.5;
    yaw_angle = navxGyro->GetYaw();
    frc::SmartDashboard::PutNumber("Yaw Start",yaw_angle);

    t = 0;
    rot_dir = 1;

    while (abs(yaw_angle) > err && t < 15000)
    {
        // adjustment of yaw to zero
        //rot = FindMax(abs(yaw_angle)/180, 0.1)*FindSign(yaw_angle);
        rot = 0.1*FindSign(yaw_angle)*rot_dir;

        gainP = frc::SmartDashboard::GetNumber("driveP", 0.7);
        gainI = frc::SmartDashboard::GetNumber("driveI", 0);
        gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

        SwerveDriveMe(0, 0, rot , 0, false, 0, gainP, gainI, gainD);

        // Move
        Locomotion(AutonomousDrive);   
        frc::Wait(0.001);  

        yaw_prev = yaw_angle;
        yaw_angle = navxGyro->GetYaw();

        frc::SmartDashboard::PutNumber("Yaw",yaw_angle);
        frc::SmartDashboard::PutNumber("iter",t);

        if (abs(yaw_angle) > abs(yaw_prev) && FindSign(yaw_angle) == FindSign(yaw_prev))
        {
            rot_dir = -rot_dir;
        }

        t++;
    }

}

void GS_StraightPickup(int loop1count, int loop2count)
{
    float rot, bodyAng, strf, bodyAng_prev, rot_dir;
    int t, u;
    bool CellIntakeStatusCurrent;
    bool CellIntakeNotDetected = true;         
    
    CellIntakeStatusCurrent = CellIntakeSensor->Get();
        
    SetMotorsToBrake(); 
    //SetIntakeMotorsOn();     
    
    gainP = frc::SmartDashboard::GetNumber("driveP", 0.85);
    gainI = frc::SmartDashboard::GetNumber("driveI", 0);
    gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

    // First Section - Move Forward with Arm Up and Motors Off
    t = 0;
    while (t < loop1count)
    {
        SwerveDriveMe(0.5, 0, rot, 0, false, 0, gainP, gainI, gainD);

        // Move
        Locomotion(AutonomousDrive);    
        frc::Wait(0.00001);

        t++;
    }
    
    //AutonMove(0,0.5,0,false,0,200); 
    AutonMove(0,0,0,false,0,100); 

    // Second Section - Move Forward with Arm Down and Motors On
    SetIntakeMotorsOn();  
    frc::Wait(1);

    t = 0;
    rot_dir = 1; 
 
    while (CellIntakeStatusCurrent == CellIntakeNotDetected && t < loop2count)
    {
        
        bodyAng_prev = bodyAng;
        bodyAng = navxGyro->GetYaw();
        frc::SmartDashboard::PutNumber("Yaw", bodyAng);

        pc_err = frc::SmartDashboard::GetNumber("cc", 1000) + 640;

        rot = bodyAng/15;

        if (abs(rot) > 0.5)
            rot = 0.5*FindSign(rot);
        
        frc::SmartDashboard::PutNumber("Rot", rot);

        // strafe, if necessary
        if (pc_err < 1280 && pc_err > 0)       
        { 
            strf = (pc_err-640)/640*0.3; 
        }
        else
        {
            strf = 0;
        }         
        
        if (abs(strf) > 0.5)
            strf = 0.5*FindSign(strf);
        
        SwerveDriveMe(0.5, strf, rot, 0, false, 0, gainP, gainI, gainD);

        // Move
        Locomotion(AutonomousDrive);    
        frc::Wait(0.00001);

        // Check Cell Intake       
        CellIntakeStatusCurrent = CellIntakeSensor->Get();

        t++;    
    }

    SetIntakeMotorsOff(); 
    //SwerveDriveMe(0, 0, 0, 0, false, 0, gainP, gainI, gainD);
    //Locomotion(AutonomousDrive);
    AutonMove(0,0,0,false,0,100); 
    //frc::Wait(1);        
    SetMotorsToCoast();     
    
}

void GS_Align(void)
{
    bool PowerCellNotDetected, hasCameraPickedUpPowerCell;
    float strf, rot, fwd, current_angle, rot1, ang;
    float a, b, c, d;
    int loopcount;
    
    float cc_err = 50;          // +/-pixels
    float cc_desired_pos = 640; // pixel

    float angle_err = 1;           // +/- degrees
    float desired_angle = 0;       // degree

    //float area_err = 0.03;          // % pixels^2
    float desired_area = 48000;     // pixels^2

    // Calculate max and min values
    float cc_min = cc_desired_pos - cc_err;
    float cc_max = cc_desired_pos + cc_err;

    float area_range = abs(pc_area - desired_area);
    
    //float area_min = desired_area - area_err*area_range;
    //float area_max = desired_area + area_err*area_range;

    float angle_min = desired_angle - angle_err;
    float angle_max = desired_angle + angle_err;

    // This function picks up a powercell once it is straight in front of the Robot
    pc_err = frc::SmartDashboard::GetNumber("cc", 1000) + 640;
    pc_area = frc::SmartDashboard::GetNumber("area", 1000);
    current_angle = navxGyro->GetYaw();
    PowerCellNotDetected = CellIntakeSensor->Get();
    hasCameraPickedUpPowerCell = false;

    SetMotorsToBrake();   
    
    // two things we need to keep adjusting here:
    // 1 - strafing to keep powercell at centerline
    // 2. body rotation to match StartAngle
    d = 0;        

    while(d < 1)
    {
        a = 0;
        b = 0;
        c = 0;

        strf = 0;
        fwd = 0;
        rot = 0;
        
        // -----------------------------------------------------------------------------
        // Calculate Strafe to keep Powercell at Centerline +/- error  
        
        if (pc_err <= 1280)       
            strf = (pc_err-640)/640*0.3;   
        
        strf = FindMax(abs(strf), 0.06)*FindSign(strf);

        if (pc_err >= cc_min && pc_err <= cc_max)
        {
            //strf = 0;
            a = 1;
        }  
                             
        // -----------------------------------------------------------------------------

        // -----------------------------------------------------------------------------
        // Calculate Rotate to keep Robot body angle consistent
        // ang = navxGyro->GetYaw();

        // if (abs(ang) > 1)
        //     rot = 0.08*FindSign(ang);
        // else
        //     rot = ang*0.08;

        // if (current_angle >= angle_min && current_angle <= angle_max)
        // {            
        //     b = 1;
        // }        
        // -----------------------------------------------------------------------------

        // // -----------------------------------------------------------------------------
        // Calculate Forward to approach powercell
        // if (pc_area >= desired_area)
        // {
        //     fwd = 0;
        //     c = 1;
        // }
        // else
        // {
        //     if (pc_err <= 1280)     // move forward only if camera pickes up powercell
        //     {
        //         fwd = 0.1;
        //     } 
            
        //     c = 0;   
        // }
        
        d = a + b + c;

        frc::SmartDashboard::PutNumber("a", a);
        frc::SmartDashboard::PutNumber("b", b);
        frc::SmartDashboard::PutNumber("c", c);
        frc::SmartDashboard::PutNumber("str", strf);
        frc::SmartDashboard::PutNumber("pcerr", pc_err);

        // -----------------------------------------------------------------------------

        if (d < 1)
        {
            //Read PID gains from SmartDashboard
            gainP = frc::SmartDashboard::GetNumber("driveP", 0.7);
            gainI = frc::SmartDashboard::GetNumber("driveI", 0);
            gainD = frc::SmartDashboard::GetNumber("driveD", 0.08);

            // Calculate Swerve Drive Parameters
            //SwerveDriveMe(fwd, FindBound(abs(strf), 0.01, 0.1)*FindSign(strf), rot , 0, false, 0, gainP, gainI, gainD);
            SwerveDriveMe(fwd, strf, rot, 0, false, 0, gainP, gainI, gainD);

            // Move
            Locomotion(AutonomousDrive);   
            frc::Wait(0.001);           
        }     

        // Remeasure all values
        pc_err = frc::SmartDashboard::GetNumber("cc", 1000) + 640;
        pc_area = frc::SmartDashboard::GetNumber("area", 1000);
        current_angle = navxGyro->GetYaw();
        PowerCellNotDetected = CellIntakeSensor->Get();
        
    }

    // Calculate Swerve Drive Parameters
    SwerveDriveMe(0, 0, 0 , 0, false, 0, gainP, gainI, gainD);

    // Move
    Locomotion(AutonomousDrive);   
    frc::Wait(0.5);
    
    SetMotorsToCoast();
}


