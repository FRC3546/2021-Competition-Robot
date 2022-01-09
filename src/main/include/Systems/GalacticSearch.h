void GS_B_Red(void)
{
    int t = 0;
    
    SetIntakeMotorsOn();	
    frc::Wait(1);

    // Pick Up First PowerCell
    AutonMove(0, 0, -0.6, false, 0, 55);	// Rotate small angle ccw
    AutonMove(0, 0.5, 0, false, 0, 60);	// 
    while(t < 1200)
    {
        SwerveDriveMe(0.5*backward, -0.1*right, navxGyro->GetYaw()*deg2rad/15, 0, true, 0, 0.5, 0, 0.0);      
        Locomotion(TeleopDrive);
        frc::Wait(0.001);
        t++; 
    }
    TurnToFieldDirection(0);
    AutonMove(0,0,0,true,0,10);

    // Strafe In front of Second PowerCell
    t = 0;
    while(t < 560)
    {
        SwerveDriveMe(0.45*backward, -0.5*left, navxGyro->GetYaw()*deg2rad/15, 0, true, 0, 0.7, 0, 0.08);      
        Locomotion(TeleopDrive);   
        frc::Wait(0.001);
        t++; 
    }
    TurnToFieldDirection(0);

    // Pick up Second PowerCell
    t = 0;
    while(t < 560)
    {
        SwerveDriveMe(0.5*backward, 0, navxGyro->GetYaw()*deg2rad/10, 0, true, 0, 0.5, 0, 0.0);      
        Locomotion(TeleopDrive);   
        frc::Wait(0.001);
        t++; 
    }
    TurnToFieldDirection(0);
    AutonMove(0,0,0,true,0,10);

    // Strafe In front of Third PowerCell
    t = 0;
    while(t < 695)
    {
        SwerveDriveMe(0.051*forward, -0.5*right, navxGyro->GetYaw()*deg2rad/15, 0, true, 0, 0.7, 0, 0.08);      
        Locomotion(TeleopDrive);   
        frc::Wait(0.001);
        t++; 
    }
    TurnToFieldDirection(0);

    // Pick up Third PowerCell
    t = 0;
    while(t < 1000)
    {
        SwerveDriveMe(0.5*backward, 0, navxGyro->GetYaw()*deg2rad/15, 0, true, 0, 0.5, 0, 0.0);      
        Locomotion(TeleopDrive);   
        frc::Wait(0.001);
        t++; 
    }

    SetIntakeMotorsOff();

    // // Sprint to end zone
    t = 0;
    while(t < 2500)
    {
        SwerveDriveMe(0.5*backward, 0, navxGyro->GetYaw()*deg2rad/15, 0, true, 0, 0.5, 0, 0.0);      
        Locomotion(TeleopDrive);   
        frc::Wait(0.001);
        t++; 
    }    

     AutonMove(0,0,0,true,0,10);
}
//===================================================================================================
void GS_A_Red(void)
{
    int t = 0;
    
    SetIntakeMotorsOn();	
    frc::Wait(1);

    // Pick Up First PowerCell
    while(t < 1200)
    {
        SwerveDriveMe(0.5*backward, 0, navxGyro->GetYaw()*deg2rad/15, 0, true, 0, 0.5, 0, 0.0);      
        Locomotion(TeleopDrive);
        frc::Wait(0.001);
        t++; 
    }

    // Strafe In front if Second PowerCell
    t = 0;
    while(t < 580)
    {
        SwerveDriveMe(0.4*backward, -0.5*left, navxGyro->GetYaw()*deg2rad/15, 0, true, 0, 0.7, 0, 0.08);      
        Locomotion(TeleopDrive);   
        frc::Wait(0.001);
        t++; 
    }
    TurnToFieldDirection(0);

    // Pick up Second PowerCell
    t = 0;
    while(t < 550)
    {
        SwerveDriveMe(0.5*backward, 0, navxGyro->GetYaw()*deg2rad/10, 0, true, 0, 0.5, 0, 0.0);      
        Locomotion(TeleopDrive);   
        frc::Wait(0.001);
        t++; 
    }
    TurnToFieldDirection(0);

    // Strafe In front if Third PowerCell
    t = 0;
    while(t < 970)
    {
        SwerveDriveMe(0.1*forward, -0.5*right, navxGyro->GetYaw()*deg2rad/15, 0, true, 0, 0.7, 0, 0.08);      
        Locomotion(TeleopDrive);   
        frc::Wait(0.001);
        t++; 
    }
    TurnToFieldDirection(0);

    // Pick up Third PowerCell
    t = 0;
    while(t < 900)
    {
        SwerveDriveMe(0.5*backward, 0, navxGyro->GetYaw()*deg2rad/15, 0, true, 0, 0.5, 0, 0.0);      
        Locomotion(TeleopDrive);   
        frc::Wait(0.001);
        t++; 
    }

    SetIntakeMotorsOff();

    // // Sprint to end zone
    t = 0;
    while(t < 800)
    {
        SwerveDriveMe(0.5*backward, 0, navxGyro->GetYaw()*deg2rad/15, 0, true, 0, 0.5, 0, 0.0);      
        Locomotion(TeleopDrive);   
        frc::Wait(0.001);
        t++; 
    }    

    AutonMove(0,0,0,true,0,10);
}