void Slalom_New2(void)
{
    AutonMove(0,-0.6,0,false,0,10);
    CircleMove1(20, 0.6);
    CircleMove8(20, 0.6);
    AutonMove(0,-0.6,0,false,0,15);
    CircleMove5(13, 0.6);
    AutonMove(-0.6,0,0,false,0,25);
    CircleMove6(13, 0.6);
    AutonMove(0,0.6,0,false,0,15);
    CircleMove7(20, 0.6);
    CircleMove2(15, 0.6);
    AutonMove(0,0.6,0,false,0,10);
}

void Slalom_New(void)
{
    // Launch
    // AutonMove(0,-0.7,0,false,0,50);
	// CircleMove5(30, 0.7);
	// CircleMove4(25, 0.7);

    // // Forward Section - Go Full Speed Here?
    // AutonMove(0.1,-0.8,0,false,0,360);

    // Circle Around
    AutonMove(0,-0.7,0,false,0,10);
    CircleMove1(40, 0.7);
    CircleMove8(40, 0.7);
    AutonMove(0,-0.7,0,false,0,20);
    CircleMove5(25, 0.7);
    AutonMove(-0.7,0,0,false,0,50);
    CircleMove6(25, 0.7);
    AutonMove(0,0.7,0,false,0,20);
    CircleMove7(40, 0.7);
    CircleMove2(30, 0.7);
    AutonMove(0,0.7,0,false,0,15);

    // Backward Section - Go Full Speed Here?
    // AutonMove(0,0.8,0,false,0,345);

    // // End Zone & Stop
    // AutonMove(0,0.7,0,false,0,60);
	// CircleMove3(30, 0.7);
	// CircleMove6(25, 0.7);
    AutonMove(0,0,0,false,0,5);
}

void Slalom_Move0(void)
{
   // loopcount = frc::SmartDashboard::GetNumber("slalom1", 220);
    AutonMove(0,-0.7,0,false,0,160);
    //AutonMove(0,0,0,false,0,2);
    //TurnToFieldDirection(90);
}

void Slalom_Move1(void)
{
    loopcount = frc::SmartDashboard::GetNumber("slalom1", 330);
    
    //AutonMove2(-0.5, 0, 0, false, 0, loopcount, 1, RightLidarID);
    //AutonMove(-0.5,0,0,false,0,35);
    AutonMove(-0.7,0,0,false,0,175);
    //TurnToFieldDirection(0);
    //AutonMove(0,0,0,false,0,2);
    
}

void Slalom_Move2(void)
{
    loopcount = frc::SmartDashboard::GetNumber("slalom2", 1500);
    //AutonMove2(0, -0.7, 0, false, 0, loopcount, 2, RightLidarID);
    AutonMove(0,-0.8,0,false,0,590);
    //TurnToFieldDirection(0);
    //AutonMove(0,0,0,false,0,2);
}

void Slalom_Move3(void)
{
    loopcount = frc::SmartDashboard::GetNumber("slalom3", 440);
    AutonMove(0.5,0,0,false,0,380);
    //AutonMove(0,0,0,false,0,20);  
    //TurnToFieldDirection(0);  
    //AutonMove(0,0,0,false,0,2);  
}

void Slalom_Move4(void)
{
    //loopcount = frc::SmartDashboard::GetNumber("slalom4", 400);
    //AutonMove2(0, -0.7, 0, false, 0, 300, 1, LeftLidarID);
    AutonMove(0,-0.7,0,false,0,220);
    
    //TurnToFieldDirection(0); 
    //AutonMove(0,0,0,false,0,2);
}

void Slalom_Move5(void)
{
    loopcount = frc::SmartDashboard::GetNumber("slalom5", 360);
    AutonMove(-0.5,0,0,false,0,315);
    //TurnToFieldDirection(0);
    //AutonMove(0,0,0,false,0,2);
    
}

void Slalom_Move6(void)
{
    loopcount = frc::SmartDashboard::GetNumber("slalom6", 400);
    //AutonMove2(0, 0.7, 0, false, 0, loopcount, 1, LeftLidarID);
    AutonMove(0,0.7,0,false,0,240);
    //TurnToFieldDirection(0);
    //AutonMove(0,0,0,false,0,2);
    
}

void Slalom_Move7(void)
{
    //loopcount = frc::SmartDashboard::GetNumber("slalom7", 360);
    loopcount = 325;
    AutonMove(0.5,0,0,false,0,loopcount);
    //TurnToFieldDirection(0);
    //AutonMove(0,0,0,false,0,2);
}

void Slalom_Move8(void)
{
    loopcount = frc::SmartDashboard::GetNumber("slalom8", 1500);
    //AutonMove(0,-0.5,0,false,0,1000);
    //AutonMove2(0, 0.7, 0, false, 0, loopcount, 2, RightLidarID);
    AutonMove(0,0.8,0,false,0,578);
    //TurnToFieldDirection(0);
    //AutonMove(0,0,0,false,0,2);
    
}

void Slalom_Move9(void)
{
    loopcount = frc::SmartDashboard::GetNumber("slalom9", 370);
    AutonMove(-0.7,0,0,false,0,280);
    //AutonMove(0,0,0,false,0,2);
}

void Slalom_Move10(void)
{
    loopcount = frc::SmartDashboard::GetNumber("slalom10", 200);
    loopcount = 150;
    AutonMove(0,0.7,0,false,0,loopcount);
    AutonMove(0,0,0,false,0,20);    
}