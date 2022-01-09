void Barrel_New2(void)
{
    //===========================
    // AutoNav Barrel Race
    
    // Drive straight forward, stop next to first marker
    AutonMove(0,-1,0,false,0,210);

    // Circle around first marker
    //CircleAroundRight(20, 25, 0.7);		//CircleAroundRight(#of polygon sides, loopcount, speed)
    //TurnToFieldDirection(0);
    CircleMove1(30, 0.7);
    CircleMove2(30, 0.7);
    AutonMove(0,0.5,0,false,0,70);
    CircleMove3(40, 0.7);
    CircleMove4(40, 0.7);

    // // Drive straight forward, stop next to second marker
    AutonMove(0.35,-1,0,false,0,195);

    // //Circle around second marker counter-clockwise
    // CircleAroundLeft(20, 50, 0.7);
    // //TurnToFieldDirection(0);
    CircleMove5(30, 0.7);
    CircleMove6(30, 0.7);
    AutonMove(0,0.5,0,false,0,70);
    CircleMove7(40, 0.7);
    CircleMove8(30, 0.7);

    // // Drive forward-right, stop next to third marker
    AutonMove(0.82,-1,0,false,0,190);
    //AutonMove(0,-0.5,0,false,0,30);
    // //TurnToFieldDirection(0);

    // //Circle around third marker counter-clockwise
    // CircleAroundLeft2(20, 55, 0.7);
    CircleMove5(40, 0.7);
    CircleMove6(40, 0.7);

    // // Move Back to starting zone
    AutonMove(0.1,1,0,false,0,610);

    // Stop
    AutonMove(0,0,0,false,0,10);
}

void Barrel_New(void)
{
    //===========================
    // AutoNav Barrel Race
    
    // Drive straight forward, stop next to first marker
    AutonMove(0,-0.7,0,false,0,385);

    // Circle around first marker
    CircleAroundRight(20, 55, 0.7);		//CircleAroundRight(#of polygon sides, loopcount, speed)
    //TurnToFieldDirection(0);

    // Drive straight forward, stop next to second marker
    AutonMove(0.25,-0.7,0,false,0,350);

    //Circle around second marker counter-clockwise
    CircleAroundLeft(20, 50, 0.7);
    //TurnToFieldDirection(0);

    // Drive forward-right, stop next to third marker
    AutonMove(0.7,-0.5,0,false,0,350);
    AutonMove(0,-0.5,0,false,0,30);
    //TurnToFieldDirection(0);

    //Circle around third marker counter-clockwise
    CircleAroundLeft2(20, 55, 0.7);

    // Move Back to starting zone
    AutonMove(0,0.7,0,false,0,850);

    // Stop
    AutonMove(0,0,0,false,0,10);
}


void Barrel_Move1(void)
{
    loopcount = 520;
    AutonMove2(0.7*left, 0, 0, false, 0, loopcount, 1, RightLidarID);
    //AutonMove(0.4*left,0,0,false,0,15);
    //TurnToFieldDirection(0);
    //AutonMove(0,0,0,false,0,2);
    
}

void Barrel_Move2(void)
{
    loopcount = 250;
    AutonMove(0,0.7*forward,0,false,0,loopcount);
    //AutonMove(0,0,0,false,0,2);
    //TurnToFieldDirection(0);
}

void Barrel_Move3(void)
{
    loopcount = 210;
    AutonMove(0.7*right,0,0,false,0,loopcount);
    //TurnToFieldDirection(0);  
    //AutonMove(0,0,0,false,0,2);  
    
}

void Barrel_Move4(void)
{
    loopcount = 260;
    AutonMove(0,0.7*backward,0,false,0,loopcount);
    //TurnToFieldDirection(0);
    //AutonMove(0,0,0,false,0,2);
    
}

void Barrel_Move5(void)
{      
    AutonMove(0.7*left,0.12*forward,0,false,0,570);
    //TurnToFieldDirection(0);
    //AutonMove(0,0,0,false,0,2);
    
}

void Barrel_Move6(void)
{
    loopcount = 245;
    AutonMove(0,0.7*backward,0,false,0,loopcount);
    //TurnToFieldDirection(0);
    //AutonMove(0,0,0,false,0,2);
    
}

void Barrel_Move7(void)
{
    //loopcount = 500;
    //AutonMove2(0.7*right, 0, 0, false, 0, loopcount, 1, LeftLidarID);
    AutonMove(0.7*right,0,0,false,0,230);
    //TurnToFieldDirection(0); 
    //AutonMove(0,0,0,false,0,2);  
    
}

void Barrel_Move8(void)
{
    loopcount = 465;
    AutonMove(0,0.7*forward,0,false,0,loopcount);
    //TurnToFieldDirection(0);
    //AutonMove(0,0,0,false,0,2);    
}

void Barrel_Move9(void)
{
    loopcount = 500;
    //AutonMove2(0.7*left, 0, 0, false, 0, loopcount, 1, LeftLidarID);
    AutonMove(0.7*left,0,0,false,0,465);
    //TurnToFieldDirection(0);
    //AutonMove(0,0,0,false,0,2);
    
}

void Barrel_Move10(void)
{
    loopcount = 385;
    AutonMove(0,0.7*backward,0,false,0,280);
    //AutonMove(0,0,0,false,0,2);
    //TurnToFieldDirection(0);    
}

void Barrel_Move11(void)
{
    loopcount = 1050;
    AutonMove(0.7*right,0,0,false,0,loopcount);
    AutonMove(0,0,0,false,0,2);
    //TurnToFieldDirection(0);    
}