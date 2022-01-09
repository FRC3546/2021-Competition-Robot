void Bounce_New(void)
{
    // Launch
    AutonMove(0.50,0,0,false,0,70);
    CircleMove8(35,0.5);
    
    // Move Straight to Hit Marker 1
    AutonMove(0,-0.5,0,false,0,170);
    AutonMove(0,0.5,0,false,0,170);

    // Move Back
    CircleMove2(55, 0.5);
    AutonMove(0,0.50,0,false,0,80);
    
    // Move Back
    CircleMove7(55, 0.5);
    CircleMove8(55, 0.5);
    AutonMove(0,-0.50,0,false,0,475);   //hit second marker
    AutonMove(0,0.50,0,false,0,420);

    CircleMove7(55, 0.5);
    AutonMove(0.50,0,0,false,0,85);
    CircleMove8(55, 0.5);
    AutonMove(0,-0.50,0,false,0,550);   // hit third marker

    AutonMove(0,0.50,0,false,0,240);   // line up infront of end zone
    AutonMove(0.50,0,0,false,0,150);
    //CircleMove7(55, 0.5); //Get into end zone

    // Stop
    AutonMove(0,0,0,false,0,5000);

    while(true);
}

void Bounce_Move1(void)
{
    loopcount = 165;
    //AutonMove2(0.7*left, 0, 0, false, 0, loopcount, 1, RightLidarID);
    AutonMove(0.7*left,0,0,false,0,loopcount);
    TurnToFieldDirection(0);
    AutonMove(0,0,0,false,0,2);
    
}

void Bounce_Move2(void)
{
    loopcount = 150;
    //AutonMove2(0.7*left, 0, 0, false, 0, loopcount, 1, RightLidarID);
    AutonMove(0,0.7*backward,0,false,0,loopcount);
    TurnToFieldDirection(0);
    AutonMove(0,0,0,false,0,2);
    
}

void Bounce_Move3(void)
{
    loopcount = 160;
    //AutonMove2(0.7*left, 0, 0, false, 0, loopcount, 1, RightLidarID);
    AutonMove(0,0.7*forward,0,false,0,loopcount);
    TurnToFieldDirection(0);
    AutonMove(0,0,0,false,0,2);
    
}

void Bounce_Move4(void)
{
    loopcount = 95;
    //AutonMove2(0.7*left, 0, 0, false, 0, loopcount, 1, RightLidarID);
    AutonMove(0.7*left,0,0,false,0,loopcount);
    TurnToFieldDirection(0);
    AutonMove(0,0,0,false,0,2);
    
}

void Bounce_Move5(void)
{
    loopcount = 185;
    //AutonMove2(0.7*left, 0, 0, false, 0, loopcount, 1, RightLidarID);
    AutonMove(0,0.7*forward,0,false,0,loopcount);
    TurnToFieldDirection(0);
    AutonMove(0,0,0,false,0,20);
    
}

void Bounce_Move6(void)
{
    loopcount = 210;
    //AutonMove2(0.7*left, 0, 0, false, 0, loopcount, 1, RightLidarID);
    AutonMove(0.7*left,0,0,false,0,loopcount);
    TurnToFieldDirection(0);
    AutonMove(0,0,0,false,0,2);
    
}

void Bounce_Move7(void)
{
    loopcount = 350;
    //AutonMove2(0.7*left, 0, 0, false, 0, loopcount, 1, RightLidarID);
    AutonMove(0,0.7*backward,0,false,0,loopcount);
    TurnToFieldDirection(0);
    AutonMove(0,0,0,false,0,2);
    
}