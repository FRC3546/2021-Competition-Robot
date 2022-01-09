// Swerve Drive routine ver 1.3
// 02/05/2020

#include<math.h>
#include<algorithm>

// Swerve Drive Geometry
const static float Length = 25;     // wheel base
const static float Width = 20;      // track width
const static float R = sqrt(Length*Length + Width*Width);

const static int TeleopDrive = 0;
const static int AutonomousDrive = 1;

// Max Motor RPM
//const static float maxRPM = 60.0;
//const static float maxRPM = 2700;     // For manual driving challenges, normal RPM is 2650
const static float maxRPM = 3000.0;     // for Autonomous Challenges


// Conversion factor radians to degrees - 180/pi
const static float rad2deg = 57.2957795;
const static float deg2rad = 0.01745329251;

float FaceAngle = 0;
float wheelSpeed[4] = {0};
float wheelPreviousAngle[4] = {0};
float wheelAngle[4] = {0};
float throttle[4] = {0};
float steer[4] = {0};
float inputRotatePrevious, inputForwardPrevious, inputStrafePrevious;
float gainP, gainI, gainD, dt;

float bodyError = 0;
float bodyPreviousError = 0;
float bodyIntError = 0;
float bodyDiffError = 0;
float BodyAngle;

void OptimizeSteerTurning3(void)
{
    float targetAngle,targetStraightAngle, currentAngle, A, B, b0, x0, x1;;
    int t = 0;
    
    while (t < 4)
    {
        currentAngle = wheelPreviousAngle[t];
        targetAngle = wheelAngle[t];

        x0 = abs(currentAngle);
        x1 = abs(targetAngle);

        // Calculate Straight/Opposite Target Angle
        if(targetAngle > 0)
        {
            targetStraightAngle = targetAngle - 180;
        }
        else
        {
            targetStraightAngle = 180 + targetAngle;
        }

        // Calculate A & B (refer to Excel spreadsheet)
        A = abs(currentAngle - targetAngle);
        b0 = abs(currentAngle - targetStraightAngle);
        if (b0 > 180)
        {
            B = 360 - b0;
        }
        else
        {
            B = b0;
        }

        // Pick which angle position
        if (A <= B)
        {            
            wheelAngle[t] = targetAngle;
        }
        else
        {
            wheelAngle[t] = targetStraightAngle;   
            throttle[t] = -throttle[t];
        }      

        t++;
    }   
}

void ZeroSwerveDrive2(void)
{
    int loopcount = 0;
	int a, b, c, d, e;
    float rotationSpeed[]={0.4,0.4,0.3,0.4};
	
	navxGyro->ZeroYaw(); 

    while (loopcount < 1)
    {
		
        a = 0;
        b = 0;
        c = 0;
        d = 0;
        e = 0;

        steerMotor0.Set(ControlMode::PercentOutput, rotationSpeed[0]);      // front right
		steerMotor1.Set(ControlMode::PercentOutput, rotationSpeed[1]);      // front left
		steerMotor2.Set(ControlMode::PercentOutput, rotationSpeed[2]);      // rear left
		steerMotor3.Set(ControlMode::PercentOutput, rotationSpeed[3]);      // rear right
				
		while (e < 4)
		{
			// frc::SmartDashboard::PutBoolean("Front Right IRSensor", IRSensor2->Get());
            // frc::SmartDashboard::PutBoolean("Front Left IRSensor", IRSensor3->Get());
            // frc::SmartDashboard::PutBoolean("Rear Left IRSensor", IRSensor0->Get());
            // frc::SmartDashboard::PutBoolean("Rear Right IRSensor", IRSensor1->Get());
            
            if(IRSensor0->Get() == IRSensorClosed)
			{
				steerMotor0.Set(ControlMode::PercentOutput, 0);					
                if (a == 0)
                    {a = 1;rotationSpeed[0] = -rotationSpeed[0]*0.8;}			
			}
			
			if(IRSensor1->Get() == IRSensorClosed)
			{
				steerMotor1.Set(ControlMode::PercentOutput, 0);			
                if (b == 0)
                    {b = 1;rotationSpeed[1] = -rotationSpeed[1]*0.8;}				
			}
			
			if(IRSensor2->Get() == IRSensorClosed)
			{
				steerMotor2.Set(ControlMode::PercentOutput, 0);				
                if (c == 0)
                    {c = 1;rotationSpeed[2] = -rotationSpeed[2]*0.8;}				
			}
			
			
			if(IRSensor3->Get() == IRSensorClosed)
			{
				steerMotor3.Set(ControlMode::PercentOutput, 0);				
                if (d == 0)
                    {d = 1;rotationSpeed[3] = -rotationSpeed[3]*0.8;}				
			}	

            e = a + b + c + d;	
		}       

        loopcount++;
    } 
       
    FaceAngle = navxGyro->GetYaw()*deg2rad;
    SetSteerMotorInitialPosition();   
}

void ZeroWheelsOnly(void)
{
    int loopcount;
    int loopcount2;
    int loopcount3;
	int t;
    int a, b, c, d, e;
    float rotationSpeed[]={0.4,0.4,0.4,0.4};            // first zero-ing loop wheel speed
    float rotationSpeed2[]={-0.35,-0.35,-0.35,-0.35};      // second zero-ing loop wheel speed
    float rotationSpeed3[]={0.25,0.25,0.25,0.25};      // third zero-ing loop wheel speed

    int adjustAttemptMax = 10000;

    // Set these values to 1 if you do not want to run the wheel zero-ing loop
    loopcount = 0;
    loopcount2 = 0;
    loopcount3 = 0;

    // First loop - spin wheels one way until sensor is tripped
    while (loopcount < 1)
    {
		
        a = 0;
        b = 0;
        c = 0;
        d = 0;
        e = 0;
        t = 0;

        steerMotor0.Set(ControlMode::PercentOutput, rotationSpeed[0]);      // front right
		steerMotor1.Set(ControlMode::PercentOutput, rotationSpeed[1]);      // front left
		steerMotor2.Set(ControlMode::PercentOutput, rotationSpeed[2]);      // rear left
		steerMotor3.Set(ControlMode::PercentOutput, rotationSpeed[3]);      // rear right
				
		while (e < 4)
		{

            // steerMotor0.Set(ControlMode::PercentOutput, rotationSpeed[0]*(1-t/adjustAttemptMax));      // front right
		    // steerMotor1.Set(ControlMode::PercentOutput, rotationSpeed[1]*(1-t/adjustAttemptMax));      // front left
		    // steerMotor2.Set(ControlMode::PercentOutput, rotationSpeed[2]*(1-t/adjustAttemptMax));      // rear left
		    // steerMotor3.Set(ControlMode::PercentOutput, rotationSpeed[3]*(1-t/adjustAttemptMax));      // rear right
            
            if(IRSensor0->Get() == IRSensorClosed)
			{
				steerMotor0.Set(ControlMode::PercentOutput, 0);					
                if (a == 0)
                    {a = 1;}// rotationSpeed[0] = -rotationSpeed[0]*0.8;}			
			}
			
			if(IRSensor1->Get() == IRSensorClosed)
			{
				steerMotor1.Set(ControlMode::PercentOutput, 0);			
                if (b == 0)
                    {b = 1;}//rotationSpeed[1] = -rotationSpeed[1]*0.8;}				
			}
			
			if(IRSensor2->Get() == IRSensorClosed)
			{
				steerMotor2.Set(ControlMode::PercentOutput, 0);				
                if (c == 0)
                    {c = 1;}//rotationSpeed[2] = -rotationSpeed[2]*0.8;}				
			}
			
			
			if(IRSensor3->Get() == IRSensorClosed)
			{
				steerMotor3.Set(ControlMode::PercentOutput, 0);				
                if (d == 0)
                    {d = 1;}//rotationSpeed[3] = -rotationSpeed[3]*0.8;}				
			}	

            e = a + b + c + d;	
            t++;
            //frc::Wait(0.0001);
		}       

        loopcount++;
    }

    //Second loop - spin wheels the other way until sensor is tripped again to reduce any overshoots from the first loop
    while (loopcount2 < 1)
    {
		
        a = 0;
        b = 0;
        c = 0;
        d = 0;
        e = 0;
        t = 0;  

        steerMotor0.Set(ControlMode::PercentOutput, rotationSpeed2[0]);      // front right
		steerMotor1.Set(ControlMode::PercentOutput, rotationSpeed2[1]);      // front left
		steerMotor2.Set(ControlMode::PercentOutput, rotationSpeed2[2]);      // rear left
		steerMotor3.Set(ControlMode::PercentOutput, rotationSpeed2[3]);      // rear right      
				
		while (e < 4)
		{
			if(IRSensor0->Get() == IRSensorClosed)
			{
				steerMotor0.Set(ControlMode::PercentOutput, 0);					
                if (a == 0)
                    {a = 1;}			
			}
			
			if(IRSensor1->Get() == IRSensorClosed)
			{
				steerMotor1.Set(ControlMode::PercentOutput, 0);			
                if (b == 0)
                    {b = 1;}				
			}
			
			if(IRSensor2->Get() == IRSensorClosed)
			{
				steerMotor2.Set(ControlMode::PercentOutput, 0);				
                if (c == 0)
                    {c = 1;}				
			}
			
			
			if(IRSensor3->Get() == IRSensorClosed)
			{
				steerMotor3.Set(ControlMode::PercentOutput, 0);				
                if (d == 0)
                    {d = 1;}				
			}	

            e = a + b + c + d;	
            t++;
            frc::Wait(0.0001);
		}       

        loopcount2++;
    }

    //Third loop - spin wheels the other way until sensor is tripped again to reduce any overshoots from the first loop
    while (loopcount3 < 1)
    {
		
        a = 0;
        b = 0;
        c = 0;
        d = 0;
        e = 0;
        t = 0;  

        steerMotor0.Set(ControlMode::PercentOutput, rotationSpeed3[0]);      // front right
		steerMotor1.Set(ControlMode::PercentOutput, rotationSpeed3[1]);      // front left
		steerMotor2.Set(ControlMode::PercentOutput, rotationSpeed3[2]);      // rear left
		steerMotor3.Set(ControlMode::PercentOutput, rotationSpeed3[3]);      // rear right      
				
		while (e < 4)
		{
			if(IRSensor0->Get() == IRSensorClosed)
			{
				steerMotor0.Set(ControlMode::PercentOutput, 0);					
                if (a == 0)
                    {a = 1;}			
			}
			
			if(IRSensor1->Get() == IRSensorClosed)
			{
				steerMotor1.Set(ControlMode::PercentOutput, 0);			
                if (b == 0)
                    {b = 1;}				
			}
			
			if(IRSensor2->Get() == IRSensorClosed)
			{
				steerMotor2.Set(ControlMode::PercentOutput, 0);				
                if (c == 0)
                    {c = 1;}				
			}
			
			
			if(IRSensor3->Get() == IRSensorClosed)
			{
				steerMotor3.Set(ControlMode::PercentOutput, 0);				
                if (d == 0)
                    {d = 1;}				
			}	

            e = a + b + c + d;	
            t++;
            frc::Wait(0.0001);
		}       

        loopcount3++;
    }
    
    // Record Encoder Reading for Zero-ed Wheel Positions
    SetSteerMotorInitialPosition();   
}

void SwerveDriveMe(float Forward, float Strafe, float Rotate, float BodyAngle, bool isSetFaceAngle, float setFaceAngle, float Pgain, float Igain, float Dgain)
{
    int t;
    float tempval, errorAngle, magnitude; 

    // Store Input Values from Controller
    float inputForward = Forward;
    float inputStrafe = Strafe;
    float inputRotate = Rotate;

    // Record previous steer wheel angles  
    t = 0;
    while (t < 4)
    {
        wheelPreviousAngle[t] = wheelAngle[t];
        t++;
    }    
 
    //-------------------------------------------------------------------------------
    // Check if translation and rotation is above the threshold to start moving (5%)
    //-------------------------------------------------------------------------------
    if (abs(Forward) > 0.05)
    {
        Forward = (abs(Forward)-0.05)*Forward/abs(Forward);
    }
    else
    {
        Forward = 0;
    }
    //--------------------------------
    if (abs(Strafe) > 0.05)
    {
        Strafe = (abs(Strafe)-0.05)*Strafe/abs(Strafe);
    }
    else
    {
        Strafe = 0;
    }  
    //--------------------------------
    magnitude = sqrt(Forward*Forward + Strafe*Strafe); 
    //--------------------------------
    if (abs(Rotate) > 0.05)
    {
        Rotate = (abs(Rotate)-0.05)*Rotate/abs(Rotate);
    }
    else
    {   
        // This section is for when there are no input into the rotation controller
        Rotate = 0;

        // if rotation controller was just released, record Face Angle
        if (abs(inputRotatePrevious) > 0.05 && isSetFaceAngle == false)    // previously, there was input into the rotation controller
        {
            FaceAngle = navxGyro->GetYaw();
        }

        // If we are purely translating, we need to insert minute corrections to rotate
        // the body to make sure Robot faces consistent direction as it translates
        if (magnitude > 0)
        {
            //Rotate = Rotate + (BodyAngle-FaceAngle)*deg2rad*gainP;

            bodyPreviousError = bodyError;
			bodyError = (BodyAngle - FaceAngle)*deg2rad;
			bodyIntError = bodyIntError + bodyError;
			bodyDiffError = bodyError - bodyPreviousError;
			Rotate = Rotate + bodyError*Pgain + bodyIntError*Igain + bodyDiffError*Dgain;
        }        
    }    
    //-------------------------------------------------------------------------------

    BodyAngle = BodyAngle*deg2rad;
    float Vx = Forward*cos(BodyAngle) + Strafe*sin(BodyAngle);
    float Vy = -Forward*sin(BodyAngle) + Strafe*cos(BodyAngle);    

    float Rx = -Rotate*Width/R;
    float Ry = -Rotate*Length/R;    

    float V0x = Vx + Rx;
    float V1x = Vx - Rx;
    float V2x = Vx - Rx;
    float V3x = Vx + Rx;

    float V0y = -Vy + Ry;
    float V1y = -Vy + Ry;
    float V2y = -Vy - Ry;
    float V3y = -Vy - Ry;

    // Calculate Wheel Speeds
    wheelSpeed[0] = sqrt(V0x*V0x + V0y*V0y);      // front right corner
    wheelSpeed[1] = sqrt(V1x*V1x + V1y*V1y);      // front left corner
    wheelSpeed[2] = sqrt(V2x*V2x + V2y*V2y);      // rear left corner
    wheelSpeed[3] = sqrt(V3x*V3x + V3y*V3y);      // rear right corner

    // Normalize Wheel Speeds if maximum wheelSpeed > 1
    tempval = 0;
    for(t=0;t<4;t++)
    {
        if (wheelSpeed[t] > tempval)
        {
            tempval = wheelSpeed[t];
        }
    }

    if (tempval > 1)
    {
        for (t=0;t<4;t++)
        {
            wheelSpeed[t] = wheelSpeed[t]/tempval;
        }
    }

    // Convert to throttle RPM
    for(t=0;t<4;t++)
    {
        throttle[t] = wheelSpeed[t]*maxRPM;
    }

    //if (magnitude >= 0.15 || abs(Rotate) >= 0.15)  // do not adjust the steering wheel angles if we are letting go of the joystick 
    //{
        // Calculate Wheel Angles
        wheelAngle[0] = -atan2(V0y, V0x)*rad2deg;   // front right corner
        wheelAngle[1] = -atan2(V1y, V1x)*rad2deg;   // front left corner
        wheelAngle[2] = -atan2(V2y, V2x)*rad2deg;   // rear left corner
        wheelAngle[3] = -atan2(V3y, V3x)*rad2deg;   // rear right corner 
    //}

    OptimizeSteerTurning3();

    // Store Input from Controllers for use in the next loop
    inputForwardPrevious = inputForward;
    inputRotatePrevious = inputRotate;
    inputStrafePrevious = inputStrafe;
}
//==========================================================================================================================
void Locomotion(int Mode)
{    
	int t;

    // How hard robot 'holds on' to steering angle
    // set to 0.0005 on smart dashboard for regular driving
    // set to 0.0015 on smart dashboard for autonomous driving*
    // set to 0 to control steering angle via encoder position    
    //float steering_hold = frc::SmartDashboard::GetNumber("Steering_Hold", 0.0005);
    float steering_hold;

    if (Mode == TeleopDrive)
    {
        steering_hold = 0.0005;     // this value is found from experimenting
    }
    else
    {
        steering_hold = 0.00085;    // this value is found from experimenting
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    t = 0;
    while (t < 4)
    {
      if (wheelAngle[t] > 0 && wheelPreviousAngle[t] < 0  && abs(wheelAngle[t]) > 90)
      {
        wheelAngle[t] = wheelAngle[t] - 360;      
      }
      else if (wheelAngle[t] < 0 && wheelPreviousAngle[t] > 0  && abs(wheelAngle[t]) > 90)
      {
        wheelAngle[t] = 360 + wheelAngle[t];      
      }
      t++;
    }

    // Get Current Steering Motor Encoder Position
    steerMotorCurrentPosition[0] = steerMotor0.GetSelectedSensorPosition(0);
    steerMotorCurrentPosition[1] = steerMotor1.GetSelectedSensorPosition(0);
    steerMotorCurrentPosition[2] = steerMotor2.GetSelectedSensorPosition(0);
    steerMotorCurrentPosition[3] = steerMotor3.GetSelectedSensorPosition(0);

    if (steering_hold > 0)      // Control Steering Motor by how 'hard' to hold on to wheel angle
    {
        steerMotor0.Set(ControlMode::PercentOutput, steerMotorDirection[0]*((steerMotorCurrentPosition[0] - steerMotorInitialPosition[0])-(int)(wheelAngle[0]*18.7277777778))*steering_hold);
        steerMotor1.Set(ControlMode::PercentOutput, steerMotorDirection[1]*((steerMotorCurrentPosition[1] - steerMotorInitialPosition[1])-(int)(wheelAngle[1]*18.7277777778))*steering_hold);
        steerMotor2.Set(ControlMode::PercentOutput, steerMotorDirection[2]*((steerMotorCurrentPosition[2] - steerMotorInitialPosition[2])-(int)(wheelAngle[2]*18.7277777778))*steering_hold);
        steerMotor3.Set(ControlMode::PercentOutput, steerMotorDirection[3]*((steerMotorCurrentPosition[3] - steerMotorInitialPosition[3])-(int)(wheelAngle[3]*18.7277777778))*steering_hold);
    }
    else        // Control Steering Angle by servo-ing to wheel angle encoder position
    {
        steerMotor0.Set(ControlMode::Position, steerMotorDirection[0]*((steerMotorCurrentPosition[0] - steerMotorInitialPosition[0])-(int)(wheelAngle[0]*18.7277777778)));
        steerMotor1.Set(ControlMode::Position, steerMotorDirection[1]*((steerMotorCurrentPosition[1] - steerMotorInitialPosition[1])-(int)(wheelAngle[1]*18.7277777778)));
        steerMotor2.Set(ControlMode::Position, steerMotorDirection[2]*((steerMotorCurrentPosition[2] - steerMotorInitialPosition[2])-(int)(wheelAngle[2]*18.7277777778)));
        steerMotor3.Set(ControlMode::Position, steerMotorDirection[3]*((steerMotorCurrentPosition[3] - steerMotorInitialPosition[3])-(int)(wheelAngle[3]*18.7277777778)));
    }    

    t = 0;
    while (t < 4)
    {
       if (abs(wheelAngle[t]) > 180)
      {
        if (wheelAngle[t] < 0)
        {
          wheelAngle[t] = 360 + wheelAngle[t];
          steerMotorInitialPosition[t] = steerMotorInitialPosition[t] - 360*18.7277777778;
        }
        else if (wheelAngle[t] > 0)
        {
          wheelAngle[t] = wheelAngle[t] - 360;
          steerMotorInitialPosition[t] = steerMotorInitialPosition[t] + 360*18.7277777778;
        }
      } 
      t++;
    }

    //---------------------------------------------------------------------------------------------------------------------
    // Set Throttle Motors (Spark Maxes & Neos)
	
	if (abs(throttle[0]) > 0) // Front Right Corner
	{
		//throttleMotor0PIDController.SetReference(throttle[0]*maxRPM*throttleMotorDirection[0], rev::ControlType::kVelocity); 
        throttleMotor0PIDController.SetReference(throttle[0]*throttleMotorDirection[0], rev::ControlType::kVelocity); 
	}
	else
	{
		
		throttleMotor0PIDController.SetReference(0, rev::ControlType::kVoltage);
	}
	
	if (abs(throttle[1]) > 0) // Front Left Corner
	{
		//throttleMotor1PIDController.SetReference(throttle[1]*maxRPM*throttleMotorDirection[1], rev::ControlType::kVelocity);
        throttleMotor1PIDController.SetReference(throttle[1]*throttleMotorDirection[1], rev::ControlType::kVelocity); 
	}
	else
	{
		throttleMotor1PIDController.SetReference(0, rev::ControlType::kVoltage);
	}
	
	if (abs(throttle[2]) > 0) // Rear Left Corner
	{
		//throttleMotor2PIDController.SetReference(throttle[2]*maxRPM*throttleMotorDirection[2], rev::ControlType::kVelocity);
        throttleMotor2PIDController.SetReference(throttle[2]*throttleMotorDirection[2], rev::ControlType::kVelocity);
	}
	else
	{
		throttleMotor2PIDController.SetReference(0, rev::ControlType::kVoltage);
	}
	
	if (abs(throttle[3]) > 0) // Rear Left Corner
	{
		//throttleMotor3PIDController.SetReference(throttle[3]*maxRPM*throttleMotorDirection[3], rev::ControlType::kVelocity);
        throttleMotor3PIDController.SetReference(throttle[3]*throttleMotorDirection[3], rev::ControlType::kVelocity);
	}
	else
	{
		throttleMotor3PIDController.SetReference(0, rev::ControlType::kVoltage);
	}   
	
}