#include <frc/DigitalInput.h>

//==========================================================================
// LIDAR DEFINITION ON DIO PINS
//--------------------------------------------------------------------------
// When Lidar sensor detects obstacles, it will be closed (true, High)
// When Lidar sensor does not detect obstacles, it will be open (false, Low)

const static bool LeftLidarOpen = false;
const static bool LeftLidarClosed = true;

const static bool RightLidarOpen = false;
const static bool RightLidarClosed = true;

const static int LeftLidarDIOPin = 8;
const static int RightLidarDIOPin = 9;

const static int LeftLidarID = 0;
const static int RightLidarID = 1;

frc::DigitalInput *LeftLidar;
frc::DigitalInput *RightLidar;

// LIDAR - Benewake TF-Luna on Arduino Uno
const static int arduino_I2C_address = 8;
uint8_t arduino_data[4];
int LeftLidar_Distance = 0;
int RightLidar_Distance = 0;

frc::I2C *arduino;
//==========================================================================

void SetupLidarSensors(void)
{
    LeftLidar = new frc::DigitalInput(LeftLidarDIOPin);   
    frc::SmartDashboard::PutBoolean("Left Lidar", LeftLidar->Get());

    RightLidar = new frc::DigitalInput(RightLidarDIOPin);
    frc::SmartDashboard::PutBoolean("Right Lidar", RightLidar->Get());  

    arduino = new frc::I2C(frc::I2C::Port::kOnboard, arduino_I2C_address);	// for continuous distance value 
}

bool CheckLeftLidar(void)
{
    return !LeftLidar->Get();
}

bool CheckRightLidar(void)
{
    return !RightLidar->Get();
}

int Lidar_UpdateDistance(void)
{
    arduino->ReadOnly(4, arduino_data);

    LeftLidar_Distance = arduino_data[0] + arduino_data[1]*256;
    RightLidar_Distance = arduino_data[2] + arduino_data[3]*256;
}

