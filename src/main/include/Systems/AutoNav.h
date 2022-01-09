// CIRCLING THE POWERCELL TEST
//=====================================================================================
void PowerCellAvoidLeft(void)
{
    
    bool frontPosition = false;
    bool leftPosition = false;
    float forward, strafe;

    // Set steering motors to brake mode to stop coasting/drifting
    steerMotor0.SetNeutralMode(NeutralMode::Brake);
    steerMotor1.SetNeutralMode(NeutralMode::Brake);
    steerMotor2.SetNeutralMode(NeutralMode::Brake);
    steerMotor3.SetNeutralMode(NeutralMode::Brake);

    // Set throttle motors to brake mode to stop coasting/drifting
    throttleMotor0.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    throttleMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    throttleMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    throttleMotor3.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    
    pc_area = frc::SmartDashboard::GetNumber("area", 1000);

    // Move forward until area is XX
    while (frontPosition == false)
    {
        if (pc_area < 45000)
        {
            forward = -0.4;
        }
        else
        {
            forward = (pc_area - 67000)/(67000 - 45000)*0.4;
        }
        
        AutonMove(0, forward, 0, true, 0, 10);
        pc_area = frc::SmartDashboard::GetNumber("area", 1000);

        if (pc_area > 66000 && pc_area < 67000)
        {
            frontPosition = true;
            AutonMove(0, 0, 0, true, 0, 10);
            frc::Wait(0.5);
        }
    }

    pc_err = frc::SmartDashboard::GetNumber("cc", 1000);

    // Move left until area is XX
    while (leftPosition == false)
    {
        if (strafe < 0)
        {
            forward = -0.4;
        }
        else
        {
            strafe = (pc_err - 67000)/(67000 - 45000)*0.4;
        }
        
        AutonMove(0, forward, 0, true, 0, 10);
        pc_err = frc::SmartDashboard::GetNumber("cc", 1000);

        if (pc_err > 0 && pc_err < 0)
        {
            leftPosition = true;
            AutonMove(0, 0, 0, true, 0, 10);
            frc::Wait(0.5);
        }
    }

    // Set steering motors to back to coast
    steerMotor0.SetNeutralMode(NeutralMode::Coast);
    steerMotor1.SetNeutralMode(NeutralMode::Coast);
    steerMotor2.SetNeutralMode(NeutralMode::Coast);
    steerMotor3.SetNeutralMode(NeutralMode::Coast);

    // Set throttle motors back to coast
    throttleMotor0.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    throttleMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    throttleMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    throttleMotor3.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    frc::Wait(0.5);
}

