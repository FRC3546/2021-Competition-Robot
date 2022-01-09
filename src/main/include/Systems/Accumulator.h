void SetIntakeMotorsOn()
{
	SetPickupMotors(pickupMotorIntake);
    SetPickupArmPosition(pickupDownSolenoidPosition);
    accumulatorMotor1->Set(0.5*accumulatorMotor1Direction);		// vertical power cell motor
    accumulatorMotor2->Set(accumulatorMotor2Direction);			// horizontal power cell motor
}

void SetIntakeMotorsOff()
{
	SetPickupMotors(pickupMotorStop);
	SetPickupArmPosition(pickupUpSolenoidPosition);
	accumulatorMotor1->Set(0);		// vertical power cell motor
	accumulatorMotor2->Set(0);		// horizontal power cell motor 
}