//--------------------------------------------------------------------------
float FindMin(float x, float y)
{
    float retval = y;
    
    if (x < y)
        retval = x;
    
    return retval;
}
//--------------------------------------------------------------------------
float FindMax(float x, float y)
{
    float retval = y;
    
    if (x > y)
        retval = x;
    
    return retval;
}
//--------------------------------------------------------------------------
float FindBound(float val, float minbound, float maxbound)
{
    float retval = val;

    if (val < minbound)
        retval = minbound;
    
    if (val > maxbound)
        retval = maxbound;
    
    return retval;
}
//--------------------------------------------------------------------------
int FindSign(float val)
{
    return abs(val)/val;
}
//--------------------------------------------------------------------------
float ConvertAngle(float Angle)
{
    // Converts any angle from navX Gyro to 0 - 360 degrees
    float retval = Angle;

    while (retval < 0 || retval > 360)
    {
        // For negative angles
        if (retval < 0)
            retval = 360 + retval;

        // For angles more than 360
        if (retval > 360)
            retval = retval -360;
    }

    // Just use 0 if angle is 360
    if (retval == 360)
        retval = 0;

    return retval;
}
//--------------------------------------------------------------------------
int BestRotateDirection(float startAngle, float endAngle)
{
    // Convert to 360 degrees system
    float ang_start = ConvertAngle(startAngle);
    float ang_end = ConvertAngle(endAngle);

    int rotdir = 0;

    // Case 1 : if end angle > start angle
    if (ang_end > ang_start)
    {
        if (ang_end - ang_start < 180)
            rotdir = 1;    // ccw
        else
            rotdir = -1;     // cw
    }

    // Case 2 : if end angle < start angle
    if (ang_end < ang_start)
    {
        if (ang_start - ang_end < 180)
            rotdir = -1;    // ccw
        else
            rotdir = 1;     // cw
    }
    
    return rotdir;
}
//--------------------------------------------------------------------------