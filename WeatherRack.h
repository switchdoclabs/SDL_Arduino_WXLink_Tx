boolean fuzzyCompare(float compareValue, float value)
{
#define VARYVALUE 0.05

  if ( (value > (compareValue * (1.0 - VARYVALUE)))  && (value < (compareValue * (1.0 + VARYVALUE))) )
  {

    return true;

  }

  return false;



}

float voltageToDegrees(float value, float defaultWindDirection)
{

  // Note:  The original documentation for the wind vane says 16 positions.  Typically only recieve 8 positions.  And 315 degrees was wrong.

  // For 5V, use 1.0.  For 3.3V use 0.66
#define ADJUST3OR5 1.00
#define PowerVoltage 5.0

  if (fuzzyCompare(3.84 * ADJUST3OR5 , value))
    return 0.0;

  if (fuzzyCompare(1.98 * ADJUST3OR5, value))
    return 22.5;

  if (fuzzyCompare(2.25 * ADJUST3OR5, value))
    return 45;

  if (fuzzyCompare(0.41 * ADJUST3OR5, value))
    return 67.5;

  if (fuzzyCompare(0.45 * ADJUST3OR5, value))
    return 90.0;

  if (fuzzyCompare(0.32 * ADJUST3OR5, value))
    return 112.5;

  if (fuzzyCompare(0.90 * ADJUST3OR5, value))
    return 135.0;

  if (fuzzyCompare(0.62 * ADJUST3OR5, value))
    return 157.5;

  if (fuzzyCompare(1.40 * ADJUST3OR5, value))
    return 180;

  if (fuzzyCompare(1.19 * ADJUST3OR5, value))
    return 202.5;

  if (fuzzyCompare(3.08 * ADJUST3OR5, value))
    return 225;

  if (fuzzyCompare(2.93 * ADJUST3OR5, value))
    return 247.5;

  if (fuzzyCompare(4.62 * ADJUST3OR5, value))
    return 270.0;

  if (fuzzyCompare(4.04 * ADJUST3OR5, value))
    return 292.5;

  if (fuzzyCompare(4.34 * ADJUST3OR5, value))  // chart in documentation wrong
    return 315.0;

  if (fuzzyCompare(3.43 * ADJUST3OR5, value))
    return 337.5;

  //Serial.print(" FAIL WIND DIRECTION");
  return defaultWindDirection;  // return previous value if not found


}


void serviceInterruptRain()
{


  unsigned long currentTime = (unsigned long) (micros() - lastRainTime);

  // lastRainTime = micros();
  // if (currentTime > 500) // debounce
  // {
  TotalRainClicks++;
  wakeState = RAIN_INTERRUPT;
  //      interrupt_count[19]++;
  //   if(currentTime<currentRainMin)
  //   {
  //    currentRainMin=currentTime;
  //  }
  return;
  //}
  //wakeState = IGNORE_INTERRUPT;

}



void serviceInterruptAnem()
{
   wakeState = ANEMOMETER_INTERRUPT;
  // software debounce for 17 msec (limits us to 90 MPH)!

if (ignore_anemometer_interrupt== true)
{
  return;
}

  


  unsigned long currentTime = (unsigned long)(micros() - lastWindTime);

  if (currentTime < shortestWindTime)
  {
    shortestWindTime = currentTime;
  }


  windClicks++;


  /*
    if (currentTime > 500) // debounce
    {
        wakeState = ANEMOMETER_INTERRUPT;
    windClicks++;
    if (currentTime < shortestWindTime)
    {
      shortestWindTime = currentTime;
    }

    return;
    }

    wakeState = IGNORE_INTERRUPT;
  */




}


