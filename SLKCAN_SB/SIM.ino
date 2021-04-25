
#ifdef SIM
// Simulate for testing.... 
void SimVariables()
{
  if(BatterySoc < 100)
  {
    BatterySoc++;
  }
  else if (BatterySoc >= 100)
  {
    BatterySoc = 0;
  }

  if(amps < 20)
  {
    amps++;
  }
  else if (amps >= 20)
  {
    amps = 0;
  }
  
  watts = (float)(amps * 72.0);
  
  odo = 100;
  tripkm = 25.0;
  gearmode = 1;

  kphhires = 35.0;

  posTemp = 22.0;
}
#endif
