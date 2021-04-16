//#define SIM  // run sim to manipulate some values, useful for testing without bike

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
  //tripkmfloat = 10;

  gearmode = 1;
  repthrottle = 50;

  //delay(1000);
}
#endif
