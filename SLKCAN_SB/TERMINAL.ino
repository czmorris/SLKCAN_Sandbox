
#define CLS             "\033[2J"
#define HOME            "\033[H"

// Print a simplified terminal screen for quick testing CANbus finds. 
// Best viewed in a terminal emulator. (For example Putty)
void printTermScreen()
{ 
  Serial.print(CLS);
  Serial.print(HOME);
  Serial.print("Gear/Mode: ");
  Serial.println((int)gearmode);
  Serial.print("Battery Soc: ");
  Serial.println(BatterySoc);
  Serial.print("Amps: ");
  Serial.println(amps);
  Serial.print("Estimated Watts: ");
  Serial.println(watts);
  Serial.print("Odometer Miles: ");
  Serial.println((odo / 1.609));
  Serial.print("Trip Miles: ");
  Serial.println((tripkm / 1.609));  
  Serial.print("Speed kph: ");
  Serial.println(corrkph);  
  Serial.print("Speed mph: ");
  Serial.println((float)(corrkph / 1.609));    
  Serial.print("Controller Temp (C): ");
  Serial.println(posTemp); 
  Serial.print("Stand Status?: ");
  Serial.println(ssStatus, HEX); 
  Serial.print("Debug: ");
  Serial.println(debugcnt); 

}
