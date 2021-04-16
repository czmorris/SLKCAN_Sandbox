#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//#define BTTESTTERM     /* If defined bluetooth sends a terminal output rather than realdash data */

BluetoothSerial SerialBT; 

void BTSetup ()
{
    SerialBT.begin("SLKCAN"); //Bluetooth device name
}

// Decicated to printing a terminal screen to bluetooth for testing. 
// Trying different things here now playing with different phone terminal emulators.
// Eventually need to write a custom phone app. 
void BTTask( void * parameter )
{
  // Get MCP messages as fast as possible...
  // but only as fast as possible. Does not allow ISR bog. 
  for(;;)
  { 
    
  // If STREAMCANMSGS is not enabled then print a terminal window on normal serial
  // This would probably be better done somewhere else...
  #ifndef STREAMCANMSGS
     printTermScreen();
     delay(500);
  #endif


#ifdef BTTESTTERM

    // Print a terminal output to bluetooth.
    SerialBT.print(CLS);
    SerialBT.print(HOME);
    SerialBT.print("BatSoc: ");
    SerialBT.println((int)BatterySoc);
    SerialBT.print("repthrottle: ");
    SerialBT.println(repthrottle);
    SerialBT.print("GearMode: ");
    SerialBT.println(gearmode);
    SerialBT.print("Odo: ");
    SerialBT.println(odo);
    SerialBT.print("PossibleAmps: ");
    SerialBT.println(PossibleAmps);
    SerialBT.print("tripkm: ");
    SerialBT.println(tripkm);
    SerialBT.print("ssStatus: ");
    SerialBT.println(ssStatus);
    SerialBT.print("posTemp: ");
    SerialBT.println(posTemp);

#else
    SendCANFramesToSerialBT();
#endif

    delay(100);
  }
}

void SendCANFramesToSerialBT()
{
  byte buf[8];

  // NOTE!!
  // This needs to be redone.
  // Study RealDash more and come up with the best method to transmit/pack each 
  // element of information. 

  // Commented out for now. 
  // For now I will focus on good clean data via the terminal.

//  byte kph = (byte)corrkph;
//  byte trip = (byte)tripkm;
//
//  // build 1st realdash CAN frame, batterysoc, speed, gearmode
//  memcpy(buf, &BatterySoc, 1);
//  memcpy(buf + 1, &kph, 1);
//  memcpy(buf + 2, &gearmode, 1);
//  memcpy(buf + 3, &odo, 1);
//  memcpy(buf + 4, &PossibleAmps, 1);   
//  memcpy(buf + 5, &trip, 1);
//  memcpy(buf + 6, &ssStatus, 1);
//  memcpy(buf + 7, &posTemp, 1);

//  SendCANFrameToSerialBT(3201, buf); // ?? 3201 ??
}

void SendCANFrameToSerialBT(unsigned long canFrameId, const byte* frameData)
{
  // the 4 byte identifier at the beginning of each CAN frame
  // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
  const byte serialBlockTag[4] = { 0x44, 0x33, 0x22, 0x11 };
  SerialBT.write(serialBlockTag, 4);

  // the CAN frame id number (as 32bit little endian value)
  SerialBT.write((const byte*)&canFrameId, 4);

  // CAN frame payload
  SerialBT.write(frameData, 8);
}
