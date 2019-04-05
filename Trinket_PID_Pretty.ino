// portB0 is I2C data             <-- SetPoint
// portB1 is PWM hotplate control <-- Control Variable
// portB2 is I2C clock            <-- Setpoint
// portB3 is strong pullup        <-- Process Variable
// portB4 is 1-Wire I/O           <-- Process Variable


// the purpose of this trinket is to run a resistive heater via PID-control using PWM output (pin X) and measuring the temperature via ONE-WIRE, with a settable temp over i2c from
// 1) allow external device to set SETPOINT over I2C address 101 SDA PIN 5, SCL PIN 7
// 2) read temperature via ONE WIRE interface
// 3) set PWM output according to temp and PID control
//

// I2C INTERACTION SETUP STUFF ---- to communicate External World <-> PID controller ---------------------------
#include <TinyWireS.h> // i2c slave library
#define I2C_SLAVE_ADDRESS 0x10
#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif
// the registers that will be readable/alterable by the I2C master, including all relevant data
volatile uint8_t i2c_regs[] =
{
  0xfa,  // 00- arrays are 0 indexed
  0x21,  // 01- bit 5 indicates a device is present, bit one is the address CRC check
  0x02,  // 02- address family code (LSb)
  0x01,  // 03- address 48-bit serial number
  0x01,  // 04- address 48-bit serial number
  0x01,  // 05- address 48-bit serial number
  0x01,  // 06- address 48-bit serial number
  0x01,  // 07- address 48-bit serial number
  0x01,  // 08- address 48-bit serial number
  0x01,  // 09- address 8-bit CRC (MSb)
  0x01,  // 10- local comp of address CRC
  0x01,  // 11- scratchpad cold-junct-comp thermo T lsb  byte0
  0x01,  // 12- scratchpad
  0x01,  // 13- scratchpad
  0x01,  // 14- scratchpad
  0x01,  // 15- scratchpad config register
  0x01,  // 16- scratchpad ffh
  0x01,  // 17- scratchpad ffh
  0x01,  // 18- scratchpad ffh
  0x01,  // 19- scratchpad CRC
  0x01,  // 20- local comp of scratchpad CRC
  0x01,  // 21- CJC THERMO T byte1, divisor 16
  0x01,  // 22- CJC THERMO T byte2
  0x01,  // 23- CJ Temp byte1  divisor 256
  0x01,  // 24- CJ Temp byte2
  0x01,  // 25- corrected Temp byte 1, divisor 16
  0x01,  // 26- corrected temp byte 2
  0x01,  // 27- setpoint byte 1, divisor 16
  0x90,   // 28- setpoint byte 2
  0x01
}; // not sure if those hex numbers mean anything
// These variables track the register pointer position and range
volatile byte reg_position=0;
const byte reg_size = sizeof(i2c_regs);



// a set of constants to correct the "linearized" thermocouple reading
const float coeffmVperC[] = {0E0, 0.387481063640E-1, 0.332922278800E-4, 0.206182434040E-6 , -0.21882256846E-8, 0.109968809280E-10, -0.30815758720E-13, 0.454791352900E-16, -0.27512901673E-19};  
const float coeffCpermV[] = {0E0, 2.592800E1, -7.602961E-1, 4.637791E-2, -2.165394E-3, 6.048144E-5, -7.293422E-7, 0E0};


// ONE WIRE SETUP STUFF ---- to communicate PID Controller <-> Thermocouple reader  -----------------------------------
//#include <maxim-crc8.h> skipped to save memory
// slower, more compact CRC is used included in onewire.cpp
#include <OneWire.h>
OneWire maxim1(4);
byte present = 0;
//byte data[12];
byte addr[8];
//void convertT(OneWire &maxim1, byte address[8]);
//void readScratchpad(OneWire &maxim1, byte address[8]);
//void datatoTemps();

// timer interrupt setup stuff and PID
volatile int counter = 0;
volatile float SetPoint, inputT, lastTErr, TError, intTerm, PWMOutput,kP,kI,kD; 




// the setup routine runs once when you press the reset button:
void setup() {
  
  // initialize the Control Variable pin as an output.
  pinMode(1, OUTPUT);
  analogWrite(1,255);

  // initialize the I2C slave routines so that other devices can find the PID controller
  TinyWireS.begin(I2C_SLAVE_ADDRESS);
  TinyWireS.onRequest(requestEvent);
  TinyWireS.onReceive(receiveEvent);

  // Test the One-Wire bus and find the Thermocouple reader
  int i;
  if ( !maxim1.search(addr)) {
    maxim1.reset_search();
    tws_delay(250);  // a specialized delay function that doesnt interfere with I2C-slave communication
    return;
  }
  //If a thermocouple reader is found, populate its information into the I2C registers
  present = maxim1.reset();
  if (present) { 
    i2c_regs[1] = i2c_regs[1] | (1 << 4);}
  for ( i = 2; i < 10; i++) { 
    i2c_regs[i] = addr[i - 2];}
  i2c_regs[10] = OneWire::crc8( addr, 7);

  // Setup the PID control loop itself
  SetPoint = 00.0; //https://www.thorlabs.com/tutorials.cfm?tabID=5dfca308-d07e-46c9-baa0-4defc5c40c3e
  kP = 4.2;   //at 40,0,0 begins oscillation with front window open, at 7,0,0 with front window closed
  kI = 0.1;   //1.8*kP/50;      no front windows: 0.4
  kD = 2.4;   //(kP*50.0)/8.0;  no front window:  24
 
}

// the loop routine runs over and over again forever:
void loop() {
  // This needs to be here
  TinyWireS_stop_check();
  counter ++;
  //TCNT1=6;
  // try to separate the steps temporally to make sure the chip has time to compute before i read and im ont waiting
  if (counter == 50){
    convertT();
  }
  else if (counter == 75){
    readScratchpad();
    int16_t SetP;
    //noInterrupts();
    SetP = (((i2c_regs[27]<<8)&0xFF00)|((i2c_regs[28])&0x00FF));
    SetPoint = (float)SetP/16.0;
    if (SetPoint > 255)
    {
      SetPoint = 20.0;
    }
    //interrupts();
  }
  else if (counter == 100){
    datatoTemps();
  }
  else if (counter > 124){
    counter = 0;
    TError = SetPoint-inputT;
    if (TError > -6.0){
      intTerm += (kI*TError);
    }
    else{ intTerm = 0;}
    if (intTerm > 500){
      intTerm = 500;
    }
    else if (intTerm < -50){
      intTerm = -50;
    }
    
    PWMOutput = 255-(kP*TError + intTerm + kD*(TError-lastTErr));
    lastTErr=TError;
    if (PWMOutput>255){
      PWMOutput = 255;
    }
    else if (PWMOutput < 0){
      PWMOutput = 0;
    }
    //OCR0B=(int)PWMOutput;
    analogWrite(1,PWMOutput);
  } 
  tws_delay(40);
}

void requestEvent()
// this function will send the current register values
{
  //TinyWireS.flushBuffers();
  TinyWireS.send(i2c_regs[reg_position]);
  // Increment the reg position on each read, and loop back to zero
  //reg_position++;
  //if (reg_position >= reg_size)
  //{
  reg_position = 0;
  //}
  //TinyWireS.flushBuffers();
}


void receiveEvent(uint8_t howMany)
// this function will take the received data and change the setpoint to match
{
  //if (howMany < 1)
  //{
  //  // Sanity-check
  //  return;
  //}
  //if (howMany > TWI_RX_BUFFER_SIZE)
  //{
  //  // Also insane number
  //  return;
  //}
  
  if ((reg_position == 27) | (reg_position==28))
  {
    i2c_regs[reg_position] = TinyWireS.receive();
    reg_position = 0;
    howMany--;
  }
  else
  {
    reg_position = TinyWireS.receive();
    howMany--;
  }
  if (howMany<1){return;}
  while (howMany-- > 0)
  {
    TinyWireS.receive();
  }
  TinyWireS.flushBuffers();
  
  /*
  byte bob = TinyWireS.receive();
  reg_position = bob;
  howMany--;
  
  if (!howMany)
  {
    // This write was only to set the buffer for next read
    return;
  }
  while (howMany--)
  {
    if (bob == 27){
      i2c_regs[27] = TinyWireS.receive();
      bob++;
    }
    if (bob == 28){
      i2c_regs[28] = TinyWireS.receive();
    }
    reg_position = bob;
    //i2c_regs[reg_position] = TinyWireS.receive();
    //reg_position++;
    TinyWireS.receive();
    if (reg_position >= reg_size)
    {
      reg_position = 0;
    }
  }
  //TinyWireS.flushBuffers();
  */
}

// FUNCTIONS FOR ONE WIRE INTERACTION________ for PID controller <-> Max Thermocouple chip
void convertT()
// this function tells a maxim 31850 chip at address (addr) to take a T measurement
{
  maxim1.reset();
  maxim1.select(addr);
  maxim1.write(0x44, 1); // tells chip to start conversion
}

void readScratchpad()
//
{
  present = maxim1.reset();
  maxim1.select(addr);
  maxim1.write(0xBE);         // Read Scratchpad

  int i;
  noInterrupts();
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    i2c_regs[i+11] = maxim1.read();
  }
  interrupts();
}

void datatoTemps()
{
  //int i;
  //for ( i = 11; i < 20; i++) {
  //  //Serial.print(addr[i], HEX);
  //  //Serial.print(" ");
  //  i2c_regs[i] = data[i - 11];
  //}
  uint16_t reg1,reg2;
  uint16_t cjTemp, hjTemp, totTemp;
  float cjTemp_fl, hjTemp_fl,totTemp_fl,cjmV, hjmV,totmV;
  
  noInterrupts();
  reg1 = i2c_regs[14];
  reg2 = i2c_regs[13];
  cjTemp = (((reg1<<8)&0xFF00)|((reg2)&0x00F0));
  
  reg1 = i2c_regs[12];
  reg2 = i2c_regs[11];
  hjTemp = (((reg1<<8) & 0xFF00) | ((reg2) & 0x00FC));
  
  cjTemp_fl = ((float)cjTemp)/256.0;
  hjTemp_fl = ((float)hjTemp)/16.0;
  // now we awkwardly have to take these temperatures and correct them using the NIST polynomials to get the _actual_ temperatures.
  // first convert the temperature back into thermocouple measured voltage
  hjmV = (hjTemp_fl-cjTemp_fl)*.05218;          // this is the linear term used in the microchip we undo to get the hot junction voltage
  cjmV = 0.0;                                   // reset the cold junction value
  cjmV += coeffmVperC[1]*cjTemp_fl;             // linear term of NIST polynomial
  cjmV += coeffmVperC[2]*cjTemp_fl*cjTemp_fl;   // squared term of NIST polynomial
  cjmV += coeffmVperC[3]*cjTemp_fl*cjTemp_fl*cjTemp_fl; // cubed term
  cjmV += coeffmVperC[4]*cjTemp_fl*cjTemp_fl*cjTemp_fl*cjTemp_fl; // quad(?) term
  cjmV += coeffmVperC[5]*cjTemp_fl*cjTemp_fl*cjTemp_fl*cjTemp_fl*cjTemp_fl; // quint-term
  totmV = cjmV + hjmV;                          // now we have the total voltage
  totTemp_fl = 0.0;                             // now we do the whole process forwards again to get the correct temperature
  totTemp_fl += coeffCpermV[1]*totmV;           // linear term of NIST polynomial
  totTemp_fl += coeffCpermV[2]*totmV*totmV;     // squared term of NIST polynomial
  totTemp_fl += coeffCpermV[3]*totmV*totmV*totmV;   // cubed term 
  totTemp_fl += coeffCpermV[4]*totmV*totmV*totmV*totmV; // quaded term
  totTemp_fl += coeffCpermV[5]*totmV*totmV*totmV*totmV*totmV; // quinted term
  totTemp = (uint16_t)(totTemp_fl*16.0);  // convert it for output over i2c (we want to shove it into 2 bytes, but not just truncate it to an integer)
  inputT = totTemp_fl;
  
  
  //i2c_regs[20] = OneWire::crc8( data, 8);
  
  i2c_regs[21] = ((hjTemp >> 8) & 0xFF);        // ThermoT byte 1
  i2c_regs[22] = ((hjTemp) & 0xFF) ;            // ThermoT byte 2 divide by 16
  i2c_regs[23] = ((cjTemp >> 8) & 0xFF);        // CJ T byte 1
  i2c_regs[24] = ((cjTemp) & 0xFF) ;            // CJ T byte 2 divide by 256
  i2c_regs[25] = ((totTemp >> 8) & 0xFF);       // corrected T byte 1
  i2c_regs[26] = ((totTemp) & 0xFF) ;           // corrected T  byte 2 divide by 16
  interrupts();
}

//SetPoint, inputT, TError, intTerm, PWMOutput
/*
ISR(TIMER1_OVF_vect){
  counter ++;
  TCNT1=6;
  // try to separate the steps temporally to make sure the chip has time to compute before i read and im ont waiting
  if (counter == 50){
    convertT();
  }
  else if (counter == 75){
    readScratchpad();
    int16_t SetP;
    //noInterrupts();
    SetP = (((i2c_regs[27]<<8)&0xFF00)|((i2c_regs[28])&0x00FF));
    SetPoint = (float)SetP/16.0;
    //interrupts();
  }
  else if (counter == 100){
    datatoTemps();
  }
  else if (counter > 124){
    counter = 0;
    TError = SetPoint-inputT;
    intTerm += (kI*TError);
    if (intTerm > 255){
      intTerm = 255;
    }
    else if (intTerm < 0){
      intTerm = 0;
    }
    PWMOutput = 255-(kP*TError + intTerm + kD*(inputT-lastTErr));
    lastTErr=inputT;
    if (PWMOutput>255){
      PWMOutput = 255;
    }
    else if (PWMOutput < 0){
      PWMOutput = 0;
    }
    OCR0B=(int)PWMOutput;
  } 
}
*/
