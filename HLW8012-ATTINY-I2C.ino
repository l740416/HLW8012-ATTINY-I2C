
#define USE_WIRE

#include <avr/io.h>
#include <avr/interrupt.h>

#ifdef USE_WIRE
#include <Wire.h>
#else
#include <TinyWireS.h>
#endif

#include "src/HLW8012/HLW8012.h"

#define INTERRUPTPIN PCINT1 //this is PB1 per the schematic
#define PCINT_VECTOR PCINT0_vect  //this step is not necessary
#define DATADIRECTIONPIN DDB1 //Page 64 of data sheet
#define PORTPIN PB1 //Page 64
#define READPIN PINB1 //page 64
#define LEDPIN 4 //PB4


#define offsetof(st, m) ((size_t)(&((st *)0)->m))

#define VERSION_MAJOR                   0
#define VERSION_MINOR                   2

#define TRUE    1
#define FALSE   0

#define SLAVE_ADDRESS                  0x04
#define MAX_SENT_BYTES                 5
#define CMD_VERSION                    0x00   // Out:4B
#define CMD_GET_POWER                  0x10   // Out:4B, UNIT: 0.1W
#define CMD_GET_VOLTAGE                0x11   // Out:4B, UNIT: 0.01V
#define CMD_GET_CURRENT                0x12   // Out:4B, UNIT: 0.01A
#define CMD_GET_POWER_MULTIPLIER       0x13   // Out:4B, UINT: 1000x
#define CMD_GET_VOLTAGE_MULTIPLIER     0x14   // Out:4B, UNIT: 1000x
#define CMD_GET_CURRENT_MULTIPLIER     0x15   // Out:4B, UNIT: 1000x
#define CMD_SET_VOLTAGE_UPSTREAM_REG   0x20   // In :2B, UNIT: 0.1KOhm 
#define CMD_SET_VOLTAGE_DOWNSTREAM_REG 0x21   // In :2B, UNIT: 0.1KOhm
#define CMD_SET_CURRENT_REG            0x22   // In :2B, UNIT: 1.0mOhm
#define CMD_SET_PULSE_TIMEOUT          0x23   // In :2B, UNIT: 1ms
#define CMD_UPDATE_REGISTERS           0x30
#define CMD_PERFORM_MEASUREMENT        0x31
#define CMD_EMPTY                      0xFF

struct input_t
{
  uint16_t voltage_up_reg;
  uint16_t voltage_down_reg;
  uint16_t current_reg;
  uint16_t pulse_timeout;
  uint8_t  update_regs;
  uint8_t  perform_measurement;
};

struct output_t
{
  uint32_t version;
  uint32_t power;
  uint32_t voltage;
  uint32_t current;   
  uint32_t power_multiplier;   
  uint32_t voltage_multiplier;   
  uint32_t current_multiplier;   
  
};

volatile byte receivedCommands[MAX_SENT_BYTES];
volatile byte requestCommand;
volatile input_t  inputData;
volatile output_t outputData;

#define SEL_PIN 1
#define CF1_PIN 3
#define CF_PIN  4
#define UPDATE_TIME                     5000000
#define CURRENT_MODE                    HIGH
#define CURRENT_RESISTOR                0.004
#define VOLTAGE_RESISTOR_UPSTREAM       ( 6 * 470000 )
#define VOLTAGE_RESISTOR_DOWNSTREAM     ( 1000 )

HLW8012 hlw8012;

void setup()
{
#ifdef USE_WIRE
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
#else
  TinyWireS.begin(SLAVE_ADDRESS);
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);
#endif

  hlw8012.begin(CF_PIN, CF1_PIN, SEL_PIN, CURRENT_MODE, UPDATE_TIME);
  hlw8012.setResistors(CURRENT_RESISTOR, VOLTAGE_RESISTOR_UPSTREAM, VOLTAGE_RESISTOR_DOWNSTREAM);

  requestCommand = CMD_VERSION;
  outputData.version = (VERSION_MAJOR << 16) | VERSION_MINOR;
  outputData.power = 0;
  outputData.voltage = 1;
  outputData.current = 0;

  outputData.power_multiplier = hlw8012.getPowerMultiplier(1.0);
  outputData.voltage_multiplier = hlw8012.getVoltageMultiplier(1.0);
  outputData.current_multiplier = hlw8012.getCurrentMultiplier(1.0);
}

unsigned long last_update = 0;

void loop()
{
  if (inputData.perform_measurement == TRUE)
  {   
    outputData.power = hlw8012.getActivePower(1000.0);
    hlw8012.setMode(MODE_VOLTAGE);
    outputData.voltage = hlw8012.getVoltage(1000.0);
    hlw8012.setMode(MODE_CURRENT);
    outputData.current = hlw8012.getCurrent(1000.0);
    
    inputData.perform_measurement = FALSE;
  }

  if (inputData.update_regs == TRUE)
  {    
    hlw8012.setResistors((double)inputData.current_reg      / 1000.0, 
                         (double)inputData.voltage_up_reg   *  100.0, 
                         (double)inputData.voltage_down_reg *  100.0);
    hlw8012.setPulseTimeout((uint32_t)inputData.pulse_timeout * 1000);
    inputData.update_regs = FALSE;
  }

#ifndef USE_WIRE
  TinyWireS_stop_check();
#endif
}

void receiveEvent(uint8_t bytesReceived) 
{  
  for(uint8_t i = 0; i < bytesReceived; i++)
  {
    if(i < MAX_SENT_BYTES)
    {
#ifdef USE_WIRE
      receivedCommands[i] = Wire.read();
#else
      receivedCommands[i] = TinyWireS.receive();
#endif
    }
    else
    {
      // if we receive more data then allowed just throw it away
#ifdef USE_WIRE
      char c = Wire.read();
#else
      TinyWireS.receive();
#endif
    }    
  }

  switch(receivedCommands[0])
  {
    case CMD_VERSION:
    case CMD_GET_POWER:
    case CMD_GET_VOLTAGE:
    case CMD_GET_CURRENT:
    case CMD_GET_POWER_MULTIPLIER:
    case CMD_GET_VOLTAGE_MULTIPLIER:
    case CMD_GET_CURRENT_MULTIPLIER:
      if(bytesReceived == 1)
      {
        requestCommand = receivedCommands[0];
      }
      return;
      break;

    case CMD_SET_VOLTAGE_UPSTREAM_REG:
      if(bytesReceived == 3)
      {
        inputData.voltage_up_reg = receivedCommands[1] | (receivedCommands[2] << 8);
      }
      return;
      break;
      
    case CMD_SET_VOLTAGE_DOWNSTREAM_REG:
      if(bytesReceived == 3)
      {
        inputData.voltage_down_reg = receivedCommands[1] | (receivedCommands[2] << 8);
      }
      return;
      break;
      
    case CMD_SET_CURRENT_REG:
      if(bytesReceived == 3)
      {
        inputData.current_reg = receivedCommands[1] | (receivedCommands[2] << 8);
      }
      return;
      break;

    case CMD_SET_PULSE_TIMEOUT:
      if(bytesReceived == 3)
      {
        inputData.pulse_timeout = receivedCommands[1] | (receivedCommands[2] << 8);
      }
      return;
      break;
      
    case CMD_UPDATE_REGISTERS:
      if(bytesReceived == 1)
      {
        inputData.update_regs = TRUE;
      }
      return;
      break;
      
    case CMD_PERFORM_MEASUREMENT:
      if(bytesReceived == 1)
      {
        inputData.perform_measurement = TRUE;
      }
      return;
      break;
 
    default:
      return;
  }
}

void requestEvent()
{
  uint8_t* ptr = NULL;
 
  switch(requestCommand)
  {
    case CMD_VERSION:
      ptr = (uint8_t *)(&(outputData.version));
      break;

    case CMD_GET_POWER:
      ptr = (uint8_t *)(&(outputData.power));
      break;

    case CMD_GET_VOLTAGE:
      ptr = (uint8_t *)(&(outputData.voltage));
      break;

    case CMD_GET_CURRENT:
      ptr = (uint8_t *)(&(outputData.current));
      break;

    case CMD_GET_POWER_MULTIPLIER:
      ptr = (uint8_t *)(&(outputData.power_multiplier));
      break;

    case CMD_GET_VOLTAGE_MULTIPLIER:
      ptr = (uint8_t *)(&(outputData.voltage_multiplier));
      break;

    case CMD_GET_CURRENT_MULTIPLIER:
      ptr = (uint8_t *)(&(outputData.current_multiplier));
      break;

  }

  if(ptr != NULL)
  {
#ifdef USE_WIRE
    Wire.write(ptr, 4);
#else    
    TinyWireS.send(ptr[0]);
    TinyWireS.send(ptr[1]);
    TinyWireS.send(ptr[2]);
    TinyWireS.send(ptr[3]);
#endif
  }
  else
  {
#ifdef USE_WIRE
    Wire.write('0xFF');
    Wire.write('0xFF');
    Wire.write('0xFF');
    Wire.write('0xFF');
#else 
    TinyWireS.send(0xFF);
    TinyWireS.send(0xFF);
    TinyWireS.send(0xFF);
    TinyWireS.send(0xFF);
#endif
  }
}
