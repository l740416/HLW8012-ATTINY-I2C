
#define USE_WIRE

#include <avr/io.h>
#include <avr/interrupt.h>

#ifdef USE_WIRE
#include <Wire.h>
#else
#include <TinyWireS.h>
#endif

#define offsetof(st, m) ((size_t)(&((st *)0)->m))

// Firmware version
#define VERSION_MAJOR                   1
#define VERSION_MINOR                   0

// General define
#define TRUE    1
#define FALSE   0

// I2C commmands
#define SLAVE_ADDRESS                  0x04
#define MAX_SENT_BYTES                 5
#define CMD_VERSION                    0x00   // Out:4B
#define CMD_GET_POWER                  0x10   // Out:4B, UNIT: uW
#define CMD_GET_VOLTAGE                0x11   // Out:4B, UNIT: uV
#define CMD_GET_CURRENT                0x12   // Out:4B, UNIT: uA
#define CMD_GET_POWER_MULTIPLIER       0x13   // Out:4B, UINT: uW/Hz
#define CMD_GET_VOLTAGE_MULTIPLIER     0x14   // Out:4B, UNIT: uV/Hz
#define CMD_GET_CURRENT_MULTIPLIER     0x15   // Out:4B, UNIT: uA/Hz
#define CMD_SET_VOLTAGE_UPSTREAM_REG   0x20   // In :2B, UNIT: 0.1KOhm 
#define CMD_SET_VOLTAGE_DOWNSTREAM_REG 0x21   // In :2B, UNIT: 0.1KOhm
#define CMD_SET_CURRENT_REG            0x22   // In :2B, UNIT: 1.0mOhm
#define CMD_SET_PULSE_TIMEOUT          0x23   // In :2B, UNIT: 1ms
#define CMD_UPDATE_REGISTERS           0x30
#define CMD_PERFORM_MEASUREMENT        0x31
#define CMD_EMPTY                      0xFF

// HLW8012 parameters
#define SEL_MEASURE_CURRENT LOW                         // SEL level to measure current
#define SEL_MEASURE_VOLTAGE HIGH                        // SEL level to measure voltage
#define SEL_PIN 1                                       // SEL pin
#define CF1_PIN 3                                       // CF1 pin (measure current/voltage)
#define CF_PIN  4                                       // CF  pin (measure power)
#define CURRENT_RESISTOR                0.004           // shunt register
#define VOLTAGE_RESISTOR_UPSTREAM       ( 6 * 470000 )  // voltage divider upper stream
#define VOLTAGE_RESISTOR_DOWNSTREAM     ( 1000 )        // voltage divider lower stream
#define PULSE_TIMEOUT                   1000000         // Timeout after changing SEL singal, unit: us

#define V_REF               2.43                        // Internal voltage reference value
#define F_OSC               3579000                     // Frequency of the HLW8012 internal clock


// I2C input values
struct input_t
{
  uint16_t voltage_up_reg;
  uint16_t voltage_down_reg;
  uint16_t current_reg;
  uint16_t pulse_timeout;
  uint8_t  update_regs;
  uint8_t  perform_measurement;
};

// I2C output values
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

// HLW8012 pulse width readings
enum pulse_type
{
  PULSE_TYPE_POWER = 0,
  PULSE_TYPE_VOLTAGE,
  PULSE_TYPE_CURRENT,
  PULSE_TYPE_NUM    
};

#define PULSE_RECORD_MAX 1
struct reading_t
{
  unsigned long pulse_width;        // Unit: us
  unsigned long last_rising_edge;   // Unit: us
};


volatile byte receivedCommands[MAX_SENT_BYTES];
volatile byte requestCommand;
volatile input_t  inputData;
volatile output_t outputData;
volatile reading_t measureData[PULSE_TYPE_NUM];
volatile byte SEL_signal;

uint32_t hlw8012_current_multiplier; // Unit: uA/Hz
uint32_t hlw8012_voltage_multiplier; // Unit: uV/Hz
uint32_t hlw8012_power_multiplier;   // Unit: uW/Hz

inline void init_HLW8012(double currentR, double voltage_upperR, double voltage_lowerR)
{
  double current_factor = currentR;
  double voltage_resistor = (voltage_upperR + voltage_lowerR) / voltage_lowerR;
  hlw8012_current_multiplier = ( 1000000.0 * 512 * V_REF / current_factor / 24.0 / F_OSC );
  hlw8012_voltage_multiplier = ( 1000000.0 * 512 * V_REF * voltage_resistor / 2.0 / F_OSC );
  hlw8012_power_multiplier = ( 1000000.0 * 128 * V_REF * V_REF * voltage_resistor / current_factor / 48.0 / F_OSC );

  outputData.power_multiplier = (uint32_t) hlw8012_power_multiplier;
  outputData.voltage_multiplier = (uint32_t) hlw8012_voltage_multiplier;
  outputData.current_multiplier = (uint32_t) hlw8012_current_multiplier;
}

double calc_frequency(pulse_type type)
{
  if(measureData[type].pulse_width == 0)
  {
    return 0.0;
  }
  return 1.0 / (measureData[type].pulse_width / 1000000.0);
}

void set_HLW8012_SEL(byte sel_type)
{
  SEL_signal = sel_type;
  digitalWrite(SEL_PIN, SEL_signal);
}

inline void clear_readings()
{
  clear_readings(PULSE_TYPE_POWER);
  clear_readings(PULSE_TYPE_VOLTAGE);
  clear_readings(PULSE_TYPE_CURRENT);
}

void clear_readings(pulse_type type)
{
  for( int i = 0 ; i < PULSE_RECORD_MAX; i++)
  {
    measureData[type].pulse_width = 0;
    measureData[type].last_rising_edge = 0;
  }
}



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

  inputData.voltage_up_reg = VOLTAGE_RESISTOR_UPSTREAM / 100.0;
  inputData.voltage_down_reg = VOLTAGE_RESISTOR_DOWNSTREAM / 100.0;
  inputData.current_reg = CURRENT_RESISTOR * 1000;
  inputData.pulse_timeout = PULSE_TIMEOUT;

  requestCommand = CMD_VERSION;
  outputData.version = ((uint32_t)VERSION_MAJOR << 16) | VERSION_MINOR;
  outputData.power = 0;
  outputData.voltage = 0;
  outputData.current = 0;

  clear_readings();

  SEL_signal = SEL_MEASURE_VOLTAGE;
  pinMode(SEL_PIN, OUTPUT);
  digitalWrite(SEL_PIN, SEL_signal);
  pinMode(CF_PIN, INPUT);
  pinMode(CF1_PIN, INPUT);

  // Blink LED
  for(int i = 0; i < 10; i++)
  {
    digitalWrite(SEL_PIN, LOW);
    delay(100);
    digitalWrite(SEL_PIN, HIGH);
    delay(100);
  }

  init_HLW8012(CURRENT_RESISTOR, VOLTAGE_RESISTOR_UPSTREAM, VOLTAGE_RESISTOR_DOWNSTREAM);

  // Setup CF_PIN, CF1_PIN interrupts
  cli();                                  // disable interrupts during setup
  PCMSK |= (1 << PCINT3) | (1 << PCINT4); // external interrupt on PB3, PB4
  GIMSK |= (1 << PCIE);                   // enable PCINT interrupt in the general interrupt mask //SBI
  sei();                                  // last line of setup - enable interrupts after setup

}

void loop()
{
  if (inputData.perform_measurement == TRUE)
  {   
    set_HLW8012_SEL(SEL_MEASURE_CURRENT);
    clear_readings(PULSE_TYPE_CURRENT);
    delay(1000);
    set_HLW8012_SEL(SEL_MEASURE_VOLTAGE);
    clear_readings(PULSE_TYPE_VOLTAGE);
    delay(1000);
    
    outputData.power = (uint32_t) (outputData.power_multiplier * calc_frequency(PULSE_TYPE_POWER));
    outputData.voltage = (uint32_t) (outputData.voltage_multiplier * calc_frequency(PULSE_TYPE_VOLTAGE));
    outputData.current = (uint32_t) (outputData.current_multiplier * calc_frequency(PULSE_TYPE_CURRENT));

    inputData.perform_measurement = FALSE;
  }

  else if (inputData.update_regs == TRUE)
  {
    init_HLW8012((double)inputData.current_reg      / 1000.0, 
                 (double)inputData.voltage_up_reg   *  100.0, 
                 (double)inputData.voltage_down_reg *  100.0);
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


volatile uint8_t portbhistory = 0xFF;

// interrupt handler
ISR(PCINT0_vect)
{
  unsigned long t = micros();
  byte measureDataType = PULSE_TYPE_NUM;

  uint8_t changedbits = changedbits = PINB ^ portbhistory;
  portbhistory = PINB;

  if(changedbits & (1 << PB3))
  {
    // CF1 signal
    if(portbhistory & (1 << PB3))
    {
      if(SEL_signal == SEL_MEASURE_VOLTAGE)
      {
        measureDataType = PULSE_TYPE_VOLTAGE;
      }
      else
      {
        measureDataType = PULSE_TYPE_CURRENT;
      }
    }
  }

  if(changedbits & (1 << PB4))
  {
    // CF signal
    if(portbhistory & (1 << PB4))
    {
      measureDataType = PULSE_TYPE_POWER;
    }
  }

  if(measureDataType < PULSE_TYPE_NUM)
  {
    if(measureData[measureDataType].last_rising_edge != 0)
    {
      measureData[measureDataType].pulse_width = t - measureData[measureDataType].last_rising_edge;
    }
    measureData[measureDataType].last_rising_edge = t;
  }
}


