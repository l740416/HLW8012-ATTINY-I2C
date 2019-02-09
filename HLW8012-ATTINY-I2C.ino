
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
#define VERSION_MINOR                   3

// General define
#define TRUE    1
#define FALSE   0

// I2C general
#define SLAVE_ADDRESS                  0x04
#define MAX_SENT_BYTES                 5

// I2C read commmands
#define CMD_READ_COMMAND_FIRST         0x00
#define CMD_VERSION                    0x00   // Out:4B
#define CMD_GET_POWER                  0x01   // Out:4B, UNIT: uW
#define CMD_GET_VOLTAGE                0x02   // Out:4B, UNIT: uV
#define CMD_GET_CURRENT                0x03   // Out:4B, UNIT: uA
#define CMD_GET_POWER_MULTIPLIER       0x04   // Out:4B, UINT: uW/Hz
#define CMD_GET_VOLTAGE_MULTIPLIER     0x05   // Out:4B, UNIT: uV/Hz
#define CMD_GET_CURRENT_MULTIPLIER     0x06   // Out:4B, UNIT: uA/Hz
#define CMD_GET_POWER_PULSEWIDTH       0x07   // Out:4B, UNIT: us
#define CMD_GET_VOLTAGE_PULSEWIDTH     0x08   // Out:4B, UNIT: us
#define CMD_GET_CURRENT_PULSEWIDTH     0x09   // Out:4B, UNIT: us
#define CMD_READ_COMMAND_LAST          0x09

// I2C write commmands
#define CMD_WRITE_COMMAND_FIRST        0x10
#define CMD_SET_VOLTAGE_UPSTREAM_REG   0x10   // In :2B, UNIT: 0.1KOhm 
#define CMD_SET_VOLTAGE_DOWNSTREAM_REG 0x11   // In :2B, UNIT: 0.1KOhm
#define CMD_SET_CURRENT_REG            0x12   // In :2B, UNIT: 1.0mOhm
#define CMD_SET_EXPECT_VOLTAGE         0x13   // In :2B, UNIT: 0.01 V
#define CMD_SET_EXPECT_CURRENT         0x14   // In :2B, UNIT: 0.001 A
#define CMD_SET_EXPECT_POWER           0x15   // In :2B, UNIT: 0.01 W
#define CMD_WRITE_COMMAND_LAST         0x15

// I2C action commmands
#define CMD_ACTION_COMMAND_FIRST       0x20
#define CMD_UPDATE_REGISTERS           0x20
#define CMD_PERFORM_MEASUREMENT        0x21
#define CMD_CALIBRATE_VOLTAGE          0x22
#define CMD_CALIBRATE_CURRENT          0x23
#define CMD_CALIBRATE_POWER            0x24
#define CMD_ACTION_COMMAND_LAST        0x24

// HLW8012 parameters
#define SEL_MEASURE_CURRENT LOW                         // SEL level to measure current
#define SEL_MEASURE_VOLTAGE HIGH                        // SEL level to measure voltage
#define SEL_PIN 1                                       // SEL pin
#define CF1_PIN 3                                       // CF1 pin (measure current/voltage)
#define CF_PIN  4                                       // CF  pin (measure power)
#define CURRENT_RESISTOR                0.004           // shunt register
#define VOLTAGE_RESISTOR_UPSTREAM       ( 6 * 470000 )  // voltage divider upper stream
#define VOLTAGE_RESISTOR_DOWNSTREAM     ( 1000 )        // voltage divider lower stream

#define SEL_TOGGLE_TIMEOUT_MS           1000            // Toggle SEL singal, unit: ms

#define V_REF               2.43                        // Internal voltage reference value
#define F_OSC               3579000                     // Frequency of the HLW8012 internal clock


// I2C input values
// The order of the values is very IMPROTANT
// It must follow the order of I2C write commmands
const uint8_t NumWriteVarsIn2Byte = 6;
struct input_t
{
  uint16_t voltage_up_reg;
  uint16_t voltage_down_reg;
  uint16_t current_reg;
  uint16_t expect_voltage;
  uint16_t expect_current;
  uint16_t expect_power;
  uint8_t  update_regs;
  uint8_t  perform_measurement;
  uint8_t  calibrate_voltage;
  uint8_t  calibrate_current;
  uint8_t  calibrate_power;
};

// I2C output values
// The order of the values is very IMPROTANT
// It must follow the order of I2C read commmands
struct output_t
{
  uint32_t version;
  uint32_t power;
  uint32_t voltage;
  uint32_t current;   
  uint32_t power_multiplier;     // Unit: uW/Hz
  uint32_t voltage_multiplier;   // Unit: uV/Hz
  uint32_t current_multiplier;   // Unit: uA/Hz
  uint32_t power_pulse_width;    // Uint: us
  uint32_t voltage_pulse_width;  // Uint: us
  uint32_t current_pulse_width;  // Uint: us 
};

// HLW8012 pulse width readings
enum pulse_type
{
  PULSE_TYPE_POWER = 0,
  PULSE_TYPE_VOLTAGE,
  PULSE_TYPE_CURRENT,
  PULSE_TYPE_NULL,                // Used to avoid boundary check
  PULSE_TYPE_NUM    
};

#define STABLE_PULSE_COUNT 32     // Start to measure pulse after 32 pluses
#define ACCUMULATE_PULSE_COUNT 64 // Accumulate 64 pulses and then average it 
struct reading_t
{
  unsigned long pulse_width;        // Unit: us
  unsigned long last_rising_edge;   // Unit: us
};

// Global veriables
byte receivedCommands[MAX_SENT_BYTES];
byte requestCommand = CMD_VERSION;
input_t  inputData = {
  VOLTAGE_RESISTOR_UPSTREAM,
  VOLTAGE_RESISTOR_DOWNSTREAM,
  CURRENT_RESISTOR,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0
};
output_t outputData = {0, 0, 0 ,0 ,0 ,0, 0 ,0 ,0, 0};
reading_t measureData[PULSE_TYPE_NUM];
byte SEL_signal;

inline void init_HLW8012(double currentR, double voltage_upperR, double voltage_lowerR)
{
  double current_factor = currentR;
  double voltage_factor = (voltage_upperR + voltage_lowerR) / voltage_lowerR;

  outputData.current_multiplier = (uint32_t) ((( 1000000.0 * 512 * V_REF ) / ( 24.0 * F_OSC )) / current_factor);
  outputData.voltage_multiplier = (uint32_t) ((( 1000000.0 * 512 * V_REF  ) / ( 2.0 * F_OSC )) * voltage_factor);
  outputData.power_multiplier = (uint32_t) ((( 1000000.0 * 128 * V_REF * V_REF ) / ( 48.0 * F_OSC )) * ( voltage_factor / current_factor ));
}

double calc_frequency(pulse_type type)
{
  if(measureData[type].pulse_width == 0)
  {
    return 0.0;
  }
  return 1000000.0 / measureData[type].pulse_width;
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
  measureData[type].pulse_width = 0;
  measureData[type].last_rising_edge = 0;
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

  outputData.version = ((uint32_t)VERSION_MAJOR << 16) | VERSION_MINOR;
  //clear_readings();

  pinMode(SEL_PIN, OUTPUT);
  pinMode(CF_PIN, INPUT);
  pinMode(CF1_PIN, INPUT);
  set_HLW8012_SEL(SEL_MEASURE_VOLTAGE);

  // Blink LED
  for(int i = 0; i < 10; i++)
  {
    digitalWrite(SEL_PIN, LOW);
    delay(100);
    digitalWrite(SEL_PIN, HIGH);
    delay(100);
  }

  init_HLW8012(CURRENT_RESISTOR, VOLTAGE_RESISTOR_UPSTREAM, VOLTAGE_RESISTOR_DOWNSTREAM);

  // Only set interrupt on power pin
  // Use pulseIn for voltage/current pin
  cli();                                  // disable interrupts during setup
  PCMSK |= (1 << PCINT4); // external interrupt on PB4
  GIMSK |= (1 << PCIE);                   // enable PCINT interrupt in the general interrupt mask //SBI
  sei();                                  // last line of setup - enable interrupts after setup

}

unsigned long last_change_sel = 0;

void loop()
{
  if (inputData.perform_measurement == TRUE)
  {  
    outputData.power_pulse_width = measureData[PULSE_TYPE_POWER].pulse_width;
    outputData.voltage_pulse_width = measureData[PULSE_TYPE_VOLTAGE].pulse_width;
    outputData.current_pulse_width = measureData[PULSE_TYPE_CURRENT].pulse_width;
     
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

  else if (inputData.calibrate_voltage == TRUE)
  {
    outputData.voltage_multiplier *= (10000.0 * (double) inputData.expect_voltage / (double) outputData.voltage);
    inputData.calibrate_voltage = FALSE;
  }

  else if (inputData.calibrate_current == TRUE)
  {
    outputData.current_multiplier *= (1000.0 * (double) inputData.expect_current / (double) outputData.current);
    inputData.calibrate_current = FALSE;
  }

  else if (inputData.calibrate_power == TRUE)
  {
    outputData.power_multiplier *= (10000.0 * (double) inputData.expect_power / (double) outputData.power);
    inputData.calibrate_power = FALSE;
  }

  // calculate result if nothing to do
  else
  {
    outputData.power_pulse_width = measureData[PULSE_TYPE_POWER].pulse_width;
    outputData.voltage_pulse_width = measureData[PULSE_TYPE_VOLTAGE].pulse_width;
    outputData.current_pulse_width = measureData[PULSE_TYPE_CURRENT].pulse_width;
     
    uint32_t power = (uint32_t) (outputData.power_multiplier * calc_frequency(PULSE_TYPE_POWER));
    outputData.voltage = (uint32_t) (outputData.voltage_multiplier * calc_frequency(PULSE_TYPE_VOLTAGE));
    outputData.current = (uint32_t) (outputData.current_multiplier * calc_frequency(PULSE_TYPE_CURRENT));

    // discard bad reading, when power < 1W but current > 0.01A
    if (!(power < 1000000 && outputData.current > 10000))
    {
      outputData.power = power; 
    }
  }

  unsigned long t = millis();
  if (t - last_change_sel > SEL_TOGGLE_TIMEOUT_MS)
  {
    last_change_sel = t;

    if(SEL_signal == SEL_MEASURE_VOLTAGE)
    {
      measureData[PULSE_TYPE_VOLTAGE].pulse_width = pulseIn(CF1_PIN, HIGH) * 2;
    }
    else
    {
      measureData[PULSE_TYPE_CURRENT].pulse_width = pulseIn(CF1_PIN, HIGH) * 2;
    }
    set_HLW8012_SEL(!SEL_signal);   
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

  if(receivedCommands[0] >= CMD_READ_COMMAND_FIRST && receivedCommands[0] <= CMD_READ_COMMAND_LAST)
  {
    requestCommand = receivedCommands[0] - CMD_READ_COMMAND_FIRST;
  }
  else if(receivedCommands[0] >= CMD_WRITE_COMMAND_FIRST && receivedCommands[0] <= CMD_WRITE_COMMAND_LAST)
  {
    uint16_t* writePtr = (uint16_t*)(&inputData) + (receivedCommands[0] - CMD_WRITE_COMMAND_FIRST);
    uint16_t writeValue = receivedCommands[1] | (receivedCommands[2] << 8);
    *writePtr = writeValue;
  }
  else if(receivedCommands[0] >= CMD_ACTION_COMMAND_FIRST && receivedCommands[0] <= CMD_ACTION_COMMAND_LAST)
  {
    // there are NumWriteVarsIn2Byte uint16_t value ahead of the action input variables (all uint8_t)
    uint8_t* actionPtr = ((uint8_t*)((uint16_t*)(&inputData) + NumWriteVarsIn2Byte)) + (receivedCommands[0] - CMD_ACTION_COMMAND_FIRST);
    *actionPtr = TRUE;
  }

}

void requestEvent()
{
  uint8_t* ptr = ((uint8_t *)(&(outputData))) + ((requestCommand - CMD_READ_COMMAND_FIRST) << 2);

#ifdef USE_WIRE
  Wire.write(ptr, 4);
#else    
  TinyWireS.send(ptr[0]);
  TinyWireS.send(ptr[1]);
  TinyWireS.send(ptr[2]);
  TinyWireS.send(ptr[3]);
#endif
}


volatile uint8_t portbhistory = 0xFF;

// interrupt handler
ISR(PCINT0_vect)
{       
  unsigned long t = micros();
  uint8_t changedbits = PINB ^ portbhistory;
  portbhistory = PINB;

  if(changedbits & (1 << PB4))
  {
    // CF signal
    if(portbhistory & (1 << PB4))
    {
      measureData[PULSE_TYPE_POWER].pulse_width = t - measureData[PULSE_TYPE_POWER].last_rising_edge;
      measureData[PULSE_TYPE_POWER].last_rising_edge = t;
    }
  }
}
