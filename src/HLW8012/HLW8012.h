/*

HLW8012 1.0.0

Copyright (C) 2016 by Xose Pérez <xose dot perez at gmail dot com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef HLW8012_h
#define HLW8012_h

#include <Arduino.h>

// Internal voltage reference value
#define V_REF               2.43

// The factor of a 1mOhm resistor
// as per recomended circuit in datasheet
// A 1mOhm resistor allows a ~30A max measurement
#define R_CURRENT           0.001

// This is the factor of a voltage divider of 6x 470K upstream and 1k downstream
// as per recomended circuit in datasheet
#define R_VOLTAGE           2821

// Frequency of the HLW8012 internal clock
#define F_OSC               3579000

// Minimum delay between selecting a mode and reading a sample
#define READING_INTERVAL    3000

// Maximum pulse with in microseconds
// If longer than this pulse width is reset to 0
// This value is purely experimental.
// Higher values allow for a better precission but reduce sampling rate
// and response speed to change
// Lower values increase sampling rate but reduce precission
// Values below 0.5s are not recommended since current and voltage output
// will have no time to stabilise
#define PULSE_TIMEOUT       2000000

// CF1 mode
typedef enum {
    MODE_CURRENT,
    MODE_VOLTAGE
} hlw8012_mode_t;

class HLW8012 {

    public:

        void cf_interrupt();
        void cf1_interrupt();

        void begin(
            uint8_t cf_pin,
            uint8_t cf1_pin,
            uint8_t sel_pin,
            uint8_t currentWhen = HIGH,
            uint32_t pulse_timeout = PULSE_TIMEOUT);

        void setMode(hlw8012_mode_t mode);
        hlw8012_mode_t getMode();
        hlw8012_mode_t toggleMode();

        uint32_t getCurrent(double mulitplier = 1.0);
        uint32_t getVoltage(double mulitplier = 1.0);
        uint32_t getActivePower(double mulitplier = 1.0);


        void setPulseTimeout(uint32_t pulse_timeout) { _pulse_timeout = pulse_timeout;}
        void setResistors(double current, double voltage_upstream, double voltage_downstream);

        void expectedCurrent(double current);
        void expectedVoltage(double current);
        void expectedActivePower(double power);

        uint32_t getCurrentMultiplier(double mulitplier = 1.0) { return mulitplier * _current_multiplier; };
        uint32_t getVoltageMultiplier(double mulitplier = 1.0) { return mulitplier * _voltage_multiplier; };
        uint32_t getPowerMultiplier(double mulitplier = 1.0) { return mulitplier * _power_multiplier; };

        void setCurrentMultiplier(double current_multiplier) { _current_multiplier = current_multiplier; };
        void setVoltageMultiplier(double voltage_multiplier) { _voltage_multiplier = voltage_multiplier; };
        void setPowerMultiplier(double power_multiplier) { _power_multiplier = power_multiplier; };
        void resetMultipliers();

    private:

        uint8_t _cf_pin;
        uint8_t _cf1_pin;
        uint8_t _sel_pin;

        double _current_resistor = R_CURRENT;
        double _voltage_resistor = R_VOLTAGE;

        double _current_multiplier; // Unit: us/A
        double _voltage_multiplier; // Unit: us/V
        double _power_multiplier;   // Unit: us/W

        uint32_t _pulse_timeout = PULSE_TIMEOUT;    //Unit: us
        volatile unsigned long _voltage_pulse_width = 0; //Unit: us
        volatile unsigned long _current_pulse_width = 0; //Unit: us
        volatile unsigned long _power_pulse_width = 0;   //Unit: us

        double _current = 0;
        double _voltage = 0;
        double _power = 0;

        uint8_t _current_mode = HIGH;
        volatile uint8_t _mode;

        void _calculateDefaultMultipliers();

};

#endif
