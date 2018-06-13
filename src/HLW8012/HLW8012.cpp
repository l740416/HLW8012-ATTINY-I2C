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

#include <Arduino.h>
#include "HLW8012.h"

void HLW8012::begin(
    uint8_t cf_pin,
    uint8_t cf1_pin,
    uint8_t sel_pin,
    uint8_t currentWhen,
    uint32_t pulse_timeout
    ) {

    _cf_pin = cf_pin;
    _cf1_pin = cf1_pin;
    _sel_pin = sel_pin;
    _current_mode = currentWhen;
    _pulse_timeout = pulse_timeout;

    pinMode(_cf_pin, INPUT_PULLUP);
    pinMode(_cf1_pin, INPUT_PULLUP);
    pinMode(_sel_pin, OUTPUT);

    _calculateDefaultMultipliers();

    _mode = _current_mode;
    digitalWrite(_sel_pin, _mode);
}

void HLW8012::setMode(hlw8012_mode_t mode) {
    _mode = (mode == MODE_CURRENT) ? _current_mode : 1 - _current_mode;
    digitalWrite(_sel_pin, _mode);
}

hlw8012_mode_t HLW8012::getMode() {
    return (_mode == _current_mode) ? MODE_CURRENT : MODE_VOLTAGE;
}

hlw8012_mode_t HLW8012::toggleMode() {
    hlw8012_mode_t new_mode = getMode() == MODE_CURRENT ? MODE_VOLTAGE : MODE_CURRENT;
    setMode(new_mode);
    return new_mode;
}

uint32_t HLW8012::getCurrent(double mulitplier) {

    // Power measurements are more sensitive to switch offs,
    // so we first check if power is 0 to set _current to 0 too
    if (_power == 0) {
        _current_pulse_width = 0;
    } else if (_mode == _current_mode) {
        _current_pulse_width = pulseIn(_cf1_pin, HIGH, _pulse_timeout);
    }

		_current_pulse_width = pulseIn(_cf1_pin, HIGH, _pulse_timeout);
    _current = (_current_pulse_width > 0) ? _current_multiplier / _current_pulse_width / 2 : 0;
    return _current * mulitplier;

}

uint32_t HLW8012::getVoltage(double mulitplier) {

    if (_mode != _current_mode) {
        _voltage_pulse_width = pulseIn(_cf1_pin, HIGH, _pulse_timeout);
    }

    _voltage_pulse_width = pulseIn(_cf1_pin, HIGH, _pulse_timeout);
    _voltage = (_voltage_pulse_width > 0) ? _voltage_multiplier / _voltage_pulse_width / 2 : 0;
    return _voltage * mulitplier;
}

uint32_t HLW8012::getActivePower(double mulitplier) {
    _power_pulse_width = pulseIn(_cf_pin, HIGH, _pulse_timeout);
    _power = (_power_pulse_width > 0) ? _power_multiplier / _power_pulse_width / 2 : 0;
    return _power * mulitplier;
}

void HLW8012::expectedCurrent(double value) {
    if (_current == 0) getCurrent();
    if (_current > 0) _current_multiplier *= (value / _current);
}

void HLW8012::expectedVoltage(double value) {
    if (_voltage == 0) getVoltage();
    if (_voltage > 0) _voltage_multiplier *= (value / _voltage);
}

void HLW8012::expectedActivePower(double value) {
    if (_power == 0) getActivePower();
    if (_power > 0) _power_multiplier *= (value / _power);
}

void HLW8012::resetMultipliers() {
    _calculateDefaultMultipliers();
}

void HLW8012::setResistors(double current, double voltage_upstream, double voltage_downstream) {
    if (voltage_downstream > 0) {
        _current_resistor = current;
        _voltage_resistor = (voltage_upstream + voltage_downstream) / voltage_downstream;
        _calculateDefaultMultipliers();
    }
}

// These are the multipliers for current, voltage and power as per datasheet
// These values divided by output period (in useconds) give the actual value
// For power a frequency of 1Hz means around 12W
// For current a frequency of 1Hz means around 15mA
// For voltage a frequency of 1Hz means around 0.5V
void HLW8012::_calculateDefaultMultipliers() {
    _current_multiplier = ( 1000000.0 * 512 * V_REF / _current_resistor / 24.0 / F_OSC );
    _voltage_multiplier = ( 1000000.0 * 512 * V_REF * _voltage_resistor / 2.0 / F_OSC );
    _power_multiplier = ( 1000000.0 * 128 * V_REF * V_REF * _voltage_resistor / _current_resistor / 48.0 / F_OSC );
}
