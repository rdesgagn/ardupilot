/*
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

#include "AP_RangeFinder_NMEA.h"

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

// return last value measured by sensor
bool AP_RangeFinder_NMEA::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from the lidar
    float sum = 0.0f;
    uint16_t count = 0;
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();
        if (decode(c)) {
            sum += _distance_m;
            count++;
        }
    }

    // return false on failure
    if (count == 0) {
        return false;
    }

    // return average of all measurements
    reading_cm = 100.0f * sum / count;
    return true;
}

// add a single character to the buffer and attempt to decode
// returns true if a complete sentence was successfully decoded
bool AP_RangeFinder_NMEA::decode(char c)
{
    switch (c) {
    case ',':
        // end of a term, add to checksum
        _checksum ^= c;
        FALLTHROUGH;
    case '\r':
    case '\n':
    case '*':
    {
        // null terminate and decode latest term
        _term[_term_offset] = 0;
        bool valid_sentence = decode_latest_term();

        // move onto next term
        _term_number++;
        _term_offset = 0;
        _term_is_checksum = (c == '*');
        return valid_sentence;
    }

    case '$': // sentence begin
        _sentence_type = SONAR_UNKNOWN;
        _term_number = 0;
        _term_offset = 0;
        _checksum = 0;
        _term_is_checksum = false;
        _distance_m = -1.0f;
        return false;
    }

    // ordinary characters are added to term
    if (_term_offset < sizeof(_term) - 1) {
        _term[_term_offset++] = c;
    }
    if (!_term_is_checksum) {
        _checksum ^= c;
    }

    return false;
}

// decode the most recently consumed term
// returns true if new sentence has just passed checksum test and is validated
bool AP_RangeFinder_NMEA::decode_latest_term()
{
    // handle the last term in a message
    if (_term_is_checksum) {
        uint8_t nibble_high = 0;
        uint8_t nibble_low  = 0;
        if (!hex_to_uint8(_term[0], nibble_high) || !hex_to_uint8(_term[1], nibble_low)) {
            return false;
        }
        const uint8_t checksum = (nibble_high << 4u) | nibble_low;
        return ((checksum == _checksum) &&
                !is_negative(_distance_m) &&
                (_sentence_type == SONAR_DBT || _sentence_type == SONAR_DPT));
    }

    // the first term determines the sentence type
    if (_term_number == 0) {
        // the first two letters of the NMEA term are the talker ID.
        // we accept any two characters here.
        if (_term[0] < 'A' || _term[0] > 'Z' ||
            _term[1] < 'A' || _term[1] > 'Z') {
            _sentence_type = SONAR_UNKNOWN;
            return false;
        }
        const char *term_type = &_term[2];
        if (strcmp(term_type, "DBT") == 0) {
            _sentence_type = SONAR_DBT;
        } else if (strcmp(term_type, "DPT") == 0) {
            _sentence_type = SONAR_DPT;
        } else {
            _sentence_type = SONAR_UNKNOWN;
        }
        return false;
    }

    if (_sentence_type == SONAR_DBT) {
        // parse DBT messages
        if (_term_number == 3) {
            _distance_m = strtof(_term, nullptr);
        }
    } else if (_sentence_type == SONAR_DPT) {
        // parse DPT messages
        if (_term_number == 1) {
            _distance_m = strtof(_term, NULL);
        }
    }

    return false;
}
