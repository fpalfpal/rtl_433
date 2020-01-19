/*  FSK 10 byte Manchester encoded checksummed TPMS data.
 *  Works for Continental TPMS sensors find in Suzuki cars,
 *  but it can be found in other types, models.
 *  Based on tpms_citroen.c, by Ferenc Pal <fpalfpal@gmail.com>.
 *  Data decoding should be improved. E.g. G sensor data transmitted
 *  for tyre identification - auto learning function.
 *
 * Copyright (C) 2017 Christian W. Zuckschwerdt <zany@triq.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Packet nibbles:  UU  IIIIIIII FR  PP TT BB  CC
 * U = state, decoding unknown, seen e0,e1,e2,e3
 * I = id
 * F = flags, 
 * R = repeat counter (seen: 1,2,3 . 9,10.11)
 * P = Pressure (kPa in 1.364 steps, about fifth PSI?)
 * T = Temperature (deg C offset by 50)
 * B = Battery?
 * C = Checksum, CRC-8, poly>0x07, init:0xaa
 */

#include "decoder.h"

static const unsigned char preamble_pattern[2] = { 0x55, 0x56 };

static int tpms_continental_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    data_t *data;
    unsigned int start_pos;
    bitbuffer_t packet_bits = {0};
    uint8_t *b;
    int state;
    char state_str[3];
    unsigned id;
    char id_str[9];
    int flags;
    int repeat;
    int pressure;
    int temperature;
    int maybe_battery;
    int crc;

    bitbuffer_invert(bitbuffer);
    start_pos = bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 92);
    b = packet_bits.bb[0];

    if (b[6] == 0 || b[7] == 0) {
        return 0; // sanity check failed
    }

    if (crc8(b, 9, 0x07, 0xaa) !=  b[9])  {
        return 0; // bad checksum
    }

    state = b[0];
    sprintf(state_str, "%02x", state);
    id = (unsigned)b[1] << 24 | b[2] << 16 | b[3] << 8 | b[4];
    sprintf(id_str, "%08x", id);
    flags = b[5]>>4;
    repeat = b[5]&0x0f;
    pressure = b[6];
    temperature = b[7];
    maybe_battery = b[8];

    data = data_make(
        "model",        "",     DATA_STRING, "Continental",
        "type",         "",     DATA_STRING, "TPMS",
        "state",        "",     DATA_STRING, state_str,
        "id",           "",     DATA_STRING, id_str,
        "flags",        "",     DATA_INT, flags,
        "repeat",       "",     DATA_INT, repeat,
        "pressure_kPa", "Pressure",    DATA_FORMAT, "%.0f kPa", DATA_DOUBLE, (double)pressure * 1.364,
        "temperature_C", "Temperature", DATA_FORMAT, "%.0f C", DATA_DOUBLE, (double)temperature - 50.0,
//        "battery_mV",   "Battery", DATA_INT, battery_mV,
        "maybe_battery", "",     DATA_INT, maybe_battery,
        "mic",          "",     DATA_STRING, "CHECKSUM",
        NULL);

    decoder_output_data(decoder, data);
    return 1;
}

static int tpms_continental_callback(r_device *decoder, bitbuffer_t *bitbuffer) {
    unsigned bitpos = 0;
    int events = 0;

    // Find a preamble with enough bits after it that it could be a complete packet
    while ((bitpos = bitbuffer_search(bitbuffer, 0, bitpos, (uint8_t *)&preamble_pattern, 16)) + 178 <=
            bitbuffer->bits_per_row[0]) {
        events += tpms_continental_decode(decoder, bitbuffer, 0, bitpos + 16);
        bitpos += 2;
    }

    return events;
}

static char *output_fields[] = {
    "model",
    "type",
    "state",
    "id",
    "flags",
    "repeat",
    "pressure_kPa",
    "temperature_C",
//    "battery_mV",
    "maybe_battery",
    "code",
    "mic",
    NULL
};

r_device tpms_continental = {
    .name           = "Continental TPMS",
    .modulation     = FSK_PULSE_PCM,
    .short_width    = 52, // 12-13 samples @250k
    .long_width     = 52, // FSK
    .reset_limit    = 150, // Maximum gap size before End Of Message [us].
    .decode_fn      = &tpms_continental_callback,
    .disabled       = 0,
    .fields         = output_fields,
};
