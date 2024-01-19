##
# This file is part of the libsigrokdecode project.
##
# Copyright (C) 2024 Matteo Bonora <bonora.matteo@gmail.com>
##
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
##
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
##
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd
from enum import Enum


class SamplerateError(Exception):
    pass


class Ann:
    FAST_FIELD_SYNC, FAST_FIELD_STATUS, FAST_FIELD_DATA, FAST_FIELD_CRC, FAST_FIELD_ERROR, \
        FAST_DATA, FAST_STATUS, FAST_CRC,  \
        SLOW_ID, SLOW_DATA, SLOW_CRC, SLOW_ERROR,  \
        SLOW_FRAME, SLOW_FRAME_ERROR = \
        range(14)


class Decoder(srd.Decoder):
    api_version = 3
    id = 'sent'
    name = 'SENT'
    longname = 'Single Edge Nibble Transmission'
    desc = 'PWM-based one-way communication protocol'
    license = 'gplv3+'
    inputs = ['logic']
    outputs = []
    tags = ['Automotive']
    channels = (
        {'id': 'data', 'name': 'Data', 'desc': 'Data line'},
    )
    options = (
        {'id': 'format', 'desc': 'Data format', 'default': 'hex',
         'values': ('dec', 'hex', 'oct', 'bin')},
    )
    annotations = (
        # fast fields
        ('ff_sync', 'Sync'),
        ('ff_status', 'Status'),
        ('ff_data', 'Data'),
        ('ff_crc', 'CRC'),
        ('ff_error', 'Error'),
        # fast nibbles
        ('f_data', 'Data'),
        ('f_status', 'Status'),
        ('f_crc', 'CRC'),
        # slow data
        ('s_id', 'ID'),
        ('s_data', 'Data'),
        ('s_crc', 'CRC'),
        ('s_crc_e', 'CRC Error'),
        # slow frame
        ('s_frame', 'Frame'),
        ('s_frame_error', 'Frame Error')
    )
    annotation_rows = (
        ('fields', 'Fast Fields', (Ann.FAST_FIELD_SYNC, Ann.FAST_FIELD_STATUS, Ann.FAST_FIELD_DATA, Ann.FAST_FIELD_CRC, Ann.FAST_FIELD_ERROR,)),
        ('fast_nibbles', 'Fast Nibbles', (Ann.FAST_DATA, Ann.FAST_STATUS, Ann.FAST_CRC)),
        ('slow_data', 'Slow Data', (Ann.SLOW_ID, Ann.SLOW_DATA, Ann.SLOW_CRC, Ann.SLOW_ERROR)),
        ('slow_frame', 'Slow Frames', (Ann.SLOW_FRAME, Ann.SLOW_FRAME_ERROR,)),
    )

    sent = None
    captures = [0] * 5  # history of samplenums

    def __init__(self):
        self.sent = SENT(self)
        self.reset()

    def reset(self):
        self.samplerate = None
        self.captures = [0]*5
        self.ss_block = self.es_block = None

        self.sent.reset()

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def format_value(self, v, bits):
        fmt = self.options['format']
        if fmt == 'dec':
            return '{:d}'.format(v)

        if fmt == 'hex':
            digits = (bits + 4 - 1) // 4
            fmtchar = 'X'
        elif fmt == 'oct':
            digits = (bits + 3 - 1) // 3
            fmtchar = 'o'
        elif fmt == 'bin':
            digits = bits
            fmtchar = 'b'
        else:
            fmtchar = None
        if fmtchar is not None:
            fmt = '{{:0{:d}{:s}}}'.format(digits, fmtchar)
            return fmt.format(v)

        return None

    def putx(self, data):
        self.put(self.ss_block, self.es_block, self.out_ann, data)

    def crc4(self, data, num_nibbles):
        # CRC table for poly = 0x1D

        crc4_table = [0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5]
        checksum16 = 5
        for i in range(0, num_nibbles):
            checksum16 = data[i] ^ crc4_table[checksum16]

        # checksum with an extra 0 value
        checksum16 = 0 ^ crc4_table[checksum16]
        # expected result for 748748: Checksum16=3
        return checksum16

    def crc6(self, data):
        crc6_table = [0, 25, 50, 43, 61, 36, 15, 22, 35, 58, 17, 8, 30, 7, 44, 53,
                      31, 6, 45, 52, 34, 59, 16, 9, 60, 37, 14, 23, 1, 24, 51, 42,
                      62, 39, 12, 21, 3, 26, 49, 40, 29, 4, 47, 54, 32, 57, 18, 11,
                      33, 56, 19, 10, 28, 5, 46, 55, 2, 27, 48, 41, 63, 38, 13, 20]

        checksum64 = 21  # initialize checksum
        for i in range(0, 4):
            checksum64 = data[i] ^ crc6_table[checksum64]
        # checksum with an extra 0 value (message is augmented by six zeros)
        checksum64 = 0 ^ crc6_table[checksum64]
        return checksum64

    def advance(self, edge):
        """Advance decoder by one edge while updating 'captures' list."""
        self.wait({0: edge})
        self.captures[4] = self.captures[3]
        self.captures[3] = self.captures[2]
        self.captures[2] = self.captures[1]
        self.captures[1] = self.captures[0]
        self.captures[0] = self.samplenum

    def decode(self):
        if not self.samplerate:
            raise SamplerateError('Cannot decode without samplerate.')

        self.advance('f')
        self.advance('r')
        self.advance('f')
        while True:
            self.advance('r')
            self.advance('f')
            self.sent.fsm()


class SENT():
    class State(Enum):
        SYNC, STATUS, DATA = range(3)

    tick_duration = 0  # duration of tick period. updated after every sync pulse
    frame = [0] * 6  # data of a SENT frame. Used to compute CRC
    nibble_i = 0  # nibble index
    state = State.SYNC  # decoder state
    data_start = 0

    def __init__(self, decoder):
        self.decoder = decoder
        self.slow = SlowFrame(self.decoder)

    def time_to_human(self, time):
        if time < 1e-6:
            time_s = '%.2f ns' % (time / 1e-9)
        elif time < 1e-3:
            time_s = '%.2f μs' % (time / 1e-6)
        elif time < 1e0:
            time_s = '%.2f ms' % (time / 1e-3)
        else:
            time_s = '%.2f s' % (time)
        return time_s

    def nibble_duration(self, start_f, end_f):
        return round(((end_f - start_f) / self.decoder.samplerate) / self.tick_duration)

    def is_nibble(self, start_f, mid_r, end_f):
        if (mid_r - start_f) / self.decoder.samplerate > (self.tick_duration * 4) * .8:
            nibble_duration = self.nibble_duration(start_f, end_f)
            if 12 <= nibble_duration <= 27:
                return nibble_duration

        return 0

    def reset(self):
        self.nibble_i = 0
        self.state = self.State.SYNC
        self.slow.reset()

    def fsm(self):
        match self.state:
            case self.State.SYNC:  # waiting for sync pulse
                # 1 tick = sync_pulse / 56
                self.tick_duration = float((self.decoder.captures[2] - self.decoder.captures[4]) / self.decoder.samplerate) / 56

                if 3e-6 * .8 <= self.tick_duration <= 90e-6 * 1.2:
                    # if tick is within 3 to 90us +-20%
                    if self.is_nibble(self.decoder.captures[2], self.decoder.captures[1], self.decoder.captures[0]):
                        # if next pulse is a valid nibble then this is a sync pulse
                        self.decoder.ss_block = self.decoder.captures[4]
                        self.decoder.es_block = self.decoder.captures[2]
                        self.decoder.putx([Ann.FAST_FIELD_SYNC, [f'Sync, Tick: {self.time_to_human(self.tick_duration)}']])
                        self.state = self.State.STATUS

            case self.State.STATUS:
                nibble_value = self.nibble_duration(self.decoder.captures[4], self.decoder.captures[2]) - 12

                # Run slow frame decoder
                self.slow.fsm(nibble_value)

                self.decoder.ss_block = self.decoder.captures[4]
                self.decoder.es_block = self.decoder.captures[2]
                self.decoder.putx([Ann.FAST_FIELD_STATUS, ['Status']])
                self.decoder.putx([Ann.FAST_STATUS, [self.decoder.format_value(nibble_value, 4)]])

                self.data_start = self.decoder.captures[2]
                self.state = self.State.DATA

            case self.State.DATA:
                if nibble_duration := self.is_nibble(self.decoder.captures[4], self.decoder.captures[3], self.decoder.captures[2]):
                    nibble_value = nibble_duration - 12

                    if self.nibble_i == 6 or (self.nibble_i == 4 and not self.is_nibble(self.decoder.captures[2], self.decoder.captures[1], self.decoder.captures[0])):
                        # check whether we are looking at the CRC nibble

                        # end the data block
                        self.decoder.ss_block = self.data_start
                        self.decoder.es_block = self.decoder.captures[4]
                        msg = ', '.join(map(lambda x: self.decoder.format_value(x, 4), self.frame[:self.nibble_i]))
                        self.decoder.putx([Ann.FAST_FIELD_DATA, [f'Data: [ {msg} ]']])

                        self.decoder.ss_block = self.decoder.captures[4]
                        self.decoder.es_block = self.decoder.captures[2]
                        crc_val = self.decoder.format_value(nibble_value, 4)
                        if (crc := self.decoder.crc4(self.frame[0:self.nibble_i], self.nibble_i)) == nibble_value:
                            self.decoder.putx([Ann.FAST_FIELD_CRC, ['CRC']])
                            self.decoder.putx([Ann.FAST_CRC, [crc_val]])
                        else:
                            self.decoder.putx([Ann.FAST_FIELD_ERROR, ['CRC']])
                            self.decoder.putx([Ann.FAST_CRC, [crc_val]])
                            self.slow.abort()  # abort any slow channel messages

                        # end of frame
                        self.nibble_i = 0
                        self.state = self.State.SYNC

                    else:
                        # this is a data nibble
                        self.frame[self.nibble_i] = nibble_value
                        self.decoder.ss_block = self.decoder.captures[4]
                        self.decoder.es_block = self.decoder.captures[2]
                        self.decoder.putx([Ann.FAST_DATA, [self.decoder.format_value(nibble_value, 4)]])

                        self.nibble_i += 1

                else:
                    # invalid nibble. report error and go back to sync
                    self.decoder.ss_block = self.decoder.captures[4]
                    self.decoder.es_block = self.decoder.captures[2]
                    self.decoder.putx([Ann.FAST_FIELD_ERROR, ['Error']])
                    self.slow.abort()  # abort any slow channel messages
                    self.nibble_i = 0
                    self.state = self.State.SYNC


class SlowFrame():
    class State(Enum):
        IDLE, RX, SHORT, ENHANCED = range(4)

    state = State.IDLE
    bit_i = 0  # bit index
    frame_buf2 = 0
    frame_buf3 = 0
    first_block_start = 0  # to keep track of when various blocks start
    second_block_start = 0
    third_block_start = 0
    enhanced_data_length = 12

    def __init__(self, decoder):
        self.decoder = decoder

    def abort(self):
        """ Abort current slow channel frame """
        if self.state != self.State.IDLE:
            self.decoder.ss_block = self.first_block_start
            self.decoder.es_block = self.decoder.captures[2]
            self.decoder.putx([Ann.SLOW_FRAME_ERROR, ['Abort']])
        self.reset()

    def reset(self):
        self.frame_buf2 = 0
        self.frame_buf3 = 0
        self.bit_i = 0
        self.state = self.State.IDLE

    def fsm(self, status_nibble_value):
        if self.state == self.State.IDLE:
            if (status_nibble_value >> 3) & 1:
                # if 3rd bit of status nibble is 1, start from scratch
                self.frame_buf2 = 0
                self.frame_buf3 = 0
                self.bit_i = 0
                self.first_block_start = self.decoder.captures[4]
                self.state = self.State.RX
        # can jump straight to next state

        # push bit2 in frame buffer
        self.frame_buf2 = (self.frame_buf2 << 1) | ((status_nibble_value >> 2) & 1)
        self.frame_buf3 = (self.frame_buf3 << 1) | ((status_nibble_value >> 3) & 1)

        if self.state == self.State.RX:  # reading second status nibble to check if enhanced
            if self.bit_i == 1:
                if self.frame_buf3 & 1:
                    # if we see bit 3 high for a second time -> enhanced
                    self.state = self.State.ENHANCED
                else:
                    self.state = self.State.SHORT
        # can jump straight to next state

        if self.state == self.State.SHORT:
            self.decoder.ss_block = self.decoder.captures[4]
            self.decoder.es_block = self.decoder.captures[2]

            match self.bit_i:
                case 3:  # end of ID
                    self.decoder.ss_block = self.first_block_start
                    self.decoder.es_block = self.decoder.captures[2]
                    self.decoder.putx([Ann.SLOW_ID, [self.decoder.format_value(self.frame_buf2 & 7, 4)]])
                case 4:  # start of data
                    self.second_block_start = self.decoder.captures[4]
                case 11:  # end of data
                    self.decoder.ss_block = self.second_block_start
                    self.decoder.es_block = self.decoder.captures[2]
                    self.decoder.putx([Ann.SLOW_DATA, [self.decoder.format_value(self.frame_buf2 & 255, 8)]])
                case 12:  # start of crc
                    self.third_block_start = self.decoder.captures[4]
                case 15:  # end of frame
                    self.decoder.ss_block = self.third_block_start
                    self.decoder.es_block = self.decoder.captures[2]

                    crc_ok = False
                    data = self.frame_buf2 >> 4
                    if self.decoder.crc4([(data >> 8) & 15, (data >> 4) & 15, data & 15], 3) == self.frame_buf2 & 15:
                        crc_ok = True

                    self.decoder.putx([Ann.SLOW_CRC if crc_ok else Ann.SLOW_ERROR,
                                       [self.decoder.format_value(self.frame_buf2 & 7, 4)]])

                    self.decoder.ss_block = self.first_block_start
                    self.decoder.es_block = self.decoder.captures[2]
                    self.decoder.putx([Ann.SLOW_FRAME if crc_ok else Ann.SLOW_FRAME_ERROR,
                                       [f'Short, ID: {self.decoder.format_value(self.frame_buf2>>(self.bit_i-3), 4)}, '
                                        f'Data: {self.decoder.format_value((self.frame_buf2>>4 )& 255, 8)}, '
                                        f'CRC: {"OK" if crc_ok else "ERROR"}']])
                    self.state = self.State.IDLE

        elif self.state is self.State.ENHANCED:
            self.decoder.ss_block = self.decoder.captures[4]
            self.decoder.es_block = self.decoder.captures[2]

            if self.bit_i <= 5 and not self.frame_buf3 & 1:
                self.decoder.putx([Ann.SLOW_ERROR, ['Err']])
                self.abort()

            # if 6 <= self.bit_i <= 17:
            #    # compose the weird CRC sequence needed by the standard
            #    i = self.bit_i - 5
            #    self.crc_data[(i-1) * 2] = self.frame_buf2 & 1
            #    self.crc_data[(i-1) * 2+1] = self.frame_buf3 & 1

            if self.bit_i in [6, 12, 17] and self.frame_buf3 & 1:
                # check that bits 7, 13 and 18 of 3rd bit are 0
                self.decoder.putx([Ann.SLOW_ERROR, ['Not 0']])
                self.abort()

            match self.bit_i:
                case 5:  # end of crc
                    self.crc_end = self.decoder.captures[2]

                case 6:  # start of data
                    if self.frame_buf3 & 1:
                        self.decoder.putx([Ann.SLOW_ERROR, ['Start seq.']])
                        self.abort()
                    self.second_block_start = self.decoder.captures[4]

                case 7:
                    if self.frame_buf3 & 1:
                        self.enhanced_data_length = 16
                    else:
                        self.enhanced_data_length = 12

                case 17:  # end of data
                    crc = (self.frame_buf2 >> 12) & 63
                    id = (self.frame_buf3 >> 2) & 240
                    id_len = 8
                    data = self.frame_buf2 & 4095
                    if self.enhanced_data_length == 16:
                        id >>= 4
                        id_len = 4
                        data |= ((self.frame_buf2 & 30) << 11)
                    else:
                        id |= (self.frame_buf3 >> 1) & 15

                    crc_data = [0] * 4
                    for i in range(4):
                        for j in range(3):
                            crc_data[i] = (crc_data[i] << 1) | ((self.frame_buf2 >> (11 - i*3 - j)) & 1)
                            crc_data[i] = (crc_data[i] << 1) | ((self.frame_buf3 >> (11 - i*3 - j)) & 1)

                    crc_ok = False
                    if self.decoder.crc6(crc_data) == (self.frame_buf2 >> 12) & 63:
                        crc_ok = True

                    self.decoder.ss_block = self.first_block_start
                    self.decoder.es_block = self.crc_end
                    self.decoder.putx([Ann.SLOW_CRC if crc_ok else Ann.SLOW_ERROR, [self.decoder.format_value(crc, 6)]])

                    self.decoder.ss_block = self.second_block_start
                    self.decoder.es_block = self.decoder.captures[2]
                    self.decoder.putx([Ann.SLOW_DATA,
                                       [f'ID: {self.decoder.format_value(id,id_len)}, Data: {self.decoder.format_value(data,self.enhanced_data_length)}']])

                    self.decoder.ss_block = self.first_block_start
                    self.decoder.es_block = self.decoder.captures[2]
                    self.decoder.putx([Ann.SLOW_FRAME if crc_ok else Ann.SLOW_FRAME_ERROR,
                                       [f'Enhanced {self.enhanced_data_length}, ID: {self.decoder.format_value(id,id_len)}, '
                                        f'Data: [ {self.decoder.format_value(data, self.enhanced_data_length)} ], '
                                        f'CRC: {"OK" if crc_ok else "ERROR"}']])
                    self.state = self.State.IDLE

        self.bit_i += 1
