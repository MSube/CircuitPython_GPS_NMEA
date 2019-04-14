"""
msube_gps - reads and parses NMEA input from a connected GPS
"""
__version__ = "0.5.0"

import time

class GPS:
    def __init__(self, uart, tz=None, debug=False):
        self._uart = uart
        self._tz = tz
        self._debug = debug

        self._buffer = bytearray(100)
        self._data = memoryview(self._buffer) # used for reading into the buffer
        self._next = 0
        self._started = 0   # when we received the first byte in the buffer
        self._received = 0  # when we received the first byte of the last message

        self._sentence = None
        self._complete = False
        self._date = None
        self._fix = None
        self._heading = None
        self._lat_lon = None
        self._speed = None
        self._time = None

    def update(self):
        if self._uart.in_waiting == 0: return False
        now = time.monotonic()
        end = min((self._next + self._uart.in_waiting), len(self._data))
        new = self._uart.readinto(self._data[self._next:end])
        if new == 0: return False # no data?

        data = None
        begin = 0 # first byte of next sentence in buffer
        index = self._next # first new byte in buffer
        self._next += new # first unused byte in buffer

        # search new data for \n, process sentence if found
        # if self._debug: print(self._buffer[index:self._next])
        for index, byte in enumerate(self._data[index:self._next], index):
            if byte == 36: # start of new message found
                begin = index
                self._started = now # adjust for length?
            if byte != 10: # sentence not complete yet
                continue
            # reached the end of the sentence
            sentence = bytes(self._data[begin:(index - 1)]) # creates a copy of the message
            received = self._started # time when this sentence started
            self._next -= (index + 1)
            if self._next: # move remaining data to front
                self._buffer[0:self._next] = self._data[(index + 1):(index + self._next)]
                self._started = now # adjust for length ? 1ms/byte?
            return self.parse(sentence, received)
        # sentence not compete yet
        if begin: # there is text from an incomplete sentence at the start of the buffer
            if self._debug: print('- {}'.format(self._data[0:begin]))
            self._buffer[0:(self._next - begin)] = self._data[begin:self._next]
        if self._next == len(self._buffer): # the buffer is full, start over again
            self._next = 0
        return False

    def parse(self, sentence, received):
        if self._debug: print('< {}'.format(sentence))
        if len(sentence) < 7: return False # too short
        if sentence[0] != 36: return False # does not start with '$'
        if sentence[-3] != 42: return False # no '*', no checksum
        xsum = int(sentence[-2:],16)
        sentence = sentence[1:-3]
        for byte in sentence: xsum ^= byte
        if xsum: return False # bad checksum
        self._sentence = sentence.split(b',')
        tag = self._sentence[0]
        if tag == b'GPGGA':
            self._complete = False
            self._time = self._sentence[1]
            self._fix = self._sentence[6]
            self._lat_lon = self._sentence[2:6]
            self._received = received
        elif tag == b'GPRMC':
            self._date = self._sentence[9]
            self._speed = self._sentence[7]
            self._heading = self._sentence[8]
            self._complete = (self._sentence[1] == self._time)
        return True

    def sendCmd(self, command, wait=True):
        self._uart.reset_input_buffer()
        xsum = 0
        for byte in command: xsum ^= byte
        xsum = b'*{:02X}'.format(xsum)
        if self._debug: print('> {}, {}'.format(command, xsum))
        self._uart.write(b'$' + command + xsum + b'\r\n')
        if not wait: return False
        while True:
            if self.update() and self._sentence[0] == b'PMTK001':
                return True

    def waitforReply(self):
        while True:
            if self.update() and self._sentence[0] == b'PMTK001':
                break

    @property
    def complete(self):
        ret, self._complete = self._complete, 0
        return ret

    @property
    def fix(self):
        return int(self._fix) if self._fix else False

    @property
    def date(self):
        return int(self._date) if self._date else 0

    @property
    def time(self):
        return float(self._time) if self._time else 0

    @property
    def datetime(self):
        t = int(float(self._time))
        d = int(self._date)
        t = time.mktime((2000 + d % 100, (d // 100) % 100, d // 10000,
                         t // 10000, (t // 100) % 100, t % 100,
                         0, 0, -1))
        for (s, o , n) in self._tz:
            if s < t:
                t += o
                break
        return time.localtime(t)

    @property
    def latitude(self):
        if not self._lat_lon or len(self._lat_lon[0]) < 3: return None
        lat = float(self._lat_lon[0]) * (1 if self._lat_lon[1] == b'N' else -1)
        return (lat//100) + (lat%100)/60

    @property
    def longitude(self):
        if not self._lat_lon or len(self._lat_lon[2]) < 3: return None
        lon = float(self._lat_lon[2]) * (1 if self._lat_lon[3] == b'E' else -1)
        return (lon//100) + (lon%100)/60