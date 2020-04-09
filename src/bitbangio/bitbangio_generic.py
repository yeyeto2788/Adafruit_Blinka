import operator
from adafruit_blinka.agnostic import detector


class GenericSPI:
    """Software-based implementation of the SPI protocol over GPIO pins."""

    MSB = 0
    LSB = 1
    CPHA = 1
    CPOL = 2

    baudrate = 100000
    mode = 0
    bits = 8

    def __init__(self, portid):
        # Even that the port is never used, it is needed for
        # instantiation purposes and for following base object creation.
        self.port = portid

    def init(self, baudrate=100000, polarity=0, phase=0, bits=8,
                  firstbit=MSB, sck=None, mosi=None, miso=None, ce=None):
        """Initialize bit bang (or software) based SPI.

        Must provide a DigitalInOut class, the SPI clock, and optionally MOSI, MISO,
        and CE (chip enable) pin objects.

        If MOSI is set to None then writes will be disabled and fail with an error,
        likewise for MISO reads will be disabled.

        If CE is set to None then CE will not be asserted high/low by the library when
        transfering data.
        """
        mode = 0
        if polarity:
            mode |= self.CPOL

        if phase:
            mode |= self.CPHA

        self.baudrate = baudrate
        self.mode = mode
        self.bits = bits
        self.chip = detector.chip

        self.clock_pin = sck
        self.mosi_pin = mosi
        self.miso_pin = miso
        self.ce_pin = ce

        # Set pins as outputs/inputs.
        self.clock_pin.switch_to_output()

        if self.mosi_pin is not None:
            self.mosi_pin.switch_to_output()

        if self.miso_pin is not None:
            self.miso_pin.switch_to_output()

        if self.ce_pin is not None:
            self.ce_pin.switch_to_output()
            # Assert CE high to start with device communication off.
            self.ce_pin.value(True)
        # Assume mode 0.
        self.set_mode(0)
        # Assume most significant bit first order.
        self.set_bit_order(self.MSB)

    @property
    def frequency(self):
        return self.baudrate

    def set_clock_hz(self, hz):
        """Set the speed of the SPI clock.  This is unsupported with the bit
        bang SPI class and will be ignored.
        """
        pass

    def set_mode(self, mode):
        """Set SPI mode which controls clock polarity and phase.

        Should be a numeric value 0, 1, 2, or 3.  See wikipedia page for details on meaning:
        http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
        """
        if mode < 0 or mode > 3:
            raise ValueError('Mode must be a value 0, 1, 2, or 3.')
        if mode & 0x02:
            # Clock is normally high in mode 2 and 3.
            self._clock_base = True
        else:
            # Clock is normally low in mode 0 and 1.
            self._clock_base = False
        if mode & 0x01:
            # Read on trailing edge in mode 1 and 3.
            self._read_leading = False
        else:
            # Read on leading edge in mode 0 and 2.
            self._read_leading = True
        # Put clock into its base state.
        self.clock_pin.value(self._clock_base)

    def set_bit_order(self, order):
        """Set order of bits to be read/written over serial lines.  Should be
        either MSBFIRST for most-significant first, or LSBFIRST for
        least-signifcant first.
        """
        # Set self._mask to the bitmask which points at the appropriate bit to
        # read or write, and appropriate left/right shift operator function for
        # reading/writing.
        if order == self.MSB:
            self._mask = 0x80
            self._write_shift = operator.lshift
            self._read_shift = operator.rshift
        elif order == self.LSB:
            self._mask = 0x01
            self._write_shift = operator.rshift
            self._read_shift = operator.lshift
        else:
            raise ValueError('Order must be MSBFIRST or LSBFIRST.')

    def close(self):
        """Close the SPI connection."""
        if self.mosi_pin is not None:
            self.mosi_pin.deinit()

        if self.miso_pin is not None:
            self.mosi_pin.deinit()

        if self.clock_pin is not None:
            self.clock_pin.deinit()

        if self.ce_pin is not None:
            self.ce_pin.deinit()

    def write(self, buf, start=0, end=None):
        """Half-duplex SPI write.

        The CE line will be asserted low, the specified bytes will be clocked out
        the MOSI line and the CE line will be assert high back again.
        """
        # Fail MOSI is not specified.
        if self.mosi_pin is None:
            raise RuntimeError('Write attempted with no MOSI pin specified.')

        if self.ce_pin is not None:
            self.ce_pin.value(False)

        for byte in buf[start:end]:

            for bit in range(8):
                # Write bit to MOSI.
                if self._write_shift(byte, bit) & self._mask:
                    self.mosi_pin.value(True)
                else:
                    self.mosi_pin.value(False)
                # Flip clock off base.
                self.clock_pin.value(not self._clock_base)
                # Return clock to base.
                self.clock_pin.value(self._clock_base)
        if self.ce_pin is not None:
            self.ce_pin.value(True)

    def read(self, length, assert_ss=True, deassert_ss=True):
        """Half-duplex SPI read.

        If assert_ss is true, the CE line will be
        asserted low, the specified length of bytes will be clocked in the MISO
        line, and if deassert_ss is true the CE line will be put back high.
        Bytes which are read will be returned as a bytearray object.
        """
        if self.miso_pin is None:
            raise RuntimeError('Read attempted with no MISO pin specified.')
        if assert_ss and self.ce_pin is not None:
            self.ce_pin.value(False)
        result = bytearray(length)
        for i in range(length):
            for j in range(8):
                # Flip clock off base.
                self.clock_pin.value(not self._clock_base)
                # Handle read on leading edge of clock.
                if self._read_leading:
                    if self.miso_pin.value:
                        # Set bit to 1 at appropriate location.
                        result[i] |= self._read_shift(self._mask, j)
                    else:
                        # Set bit to 0 at appropriate location.
                        result[i] &= ~self._read_shift(self._mask, j)
                # Return clock to base.
                self.clock_pin.value(self._clock_base)
                # Handle read on trailing edge of clock.
                if not self._read_leading:
                    if self.miso_pin.value:
                        # Set bit to 1 at appropriate location.
                        result[i] |= self._read_shift(self._mask, j)
                    else:
                        # Set bit to 0 at appropriate location.
                        result[i] &= ~self._read_shift(self._mask, j)
        if deassert_ss and self.ce_pin is not None:
            self.ce_pin.value(True)
        return result

    def transfer(self, data):
        """Full-duplex SPI read and write.

        The CE line will be asserted low, the specified bytes will be clocked
        out the MOSI line while bytes will also be read from the MISO line,
        and the CE line will be put back high.

        Bytes which are read will be returned as a bytearray object.
        """
        if self.mosi_pin is None:
            raise RuntimeError('Write attempted with no MOSI pin specified.')

        if self.miso_pin is None:
            raise RuntimeError('Read attempted with no MISO pin specified.')

        if self.ce_pin is not None:
            self.ce_pin.value(False)
        result = bytearray(len(data))

        for byte_iteration in range(len(data)):
            for bit in range(8):
                # Write bit to MOSI.
                if self._write_shift(data[byte_iteration], bit) & self._mask:
                    self.mosi_pin.value(True)
                else:
                    self.mosi_pin.value(False)
                # Flip clock off base.
                self.clock_pin.value(not self._clock_base)
                # Handle read on leading edge of clock.
                if self._read_leading:
                    if self.miso_pin.value:
                        # Set bit to 1 at appropriate location.
                        result[byte_iteration] |= self._read_shift(self._mask, bit)
                    else:
                        # Set bit to 0 at appropriate location.
                        result[byte_iteration] &= ~self._read_shift(self._mask, bit)
                # Return clock to base.
                self.clock_pin.value(self._clock_base)
                # Handle read on trailing edge of clock.
                if not self._read_leading:
                    if self.miso_pin.value:
                        # Set bit to 1 at appropriate location.
                        result[byte_iteration] |= self._read_shift(self._mask, bit)
                    else:
                        # Set bit to 0 at appropriate location.
                        result[byte_iteration] &= ~self._read_shift(self._mask, bit)
        if self.ce_pin is not None:
            self.ce_pin.value(True)
        return result