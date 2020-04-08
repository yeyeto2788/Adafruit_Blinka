from .bitbangio_base import I2C, SPI
from ..adafruit_blinka.agnostic import detector

if detector.board.any_embedded_linux:
    from .bitbangio_generic import GenericSPI