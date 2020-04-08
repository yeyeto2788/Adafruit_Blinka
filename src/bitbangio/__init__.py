from ..adafruit_blinka.agnostic import detector

# Only import the new bitbang SPI if is a linux device
if detector.board.any_embedded_linux:
    from .bitbangio_generic import GenericSPI as SPI
# Otherwise, work as it was before.
else:
    from .bitbangio_base import I2C, SPI