# display_driver.py
import lvgl as lv
import gc9a01
import lcd_bus
from machine import SPI, Pin
from micropython import const

_WIDTH = const(240)
_HEIGHT = const(240)

_SPI_HOST = const(2)
_SPI_SCK = const(12)
_SPI_MOSI = const(11)
_SPI_MISO = const(-1)

_LCD_FREQ = const(80000000)
_LCD_DC = const(14)
_LCD_CS = const(10)
_LCD_RST = const(9)
_LCD_BACKLIGHT = const(21)

def init_display():
    # Initialize the SPI bus
    spi_bus = SPI.Bus(
        host=_SPI_HOST,
        mosi=_SPI_MOSI,
        miso=_SPI_MISO,
        sck=_SPI_SCK
    )

    # Initialize the display bus
    display_bus = lcd_bus.SPIBus(
        spi_bus=spi_bus,
        dc=_LCD_DC,
        cs=_LCD_CS,
        freq=_LCD_FREQ
    )

    # Initialize and return the GC9A01 display driver
    return gc9a01.GC9A01(
        data_bus=display_bus,
        display_width=_WIDTH,
        display_height=_HEIGHT,
        reset_pin=_LCD_RST,
        reset_state=gc9a01.STATE_LOW,
        power_on_state=gc9a01.STATE_HIGH,
        backlight_pin=_LCD_BACKLIGHT,
        backlight_on_state=gc9a01.STATE_LOW,
        offset_x=0,
        offset_y=0,
        color_space=lv.COLOR_FORMAT.RGB565,
        color_byte_order=gc9a01.BYTE_ORDER_BGR,
        rgb565_byte_swap=True
    )

