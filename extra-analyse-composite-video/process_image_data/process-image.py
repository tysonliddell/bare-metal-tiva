import io
import numpy as np
import serial
from typing import Generator
from xmodem import XMODEM

import draw

# BAUD_RATE = 115200
BAUD_RATE = 1500000
NUM_SAMPLES_PER_SCANLINE = 60
NUM_SCANLINES = 300
VERTICAL_SCALE = 1
HORIZONTAL_SCALE = 8
XMODEM_DATA_PADDING_BYTE = 0x1A
XMODEM_BLOCK_SIZE = 128
USB_DEVICE = '/dev/cu.usbmodem0E234C151'

def iter_scanlines(data: bytes) -> Generator[bytes, None, None]:
    for scanline_start in range(0, len(data), NUM_SAMPLES_PER_SCANLINE):
        # print("SCANLINE")
        yield data[scanline_start:][:NUM_SAMPLES_PER_SCANLINE]

def print_data(data: bytes):
    for scanline in iter_scanlines(data):
        # print(*scanline, sep=",")
        for value in scanline:
            print(f"{value:<4}", end='')
        print("")

def pgm_out(data: bytes):
    print("P2")
    print(f"{NUM_SAMPLES_PER_SCANLINE * HORIZONTAL_SCALE} 300")
    print("255")
    for scanline in iter_scanlines(data):
        # print(*scanline, sep=" ")
        for value in scanline:
            for _ in range(HORIZONTAL_SCALE):
                print(f"{value} ", end='')
        print("")

def trim_xmodem_padding(data: bytes):
    """
    Remove trailing XMODEM padding characters from data.
    """
    data = bytearray(data)
    padding_len = len(data) % NUM_SAMPLES_PER_SCANLINE
    del data[-padding_len:]

    while data[-NUM_SAMPLES_PER_SCANLINE:].count(XMODEM_DATA_PADDING_BYTE) == NUM_SAMPLES_PER_SCANLINE:
        del data[-NUM_SAMPLES_PER_SCANLINE:]

    return bytes(data)

def scale_field(field_data: bytes, vertical_scale: int, horizontal_scale: int):
    result = bytearray()
    for scanline in iter_scanlines(field_data):
        horizontally_stretched = [val for val in scanline for _ in range(horizontal_scale)]
        result.extend(horizontally_stretched * vertical_scale)
    return result

def watch_video():
    ser = serial.Serial(USB_DEVICE, timeout=1, write_timeout=1, baudrate=BAUD_RATE)
    def getc(size, timeout=1):
        return ser.read(size) or None

    def putc(data, timeout=1):
        return ser.write(data)

    modem = XMODEM(getc, putc)

    def read_next_field():
        stream = io.BytesIO()
        modem.recv(stream, crc_mode=1)
        stream.seek(0)
        data = stream.read()
        data = trim_xmodem_padding(data)
        return data[:NUM_SAMPLES_PER_SCANLINE*NUM_SCANLINES]

    def frames():
        while True:
            field = read_next_field()
            scaled_field = scale_field(field, VERTICAL_SCALE, HORIZONTAL_SCALE)
            yield np.frombuffer(scaled_field, dtype=np.uint8)

    draw.animate_frames(
        NUM_SCANLINES*VERTICAL_SCALE,
        NUM_SAMPLES_PER_SCANLINE*HORIZONTAL_SCALE,
        frames(),
    )

def main():
    watch_video()

if __name__ == '__main__':
    main()
