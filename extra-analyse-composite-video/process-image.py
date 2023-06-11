from typing import Generator
import sys


NUM_SAMPLES_PER_SCANLINE = 30
HORIZONTAL_SCALE = 8


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


def main():
    data = sys.stdin.buffer.read()  # read data from STDIN until EOF
    # print_data(data)
    pgm_out(data)


if __name__ == '__main__':
    main()
