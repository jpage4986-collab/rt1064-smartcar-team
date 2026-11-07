"""Exercise sight loss and recovery using the same firmware as the full lap."""
from pathlib import Path
import argparse
from firmware import Firmware


def bend_frame(direction):
    pixels = bytearray()
    for row in range(120):
        center = 94 + direction * (18 + (119 - row) * 0.43)
        half_width = 13 + row * .48
        pixels.extend(205 if abs(col-center) <= half_width else 35 for col in range(188))
    return bytes(pixels)


def test_side(elf, direction):
    cpu = Firmware(elf)
    lane = bend_frame(direction)
    dark = bytes(188*120)
    for _ in range(12):
        seen = cpu.step(lane, 50)
    angle = seen['servo']
    assert seen['phase'] == 0 and seen['duty'] > 0 and (88-angle)*direction >= 6, seen
    for count in range(1, 5):
        lost = cpu.step(dark, 50)
        assert lost['duty'] == 0 and lost['phase'] == 5, lost
        assert lost['servo'] == (angle if count < 4 else 88), (count, lost)
    for _ in range(12):
        seen = cpu.step(lane, 50)
    angle = seen['servo']
    first = cpu.step(dark, 50)
    rebuilding = cpu.step(lane, 50)
    assert rebuilding['servo'] == angle and rebuilding['duty'] == 0, rebuilding
    recovered = cpu.step(lane, 50)
    assert first['servo'] == angle and first['duty'] == 0, first
    assert recovered['phase'] == 0 and recovered['duty'] > 0, recovered
    assert (88-recovered['servo'])*direction >= 6, recovered
    print(f"side={direction:+d}, held={angle}, neutral at 200 ms, recovered={recovered['servo']}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--elf', type=Path, default=Path(__file__).resolve().parents[1]/'Project/GCC/wireless-app/tutorial.elf')
    args = parser.parse_args()
    for direction in (-1, 1):
        test_side(args.elf, direction)
