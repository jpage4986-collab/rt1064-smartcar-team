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
    assert seen['phase'] == 0 and seen['duty'] > 0 and ((88-angle)*direction >= 6 if direction else angle == 88), seen
    first = cpu.step(dark, 50)
    assert 0 < first['duty'] <= seen['duty'] and first['servo'] == angle, first
    for elapsed in range(50, 2101, 50):
        lost = cpu.step(dark, 50)
        assert lost['phase'] == 5 and lost['servo'] == angle, lost
        assert lost['duty'] == (first['duty'] if elapsed < 2000 else 0), (elapsed, lost)
    for _ in range(12):
        seen = cpu.step(lane, 50)
    angle = seen['servo']
    first = cpu.step(dark, 50)
    rebuilding = cpu.step(lane, 50)
    recovered = cpu.step(lane, 50)
    assert first['servo'] == rebuilding['servo'] == angle and first['duty'] > 0, first
    assert recovered['phase'] == 0 and recovered['duty'] > 0, recovered
    print(f"side={direction:+d}, held={angle}, search duty={first['duty']}, stopped at 2000 ms")



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--elf', type=Path, default=Path(__file__).resolve().parents[1]/'Project/GCC/wireless-app/tutorial.elf')
    args = parser.parse_args()
    for direction in (-1, 0, 1):
        test_side(args.elf, direction)
