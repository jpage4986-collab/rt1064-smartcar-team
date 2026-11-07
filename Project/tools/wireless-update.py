"""RT1064 wireless updater. Run with uv run --with pyserial --with pyelftools.

GCC wireless application ELF only. Requires SU7 BOOT 2 / UPDATE=2. Never writes loader.
Adapted from SU7Ultra scripts/wireless-update.py (GPL-3.0-or-later).
"""
import argparse
import struct
import time
import zlib
import re
import binascii
import json
from pathlib import Path

APP_BASE = 0x70020000
APP_LIMIT = 0x1E0000
VECTOR_OFFSET = 0x2000


def load_image(path):
    from elftools.elf.elffile import ELFFile
    with path.open('rb') as source:
        elf = ELFFile(source)
        segments = [(s['p_paddr'], s.data()) for s in elf.iter_segments()
                    if s['p_type'] == 'PT_LOAD' and s['p_filesz']]
    if not segments or min(address for address, _ in segments) != APP_BASE:
        raise ValueError('Expected an application linked at 0x70020000; loader/legacy images are rejected.')
    end = max(address + len(data) for address, data in segments)
    if end > APP_BASE + APP_LIMIT or any(address < APP_BASE for address, _ in segments):
        raise ValueError('Image exceeds the reserved application slot.')
    image = bytearray(b'\xff' * (end - APP_BASE))
    for address, data in segments:
        image[address - APP_BASE:address - APP_BASE + len(data)] = data
    validate_image(image)
    return bytes(image)


def validate_image(image):
    if not VECTOR_OFFSET + 8 < len(image) <= APP_LIMIT:
        raise ValueError('Invalid image size')
    stack, entry = struct.unpack_from('<II', image, VECTOR_OFFSET)
    if not (0x20000000 <= stack <= 0x20070000 and not stack & 7 and
            APP_BASE + VECTOR_OFFSET <= entry < APP_BASE + len(image) and entry & 1):
        raise ValueError('Invalid application vector table')


def crc(data):
    return zlib.crc32(data) & 0xffffffff


class Link:
    def __init__(self, port):
        import serial
        self.serial = serial.Serial()
        self.serial.port = port
        self.serial.baudrate = 115200
        self.serial.timeout = 0.2
        self.serial.write_timeout = 2
        self.serial.dtr = False
        self.serial.rts = False
        self.serial.open()
        self.buffer = bytearray()
        self.data_retries = 0

    def close(self):
        self.serial.close()

    def send(self, command):
        data = command.encode('ascii') + b'\r\n'
        # Pace frames for the wireless USB bridge; target ACK provides stop/wait.
        width = 24 if command.startswith('DATA ') else 1
        for start in range(0, len(data), width):
            self.serial.write(data[start:start + width])
            self.serial.flush()
            time.sleep(0.004 if width == 24 else 0.03)

    def lines_until(self, accept, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            data = self.serial.read(self.serial.in_waiting or 1)
            self.buffer.extend(data)
            while b'\n' in self.buffer:
                line, _, remaining = self.buffer.partition(b'\n')
                self.buffer = bytearray(remaining)
                text = line.decode('ascii', errors='replace').strip()
                if text.startswith('ERR '):
                    raise RuntimeError(text)
                if accept(text):
                    return text
            if len(self.buffer) > 2048:
                self.buffer.clear()
        return None

    def request(self, command, accept, timeout=3, attempts=5):
        for attempt in range(attempts):
            self.send(command)
            try:
                reply = self.lines_until(accept, timeout)
            except RuntimeError as error:
                transient = str(error) in {'ERR CRC', 'ERR COMMAND', 'ERR PACKET', 'ERR HEX', 'ERR LINE', 'ERR LENGTH'}
                # DATA and read-only HELLO are safe to retry after line damage.
                if not (command.startswith('DATA ') or command == 'HELLO') or not transient or attempt + 1 == attempts:
                    raise
                if command.startswith('DATA '):
                    self.data_retries = getattr(self, 'data_retries', 0) + 1
                print(f'Retry {command.split()[0]} after {error}', flush=True)
                time.sleep(.05)
                self.serial.reset_input_buffer()
                self.buffer.clear()
                continue
            if reply is not None:
                return reply
            if command.startswith('DATA '):
                self.data_retries = getattr(self, 'data_retries', 0) + 1
        raise TimeoutError(f'No reply to {command.split()[0]}; check mainboard power and wireless pairing.')

    def hello(self):
        def valid_identity(text):
            return bool(re.fullmatch(r'SU7 BOOT 2 (READY|UART8 115200)', text) or
                        re.fullmatch(r'SU7 TRACK\d+ CAMERA=\d+ FRAMES=\d+ CAMCFG=\d+ ARM=[01] DUTY=\d+( UPDATE=2)?', text))
        return self.request('HELLO', valid_identity)

    def enter(self):
        identity = self.hello()
        print(identity, flush=True)
        if identity.startswith('SU7 TRACK'):
            if not identity.endswith(' UPDATE=2'):
                raise RuntimeError('Current firmware has no wireless upgrade entry. Install BOOT 2 once using CMSIS-DAP.')
            self.request('ENTER', lambda text: text == 'OK ENTER')
            time.sleep(1)
            identity = self.hello()
            print(identity, flush=True)
        if not identity.startswith('SU7 BOOT '):
            raise RuntimeError('Device did not enter the update loader')

    def capabilities(self):
        try:
            return self.request('CAPS', lambda text: text.startswith('OK CAPS '), attempts=1)
        except RuntimeError as error:
            if str(error) == 'ERR COMMAND':
                return ''  # Original BOOT2 remains usable over its text protocol.
            raise

    def control(self, accepted, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.buffer:
                value = self.buffer.pop(0)
            else:
                data = self.serial.read(1)
                if not data:
                    continue
                value = data[0]
            if value == 0x18:
                raise RuntimeError('YMODEM cancelled by target; image was not committed')
            if value in accepted:
                return value
        raise TimeoutError('YMODEM response timed out')

    def ymodem(self, image, test_only=False):
        self.send(f'YMODEM TEST {len(image)}' if test_only else 'YMODEM')
        self.control({ord('C')})
        started = time.monotonic()

        def packet(number, payload, width):
            body = payload.ljust(width, b'\x1a')
            return bytes((1 if width == 128 else 2, number, number ^ 255)) + body + struct.pack('>H', binascii.crc_hqx(body, 0))

        def exchange(frame, wanted=(6,)):
            for attempt in range(10):
                self.serial.write(frame)
                self.serial.flush()
                try:
                    result = self.control({6, 0x15}, timeout=3)
                    if result in wanted:
                        return result
                except TimeoutError:
                    pass
                self.data_retries += 1
                print(f'YMODEM retry {attempt + 1}', flush=True)
            raise RuntimeError('YMODEM retry limit reached; image was not committed')

        header = b'firmware.bin\0' + str(len(image)).encode('ascii') + b'\0'
        exchange(packet(0, header.ljust(128, b'\0'), 128))
        self.control({ord('C')})
        last_report = -1
        for offset in range(0, len(image), 1024):
            exchange(packet((offset // 1024 + 1) & 255, image[offset:offset + 1024], 1024))
            done = min(offset + 1024, len(image))
            progress = done * 100 // len(image)
            if progress // 10 != last_report:
                elapsed = time.monotonic() - started
                rate = done / max(.001, elapsed)
                print(f'YMODEM {progress}% ({done}/{len(image)}), {rate:.0f} B/s', flush=True)
                print(f'PROGRESS {5 + progress * 90 // 100} 100', flush=True)
                last_report = progress // 10
        # Standard two-EOT close, followed by the empty batch header.
        if exchange(b'\x04', wanted=(0x15, 6)) == 0x15:
            exchange(b'\x04')
        self.control({ord('C')})
        exchange(packet(0, bytes(128), 128))
        if self.lines_until(lambda text: text == f'OK YMODEM {len(image)}', 3) is None:
            raise RuntimeError('YMODEM completion reply missing; image was not committed')
        elapsed = time.monotonic() - started
        print(f'YMODEM_OK bytes={len(image)} seconds={elapsed:.3f} Bps={len(image)/elapsed:.0f} retries={self.data_retries} test_only={test_only}', flush=True)
        return elapsed

    def benchmark(self, size):
        self.enter()
        if 'YMODEM-1K' not in self.capabilities():
            raise RuntimeError('Install the YMODEM-capable BOOT2 via DAP first')
        image = (bytes(range(256)) * ((size + 255) // 256))[:size]
        elapsed = self.ymodem(image, test_only=True)
        return dict(bytes=size, seconds=round(elapsed, 3), bytes_per_second=round(size/elapsed),
                    retries=self.data_retries, flash_written=False, baudrate=115200)

    def upload(self, image, expected_algorithm=None, protocol='auto'):
        total_started = time.monotonic()
        print('STAGE 进入升级', flush=True)
        self.enter()
        enter_seconds = time.monotonic()-total_started
        print(f'TIMING 进入升级 {enter_seconds:.3f}', flush=True)
        caps = self.capabilities()
        use_ymodem = protocol != 'legacy' and 'YMODEM-1K' in caps
        if protocol == 'ymodem' and not use_ymodem:
            raise RuntimeError('Target has the original BOOT2; install the new loader via DAP first')
        print('Protocol: ' + ('YMODEM-1K' if use_ymodem else 'legacy text (original BOOT2)'), flush=True)
        checksum = crc(image)
        reply = self.request(f'BEGIN {len(image)} {checksum:08x}', lambda text: text.startswith('OK BEGIN '))
        offset = int(reply.split()[-1])
        if offset > len(image) or (offset != len(image) and offset % 256):
            raise RuntimeError('Invalid resume offset')
        print('STAGE 写入', flush=True)
        started = time.monotonic()
        if use_ymodem:
            self.ymodem(image)
            offset = len(image)
        last_report = -1
        while offset < len(image):
            block = image[offset:offset + 256]
            expected = offset + len(block)
            self.request(f'DATA {offset} {block.hex()} {crc(block):08x}',
                         lambda text: text == f'OK DATA {expected}')
            offset = expected
            progress = 100 * offset // len(image)
            if progress // 10 != last_report:
                elapsed = time.monotonic() - started
                rate = max(1, (offset - int(reply.split()[-1])) / max(.001, elapsed))
                eta = (len(image) - offset) / rate
                print(f'Transfer {progress}% ({offset}/{len(image)}), {rate:.0f} B/s, ETA {eta:.0f}s', flush=True)
                print(f'PROGRESS {5 + progress * 90 // 100} 100', flush=True)
                last_report = progress // 10
        transfer_seconds = time.monotonic() - started
        print(f'TIMING 写入 {transfer_seconds:.3f}', flush=True)
        print('STAGE 校验安装', flush=True)
        commit_started = time.monotonic()
        self.request('COMMIT', lambda text: text == 'OK COMMIT', timeout=60, attempts=2)
        commit_seconds = time.monotonic() - commit_started
        print(f'Flash verified: {len(image)} bytes, CRC32 {checksum:08x}, {time.monotonic()-started:.1f}s, DATA retries {self.data_retries}', flush=True)
        print(f'TIMING 校验安装 {commit_seconds:.3f}', flush=True)
        print('STAGE 启动检查', flush=True)
        restart_started = time.monotonic()
        self.request('RUN', lambda text: text == 'OK RUN')
        time.sleep(1)
        identity = self.hello()
        print(identity, flush=True)
        if not identity.startswith('SU7 TRACK') or not identity.endswith(' UPDATE=2'):
            raise RuntimeError('Flash CRC passed, but updated application did not start.')
        if expected_algorithm and not identity.startswith(f'SU7 TRACK{expected_algorithm} '):
            raise RuntimeError('Application algorithm does not match selected preset')
        restart_seconds = time.monotonic()-restart_started
        print(f'TIMING 启动检查 {restart_seconds:.3f}', flush=True)
        print(f'UPDATE_OK total {time.monotonic()-total_started:.1f}s', flush=True)
        print('PROGRESS 100 100', flush=True)
        return dict(timings={'进入升级':round(enter_seconds,3),'写入':round(transfer_seconds,3),'校验安装':round(commit_seconds,3),'启动检查':round(restart_seconds,3)}, bytes=len(image), crc32=f'{checksum:08x}', protocol='ymodem' if use_ymodem else 'legacy',
                    total_seconds=round(time.monotonic()-total_started, 3), retries=self.data_retries,
                    transfer_seconds=round(transfer_seconds, 3), commit_seconds=round(commit_seconds, 3),
                    application=identity, baudrate=115200)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--image', type=Path)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--benchmark', type=int, metavar='BYTES', help='YMODEM throughput only; never writes Flash; stays in loader')
    parser.add_argument('--protocol', choices=('auto', 'ymodem', 'legacy'), default='auto')
    parser.add_argument('--report', type=Path)
    parser.add_argument('--expect-algorithm', type=int, choices=(1, 3, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17))
    args = parser.parse_args()
    if sum((bool(args.image), args.check, args.benchmark is not None)) != 1:
        parser.error('Choose --image, --check or --benchmark')
    if args.benchmark is not None and not 1 <= args.benchmark <= APP_LIMIT:
        parser.error('Benchmark size must fit the application slot')
    image = load_image(args.image) if args.image else None
    link = Link(args.port)
    try:
        result = None
        if image is not None:
            result = link.upload(image, args.expect_algorithm, args.protocol)
        elif args.benchmark is not None:
            result = link.benchmark(args.benchmark)
        else:
            print(link.hello(), flush=True)
        if args.report and result:
            args.report.write_text(json.dumps(result, indent=2), encoding='utf-8')
    finally:
        link.close()


if __name__ == '__main__':
    main()
