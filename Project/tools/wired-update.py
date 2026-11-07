"""DAP update using the same BOOT2 application layout as wireless updates."""
import argparse
from contextlib import ExitStack
import subprocess
import tempfile
from datetime import datetime
import hashlib
import io
import json
from pathlib import Path
import runpy
import struct
import time
import zlib
from elftools.elf.elffile import ELFFile
from intelhex import IntelHex
from pyocd.core.helpers import ConnectHelper
from pyocd.flash.file_programmer import FileProgrammer
from backup_paths import record_backup


def elf_data(path):
    with path.open('rb') as f:
        elf = ELFFile(f)
        segments = [(int(s['p_paddr']), bytes(s.data())) for s in elf.iter_segments()
                    if s['p_type'] == 'PT_LOAD' and s['p_filesz']]
        symbols = {s.name: int(s['st_value']) for s in elf.get_section_by_name('.symtab').iter_symbols()}
    return segments, symbols


def record(source, destination, purpose):
    record_backup(source, destination, '目标板Flash原始字节', purpose)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--elf', type=Path, required=True)
    p.add_argument('--loader', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--expect-algorithm', type=int, choices=(1, 3, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17))
    p.add_argument('--flash-script', type=Path, help='PowerShell HEX flasher for the selected RT1064 board')
    args = p.parse_args()
    started = time.monotonic()
    timings = {}
    api = runpy.run_path(str(Path(__file__).with_name('wireless-update.py')))
    image = api['load_image'](args.elf)
    loader, loader_symbols = elf_data(args.loader)
    if (not loader or min(a for a, _ in loader) != 0x70000000 or
            any(a < 0x70000000 or a + len(d) > 0x70020000 for a, d in loader) or
            not 0x70002400 <= loader_symbols.get('su7_boot_dispatch', 0) < 0x70020000):
        raise ValueError('Invalid BOOT2 loader')
    _, symbols = elf_data(args.elf)
    names = ('track_heartbeat', 'track_arm_request', 'track_motor_duty', 'track_mode_number',
             'camera_test_frames', 'camera_test_stage', 'track_servo_angle')
    if not all(n in symbols for n in names):
        raise ValueError('Application runtime symbols missing')
    names += tuple(n for n in ('track_mode16_phase', 'track_mode16_island_side',
                   'track_mode16_island_bottom', 'track_mode16_target_row',
                   'track_target_x', 'track_target_confidence', 'track_valid_rows') if n in symbols)
    backup_dir = args.output_dir / ('pre-wired-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    backup_dir.mkdir(parents=True, exist_ok=False)
    def connect():
        session = ConnectHelper.session_with_chosen_probe(
            target_override='mimxrt1064', connect_mode='under-reset', frequency=800000,
            options={'resume_on_disconnect': False})
        if session is None:
            raise RuntimeError('Connect CMSIS-DAP for wired flashing.')
        session.open()
        return session

    with ExitStack() as cleanup:
        session = connect()
        cleanup.callback(session.close)
        target = session.target
        target.halt()
        if (target.read32(0xE000ED00) >> 4) & 0xFFF != 0xC27:
            raise RuntimeError('Target is not Cortex-M7')
        try:
            print('STAGE 备份', flush=True)
            stage_started = time.monotonic()
            loader_changed = any(bytes(target.read_memory_block8(a, len(d))) != d for a, d in loader)
            rounded = (len(image) + 0xFFFF) & ~0xFFFF
            regions = [(0x70020000, rounded), (0x70200000, rounded), (0x703F0000, 0x10000)]
            if loader_changed:
                loader_end = max(a + len(d) for a, d in loader)
                regions.insert(0, (0x70000000, (loader_end - 0x70000000 + 0xFFFF) & ~0xFFFF))
            metadata = None
            backups = []
            for address, size in regions:
                data = bytearray()
                for off in range(0, size, 0x8000):
                    data.extend(target.read_memory_block8(address + off, min(0x8000, size - off)))
                path = backup_dir / f'{address:08x}.bin'
                path.write_bytes(data)
                record(f'RT1064 {address:#x}–{address+size-1:#x}', path.resolve(), '有线升级前保留全部将覆盖的64KiB扇区')
                backups.append(dict(address=hex(address), size=size, path=str(path.resolve()),
                                    sha256=hashlib.sha256(data).hexdigest()))
                print(f'BACKUP_OK {address:#x} {size} bytes', flush=True)
                if address == 0x703F0000:
                    metadata = data
            timings['备份'] = round(time.monotonic()-stage_started, 3)
            print(f"TIMING 备份 {timings['备份']:.3f}", flush=True)
            assert metadata is not None
            header = struct.pack('<4I', 0x37555053, 2, len(image), zlib.crc32(image))
            metadata[0xF000:0xF014] = header + struct.pack('<I', zlib.crc32(header))
            writes = [(0x70020000, image), (0x70200000, image), (0x703F0000, bytes(metadata))]
            if loader_changed:
                writes = loader + writes
            packed = IntelHex()
            for address, data in writes:
                packed.frombytes(data, offset=address)
            stream = io.StringIO()
            packed.write_hex_file(stream)
            stream.seek(0)
            print('PROGRESS 45 100', flush=True)
            print('STAGE 写入', flush=True)
            stage_started = time.monotonic()
            if args.flash_script:
                flasher = args.flash_script.resolve(strict=True)
                target = None
                session.close()
                with tempfile.TemporaryDirectory() as temp:
                    folder = Path(temp)
                    hex_path = folder / 'boot2-update.hex'
                    hex_path.write_text(stream.getvalue(), encoding='ascii')
                    (folder / 'pyocd.yaml').write_text(
                        'connect_mode: under-reset\nfrequency: 800000\nchip_erase: sector\n'
                        'resume_on_disconnect: false\n', encoding='ascii')
                    subprocess.run(['pwsh', '-NoProfile', '-File', str(flasher), '-Image', str(hex_path)],
                                   cwd=folder, check=True)
                session = connect()
                cleanup.callback(session.close)
                target = session.target
                target.halt()
            else:
                FileProgrammer(session, chip_erase='sector').program(stream, file_format='hex')
            timings['写入'] = round(time.monotonic()-stage_started, 3)
            print(f"TIMING 写入 {timings['写入']:.3f}", flush=True)
            print('STAGE 校验', flush=True)
            stage_started = time.monotonic()
            # Reset the FlexSPI/core caches before verification. A halted core can
            # otherwise expose pre-programming cache lines even in a new session.
            # Force the resident loader if the core briefly executes during reset.
            target.write32(0x400D4064, 0x41736166)
            target.write32(0x400D404C, 8)
            target.write32(0x400D4100, 0x3755424C)
            target.reset_and_halt()
            target.get_target_context().invalidate()
            for address, expected in writes + ([] if loader_changed else loader):
                if bytes(target.read_memory_block8(address, len(expected))) != expected:
                    raise RuntimeError(f'Flash readback mismatch at {address:#x}')
            print('FLASH_READBACK_OK loader/application/staging/manifest', flush=True)
            timings['校验'] = round(time.monotonic()-stage_started, 3)
            print(f"TIMING 校验 {timings['校验']:.3f}", flush=True)
            print('STAGE 启动检查', flush=True)
            stage_started = time.monotonic()
            # All paths boot through BOOT2; keep its ENTER flag clear.
            target.write32(0x400D4064, 0x41736166)
            target.write32(0x400D404C, 8)
            target.write32(0x400D4100, 0)
            target.reset()
            target.resume()
            time.sleep(3)
            def sample():
                target.halt()
                return {n: target.read32(symbols[n]) for n in names}
            first = sample()
            target.resume()
            time.sleep(.5)
            second = sample()
            if args.expect_algorithm and second['track_mode_number'] != args.expect_algorithm:
                raise RuntimeError('Application algorithm does not match selected preset')
            if (first['track_heartbeat'] == second['track_heartbeat'] or
                    second['track_arm_request'] not in (0, 1) or second['track_servo_angle'] > 180 or
                    second['track_motor_duty'] > 50000):
                raise RuntimeError('Application did not pass runtime verification')
            timings['启动检查'] = round(time.monotonic()-stage_started, 3)
            print(f"TIMING 启动检查 {timings['启动检查']:.3f}", flush=True)
            report = dict(timings=timings, elf=str(args.elf.resolve()), backups=backups, first=first, second=second,
                          elf_sha256=hashlib.sha256(args.elf.read_bytes()).hexdigest(),
                          bytes=len(image), crc32=f'{zlib.crc32(image):08x}',
                          elapsed_seconds=round(time.monotonic()-started, 1))
            (args.output_dir / 'latest-wired-update.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
            print('WIRED_UPDATE_OK ' + json.dumps(report), flush=True)
            print('PROGRESS 100 100', flush=True)
        finally:
            if target is not None:
                target.resume()


if __name__ == '__main__':
    main()
