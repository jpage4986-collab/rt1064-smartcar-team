"""Run the compiled Cortex-M7 perception and motor control for one camera frame."""
import struct
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB
from unicorn.arm_const import (UC_CPU_ARM_CORTEX_M7, UC_ARM_REG_FPEXC, UC_ARM_REG_SP,
    UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1)

class Firmware:
    def __init__(self, path):
        self.cpu=Uc(UC_ARCH_ARM,UC_MODE_THUMB)
        self.cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M7)
        for base,size in [(0x20000000,0x80000),(0x70000000,0x400000),(0x80000000,0x2000000),(0x10000000,4096)]:
            self.cpu.mem_map(base,size)
        with path.open('rb') as stream:
            elf=ELFFile(stream)
            self.symbol={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
            for seg in elf.iter_segments():
                if seg['p_type']=='PT_LOAD' and seg['p_filesz']:
                    self.cpu.mem_write(seg['p_vaddr'],seg.data())
        self.cpu.reg_write(UC_ARM_REG_FPEXC,0x40000000)
        for name in ('gpio_set','pwm_duty'):
            self.cpu.mem_write(self.symbol[name]&~1,b'\x70\x47')
        self.call('track_mode16_reset')

    def call(self,name,*args):
        cpu=self.cpu
        cpu.reg_write(UC_ARM_REG_SP,0x2006f000)
        cpu.reg_write(UC_ARM_REG_LR,0x10000001)
        for reg,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1),args):
            cpu.reg_write(reg,value)
        cpu.emu_start(self.symbol[name]|1,0x10000000,count=200000000)
        if cpu.reg_read(UC_ARM_REG_PC)!=0x10000000:
            raise RuntimeError(f'{name} did not return')

    def value(self,name):
        return struct.unpack('<i',self.cpu.mem_read(self.symbol[name],4))[0]

    def step(self,frame,dt_ms=100):
        if len(frame)!=188*120:raise ValueError('expected a 188 x 120 grayscale frame')
        self.cpu.mem_write(0x81000000,frame)
        self.call('track_mode16_set_frame_dt',dt_ms)
        self.call('track_mode16_process',0x81006000,0x81000000)
        self.call('track_control',0x81006000)
        return dict(lap=self.value('track16_lap'),phase=self.value('track_mode16_phase'),
                    servo=self.value('track_servo_angle'),duty=self.value('track_motor_duty'),
                    obstacle=self.value('track_mode16_obstacle_side'))
