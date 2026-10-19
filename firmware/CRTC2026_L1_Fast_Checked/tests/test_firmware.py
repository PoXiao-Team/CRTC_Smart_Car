import os, struct, json
from pathlib import Path
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0

root = Path(__file__).resolve().parents[1]
binary = root / 'build/CRTC2026_F103C8_DRV8833.elf'
with binary.open('rb') as f:
    elf = ELFFile(f)
    segments = [(s['p_vaddr'], s.data()) for s in elf.iter_segments() if s['p_type'] == 'PT_LOAD']
    symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}

u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
u.mem_map(0x08000000, 0x10000)
u.mem_map(0x20000000, 0x5000)
u.mem_map(0x40000000, 0x30000)
u.mem_map(0xE000E000, 0x2000)
for address, data in segments:
    u.mem_write(address, data)

def write32(address, value): u.mem_write(address, struct.pack('<I', value))
def inputs():
    return (list(struct.unpack('<4I',u.mem_read(0x40000434,16))),
            list(struct.unpack('<4I',u.mem_read(0x40012C34,16))))
def ccr():
    f,r=inputs();return [a+b for a,b in zip(f,r)]
def signed_pwm():
    f,r=inputs();return [a-b for a,b in zip(f,r)]
def state(name): return u.mem_read(symbols[name], 1)[0]
def call(name, arg=0):
    u.reg_write(UC_ARM_REG_SP, 0x20004F00)
    u.reg_write(UC_ARM_REG_LR, 0x0800FF01)
    u.reg_write(UC_ARM_REG_R0, arg)
    u.emu_start(symbols[name] | 1, 0x0800FF00, count=200000)
    assert u.reg_read(UC_ARM_REG_PC) == 0x0800FF00, 'Execution did not return'
    return u.reg_read(UC_ARM_REG_R0)

tick = 0
responses = []
def hook(uc, address, size, data):
    if address == (symbols['HAL_GetTick'] & ~1):
        uc.reg_write(UC_ARM_REG_R0, tick)
    elif address == (symbols['ps2_byte'] & ~1):
        assert responses, 'Unexpected PS2 transfer'
        uc.reg_write(UC_ARM_REG_R0, responses.pop(0))
    elif address in [(symbols['delay_us'] & ~1), (symbols['HAL_Delay'] & ~1), (symbols['ps2_configure'] & ~1)]:
        pass
    else:
        return
    uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))
main_hook_id = u.hook_add(UC_HOOK_CODE, hook)
write32(symbols['htim3'], 0x40000400)
write32(symbols['htim1'], 0x40012C00)
write32(symbols['htim4'], 0x40000800)
write32(0x4000042C, 999)
write32(0x40012C2C, 999)
write32(0x4000082C, 19999)

checks = []
def check(name, condition):
    assert condition, name
    checks.append(name)
def frame(buttons=0, lx=128, ly=128, rx=128, mode=0x73, marker=0x5A):
    global tick, responses
    tick = (tick + 20) & 0xFFFFFFFF
    raw = (~buttons) & 0xFFFF
    responses = [0xFF, mode, marker, raw & 255, raw >> 8]
    if mode in (0x73, 0x79, 0x53): responses += [rx, 128, lx, ly]
    if mode == 0x79: responses += [0] * 12
    call('Robot_Task')
    assert not responses, 'Frame length mismatch'
    forward,reverse=inputs()
    assert all(not(a and b) for a,b in zip(forward,reverse)), 'Both bridge inputs PWM simultaneously'
    assert all(0<=v<=750 for v in forward+reverse),'Motor pulse exceeds configured bounds'

# ELF placement. The compiler inlines axis in this version; check it through packets.
vector = struct.unpack('<II', u.mem_read(0x08000000, 8))
check('Vector stack is within 20KB SRAM', vector[0] == 0x20005000)
check('Reset vector is Thumb within 64KB Flash', bool(vector[1] & 1) and 0x08000000 <= vector[1] < 0x08010000)

START=1<<3; SELECT=1; R1=1<<11; L1=1<<10
UP=1<<4; RIGHT=1<<5; DOWN=1<<6; LEFT=1<<7
call('Robot_Stop')
check('Initial stop disarms and zeros all motor PWM', state('armed') == 0 and ccr() == [0]*4)
for _ in range(12): frame(R1|UP)
check('Valid drive packets cannot arm automatically', state('armed') == 0 and ccr() == [0]*4)
frame(START, rx=255)
check('START with off-center right joystick cannot arm', state('armed') == 0)
frame(); frame(START|UP)
check('START with held D-pad cannot arm', state('armed') == 0)
frame(); frame(START,lx=0,ly=255)
check('Left joystick position does not block neutral START', state('armed') == 1)
for lx,ly in [(0,0),(255,255),(0,255),(255,0)]:
    for _ in range(20): frame(R1,lx=lx,ly=ly)
    check(f'Left joystick ignored lx={lx} ly={ly}', signed_pwm()==[0]*4)
for _ in range(20): frame(UP)
check('D-pad without R1 cannot drive', signed_pwm()==[0]*4)
for _ in range(20): frame(R1|UP)
check('Reoriented forward uses four equal reverse outputs', signed_pwm()==[-500]*4)
frame()
check('Releasing R1 zeros outputs in one frame', ccr() == [0]*4)
for buttons,expected in [(RIGHT,[500,-500,-500,500]),(LEFT,[-500,500,500,-500]),
                         (DOWN,[500]*4), (UP|RIGHT,[0,-500,-500,0]),
                         (UP|LEFT,[-500,0,0,-500]), (DOWN|RIGHT,[500,0,0,500]),
                         (DOWN|LEFT,[0,500,500,0])]:
    for _ in range(20): frame(R1|buttons,lx=0,ly=0)
    check(f'D-pad direction buttons={buttons}', signed_pwm()==expected)
    frame()
for _ in range(20): frame(R1|UP|DOWN|LEFT|RIGHT)
check('Opposite D-pad inputs cancel both translation axes', signed_pwm()==[0]*4)
frame()
for _ in range(20): frame(R1, rx=255)
check('Right joystick clockwise remains unchanged', signed_pwm()==[500,-500,500,-500])
frame()
for _ in range(20): frame(R1, rx=0)
check('Right joystick counterclockwise remains unchanged', signed_pwm()==[-500,500,-500,500])
frame()
for raw in (110,128,146):
    for _ in range(20): frame(R1,rx=raw)
    check(f'Right joystick deadzone raw={raw}', signed_pwm()==[0]*4)
    frame()
for _ in range(20): frame(R1|UP, rx=255)
check('D-pad forward blocks full right-stick rotation', signed_pwm()==[-500]*4)
frame()
# Full stick excursions must never alter any of the eight D-pad movement targets.
for buttons,expected in [(UP,[-500]*4),(DOWN,[500]*4),
                         (RIGHT,[500,-500,-500,500]),(LEFT,[-500,500,500,-500]),
                         (UP|RIGHT,[0,-500,-500,0]),(UP|LEFT,[-500,0,0,-500]),
                         (DOWN|RIGHT,[500,0,0,500]),(DOWN|LEFT,[0,500,500,0])]:
    for raw in (0,255):
        for _ in range(30): frame(R1|buttons,rx=raw)
        check(f'Pure D-pad buttons={buttons} ignores right stick raw={raw}',signed_pwm()==expected)
        frame()
for _ in range(20): frame(R1|UP|DOWN|LEFT|RIGHT,rx=255)
check('Conflicting D-pad directions also block rotation',signed_pwm()==[0]*4)
frame()
for _ in range(20): frame(R1,rx=255)
check('Releasing D-pad restores right-stick rotation',signed_pwm()==[500,-500,500,-500])
frame()
for _ in range(20): frame(R1|L1|RIGHT)
check('Holding L1 raises right translation to 75-percent', signed_pwm()==[750,-750,-750,750])
frame()
for _ in range(20): frame(R1|L1|UP|RIGHT,rx=255)
check('Fast diagonal stays within 75-percent duty', max(ccr()) <= 750)
frame()
for buttons,expected in [(UP,[-750]*4),(DOWN,[750]*4),
                         (LEFT,[-750,750,750,-750]),(RIGHT,[750,-750,-750,750])]:
    for _ in range(30): frame(R1|L1|buttons)
    check(f'L1 fast D-pad buttons={buttons}',signed_pwm()==expected)
    frame()
for _ in range(30): frame(R1|L1,rx=255)
check('L1 fast right-stick rotation reaches 75-percent',signed_pwm()==[750,-750,750,-750])
for _ in range(30): frame(R1,rx=255)
check('Releasing L1 returns rotation to 50-percent',signed_pwm()==[500,-500,500,-500])
frame()
for _ in range(30): frame(R1|L1|UP)
for _ in range(30): frame(R1|UP)
check('Releasing L1 returns translation to 50-percent',signed_pwm()==[-500]*4)
frame()
for _ in range(30): frame(L1|UP)
check('L1 never bypasses R1 hold-to-run requirement',ccr()==[0]*4)

frame(SELECT)
check('SELECT disarms and zeros all motor PWM', state('armed') == 0 and ccr() == [0]*4)
for _ in range(12): frame()
frame(START)
for _ in range(20): frame(R1|UP)
frame(R1|UP,marker=0)
check('Malformed PS2 header disarms immediately', state('armed') == 0 and ccr() == [0]*4)
for _ in range(12): frame()
frame(START)
frame(mode=0x41)
check('Digital-only mode never drives analog commands', state('armed') == 0 and ccr() == [0]*4)
for _ in range(12): frame(mode=0x79)
frame(START,mode=0x79)
check('21-byte analog-pressure packet accepted after full read', state('armed') == 1)
# Gap greater than 100ms cannot retain enable.
tick += 101
frame(R1|UP)
check('Control scheduling gap greater than 100ms disarms', state('armed') == 0 and ccr() == [0]*4)
check('Stopped servo channels remain zero', struct.unpack('<II',u.mem_read(0x40000838,8)) == (0,0))

# Exercise actual compiled one-servo teach-and-cycle state machine.
L2=1<<8; R2=1<<9; TRIANGLE=1<<12; CIRCLE=1<<13; CROSS=1<<14; SQUARE=1<<15
status=symbols['g_robot_status']
def s16(offset): return struct.unpack('<H',u.mem_read(status+offset,2))[0]
def s8(offset): return u.mem_read(status+offset,1)[0]
for _ in range(12): frame()
frame(START); frame()
check('Armed idle outputs one centered servo pulse only', struct.unpack('<II',u.mem_read(0x40000838,8)) == (1500,0))
frame(TRIANGLE)
check('Automatic cycle rejected before teaching endpoints',s8(11)==0)
frame(L2)
check('L2 decreases pulse by 10us per 20ms frame',s16(0)==1490)
frame(R2)
check('R2 increases pulse by 10us per 20ms frame',s16(0)==1500)

for _ in range(180): frame(L2)
check('Manual decrement stops at new 600us lower bound',s16(0)==600)
for _ in range(40): frame(L2)
check('Holding L2 cannot exceed new lower bound',s16(0)==600)
frame(SQUARE); frame()
check('Square teaches down position',s16(2)==600 and s8(12)==1)
for _ in range(320): frame(R2)
check('Manual increment stops at unchanged 2000us upper bound',s16(0)==2000)
for _ in range(40): frame(R2)
check('Holding R2 cannot exceed new upper bound',s16(0)==2000)
frame(CIRCLE); frame()
check('Circle teaches dump position',s16(4)==2000 and s8(12)==3)
frame(R1|UP|TRIANGLE)
check('Cycle starts and blocks wheel output immediately',s8(11)==1 and ccr()==[0]*4)
frame(R1|UP)
check('Automatic move decreases pulse by 10us per frame',s16(0)==1990 and ccr()==[0]*4)

visited={s8(11)}
for i in range(1000):
 frame(R1|UP);visited.add(s8(11));assert ccr()==[0]*4
 if s8(11)==0: break
else:raise AssertionError('Cycle did not finish')
check('Cycle visits all move/wait states and returns down',visited==set(range(7)) and s16(0)==600)
for _ in range(20):frame(R1|UP)
check('Held R1 cannot restart chassis after shovel action',ccr()==[0]*4)
frame()
for _ in range(20):frame(R1|UP)
check('Release and re-press R1 restores chassis control',ccr()==[500]*4)
frame(TRIANGLE)
for _ in range(150):frame()
position=s16(0)
frame(CROSS)
check('Cross cancels cycle and holds current target pulse',s8(11)==0 and s16(0)==position)
frame(SELECT)
check('SELECT clears taught positions and disables servo pulse',s8(12)==0 and struct.unpack('<II',u.mem_read(0x40000838,8))==(0,0))
for _ in range(12):frame()
frame(START);frame()
frame(SQUARE);frame();frame(CIRCLE);frame();frame(TRIANGLE)
check('Coincident endpoints reject automatic cycle',s8(11)==0)
# Reversed pulse direction: down > dump works as well.
for _ in range(320):frame(R2)
frame(SQUARE);frame()
for _ in range(320):frame(L2)
frame(CIRCLE);frame();frame(TRIANGLE)
for _ in range(150):frame()
frame(marker=0)
check('Invalid frame aborts active cycle and clears endpoint teaching',s8(11)==0 and s8(12)==0 and state('armed')==0)
for _ in range(12):frame()
frame(START);frame()
for _ in range(320):frame(R2)
frame(SQUARE);frame()
for _ in range(320):frame(L2)
frame(CIRCLE);frame();frame(TRIANGLE)
for i in range(1000):
 frame()
 if s8(11)==0:break
check('Reversed taught direction returns correctly to 2000us',s8(11)==0 and s16(0)==2000)
frame(mode=0x53)
check('Green alternate button mode drained but rejected',state('armed')==0)
# Scheduling and pulse comparisons across HAL tick rollover.
tick=0xFFFFFFF0;write32(symbols['last_poll'],tick)
for _ in range(12):frame()
frame(START);frame()
check('Neutral enable works across millisecond tick rollover',state('armed')==1)
for _ in range(20):frame(R1|DOWN)
check('Reoriented backward uses the four TIM3 bridge inputs',inputs()==([500]*4,[0]*4))
frame(SELECT)
check('SELECT clears all eight bridge-input PWM channels',inputs()==([0]*4,[0]*4))
check('Both future servo outputs remain disabled',struct.unpack('<II',u.mem_read(0x4000083C,8))==(0,0))
# Fault injection: run actual microsecond wait with a stalled simulated TIM2.
u.hook_del(main_hook_id)
write32(symbols['htim2'],0x40000000)
write32(0x40000024,0)
write32(symbols['g_fault_tag'],0)
fault_entered=[]
def trap_error(uc,address,size,data):
    if address==(symbols['Error_Handler'] & ~1):
        fault_entered.append(True)
        uc.emu_stop()
trap_id=u.hook_add(UC_HOOK_CODE,trap_error)
u.reg_write(UC_ARM_REG_SP,0x20004F00)
u.reg_write(UC_ARM_REG_LR,0x0800FF01)
u.reg_write(UC_ARM_REG_R0,16)
u.emu_start(symbols['delay_us']|1,0x0800FF00,count=3000000)
check('Stalled TIM2 wait reaches error handling within bounded instructions',bool(fault_entered))
check('Stalled TIM2 wait records fault tag 2',struct.unpack('<I',u.mem_read(symbols['g_fault_tag'],4))[0]==2)
u.hook_del(trap_id)
report = {'result':'PASS','count':len(checks),'checks':checks,'method':'Actual ELF ARM Thumb instructions executed in Unicorn; PS2 bytes, tick, delays and PS2 configuration are substituted; RAM and timer registers simulated. No real receiver, GPIO electrical timing, motors, watchdog or mechanisms tested.'}
(root / 'tests/test_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
