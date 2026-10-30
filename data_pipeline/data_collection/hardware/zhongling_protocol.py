"""Zhongling ASCII bus-servo command encoding.

Source: Hangzhou Zhongling servo handbook, section 1.4 (pages 25-27):
https://www.renrendoc.com/paper/99607126.html
The handbook documents P...T..., PULR (hold current position), and PDST.
Motion requires a successful position-mode query at runtime; no mode is changed.
"""

PROTOCOL_NAME = 'zhongling_ascii'
POSITION_MODE_RANGE_DEG = {1: 270., 2: 270., 3: 180., 4: 180.}


def servo_prefix(servo_id):
    if type(servo_id) is not int or not 0 <= servo_id <= 254:
        raise ValueError('众灵单舵机 ID 必须为 0..254；不允许广播 ID 255')
    return f'#{servo_id:03d}'


def read_position_command(servo_id):
    return servo_prefix(servo_id) + 'PRAD!'


def read_mode_command(servo_id):
    return servo_prefix(servo_id) + 'PMOD!'


def hold_command(servo_id):
    return servo_prefix(servo_id) + 'PULR!'


def stop_command(servo_id):
    return servo_prefix(servo_id) + 'PDST!'


def release_command(servo_id):
    return servo_prefix(servo_id) + 'PULK!'


def position_command(servo_id, position, duration_ms):
    prefix = servo_prefix(servo_id)
    if type(position) is not int or not 500 <= position <= 2500:
        raise ValueError('众灵位置指令 P 必须为 500..2500 的整数')
    if type(duration_ms) is not int or not 0 <= duration_ms <= 9999:
        raise ValueError('众灵位置指令 T 必须为 0..9999 毫秒的整数')
    return f'{prefix}P{position:04d}T{duration_ms:04d}!'
