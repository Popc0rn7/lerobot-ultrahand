import ctypes

def decode_position(pos: int) -> int:
    return ctypes.c_int32(pos).value

def encode_position(pos: int) -> int:
    return pos

def encode_current(current: int) -> int:
    return current

def decode_current(current: int) -> int:
    return current