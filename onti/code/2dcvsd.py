import struct
import binascii
inp = '\x79'

def str2hex(s):
    return binascii.hexlify(bytes(str.encode(s)))

print(int(str2hex(inp)))