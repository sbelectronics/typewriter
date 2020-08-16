# http://www.smbaker.com//
#
# Generates mapping of keyboard scancodes to midi notes and channels 

import sys
from ps2_scancode_list import *

def find_ps2_code(i):
    for ps2_code in ps2_codes:
        if ps2_code[0]==i:
            return ps2_code
    return None


def find_ps2_extcode(i):
    for ps2_code in ps2_extended_codes:
        if ps2_code[0]==i:
            return ps2_code
    return None

synonyms = {RALT: LALT,
            LGUI: KPMULT,   # finish
            RGUI: KPMINUS}   # go

scancode_list = []
shifted_scancode_list = []
caps_scancode_list = []
caps_shifted_scancode_list = []
for i in range(0, 128):
    ps2_code = find_ps2_code(i)
    if not ps2_code:
        print >> sys.stderr, "Failed to find ps2 code for", i
        scancode_list.append(0x00)
        shifted_scancode_list.append(0x00)
        caps_scancode_list.append(0x00)
        caps_shifted_scancode_list.append(0x00)
        continue

    scancode_list.append(ord(ps2_code[1]))
    shifted_scancode_list.append(ord(ps2_code[2]))

    if ((ps2_code[1] >= "a") and (ps2_code[1]<="z")):
        caps_scancode_list.append(ord(ps2_code[2]))
        caps_shifted_scancode_list.append(ord(ps2_code[1]))
    else:
        caps_scancode_list.append(ord(ps2_code[1]))
        caps_shifted_scancode_list.append(ord(ps2_code[2]))

extcode_list = []
shifted_extcode_list = []
for i in range(0, 128):
    ps2_code = find_ps2_extcode(i)
    if not ps2_code:
        print >> sys.stderr, "Failed to find ps2 extcode for", i
        extcode_list.append(0x00)
        shifted_extcode_list.append(0x00)
        continue

    extcode_list.append(ord(ps2_code[1]))
    shifted_extcode_list.append(ord(ps2_code[2]))

 
print "const unsigned char scancode_map[128] PROGMEM = {%s};" % ",".join([str(x) for x in scancode_list])
print "const unsigned char shifted_scancode_map[128] PROGMEM = {%s};" % ",".join([str(x) for x in shifted_scancode_list])
print "const unsigned char caps_scancode_map[128] PROGMEM = {%s};" % ",".join([str(x) for x in caps_scancode_list])
print "const unsigned char caps_shifted_scancode_map[128] PROGMEM = {%s};" % ",".join([str(x) for x in caps_shifted_scancode_list])
print "const unsigned char extcode_map[128] PROGMEM = {%s};" % ",".join([str(x) for x in extcode_list]) 
print "const unsigned char shifted_extcode_map[128] PROGMEM = {%s};" % ",".join([str(x) for x in shifted_extcode_list]) 
