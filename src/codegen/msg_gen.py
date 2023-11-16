#!/bin/env python3
filepath = "Megasquirt_CAN_Broadcast.csv"
file = open(filepath,'r')
lines = file.readlines()

def print_group(group, attrs):
    print("struct RtMsg%02d_t" % group)
    print("{")
    print("  uint8_t data[8];")
    for attr in attrs:
        print("")
        CTYPE_LUT = {1:'int8_t',2:'int16_t',4:'int32_t',8:'int64_t'}
        name = attr['name']
        offset = attr['offset_in_group']
        size = attr['size']
        ctype = ('' if attr['signed'] else 'u') + CTYPE_LUT[size]
        print("  // %s" % (attr['description']))
        if attr['units'] != '-':
            print("  // units: %s" % (attr['units']))
        mult = attr['mult']
        div = attr['div']
        val_getter = ""
        if size == 1:
            val_getter = "data[%d]" % offset
        elif size == 2:
            val_getter = "MSG_GET_U16(data,%d)" % offset
        elif size == 4:
            val_getter = "MSG_GET_U32(data,%d)" % offset
        else:
            raise RuntimeError("unsupported size")
        print("  uint16_t %s_raw() const   {return %s;}" % (name, val_getter))
        print("  uint16_t %s_whole() const {return %s * %d / %d;}" % (name, val_getter, mult, div))
        print("  uint16_t %s_frac() const  {return %s * %d / %d;}" % (name, val_getter, mult, div))
        print("  float    %s_flt() const   {return (float)(%s) * %d / %d;}" % (name, val_getter, mult, div))
    print("};")
    print("")

group_attrs = []
prev_group = 0
for line in lines:
    line = line.strip()
    parts = line.split(',')

    group = int(parts[0])
    offset_in_group = int(parts[1])
    total_offset = int(parts[2])
    size = int(parts[3])
    signed = parts[4] == 'Y'
    name = parts[5].strip()
    name = name.replace(' ','')
    description = parts[6]
    units = ''
    mult = 1
    div = 1
    add = 0
    ms2 = False
    if description != 'Unused':
        units = parts[7]
        mult = int(parts[8])
        div = int(parts[9])
        add = int(parts[10])
        ms2 = parts[11] == 'Y'

    if group != prev_group:
        print_group(prev_group, group_attrs)
        group_attrs = []

    group_attrs.append({
        'offset_in_group' : offset_in_group,
        'size' : size,
        'signed' : signed,
        'name' : name,
        'description' : description,
        'units' : units,
        'mult' : mult,
        'div' : div,
        'add' : add,
        'ms2' : ms2
    })

    prev_group = group
