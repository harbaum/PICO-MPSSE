#!/usr/bin/python3
# usbmon_mpsse_parse.py

# This script parses the Linux USBMon output generated while taking a trace of a FTDI MPSSE run.
# It's primary intention is to capture and analyze the traffic between closed source FPGA programmer
# software and a FPGA board using a FTDI FT232/2232/4232 or similar for configuration upload

# Related docs
# https://www.ftdichip.com/Documents/AppNotes/AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
# https://github.com/lipro/libftdi/tree/master/src

import sys

print("USBMON MPSSE parser")

# parse command line
if len(sys.argv) < 3:
    print("Usage:",sys.argv[0],"<trace> <bus>:<dev>")
    print("  Get trace from usbmon for e.g. bus 1:")
    print("  $ sudo modprobe usbmon")
    print("  $ sudo cat /sys/kernel/debug/usb/usbmon/1u > usbmon_trace.txt")
    sys.exit(-1)

FILE = sys.argv[1]
BUS = int(sys.argv[2].split(":")[0])
DEV = int(sys.argv[2].split(":")[1])

# some ansi color codes for output
BLACK = "\033[0;30m"
RED = "\033[0;31m"
GREEN = "\033[0;32m"
BROWN = "\033[0;33m"
BLUE = "\033[0;34m"
PURPLE = "\033[0;35m"
CYAN = "\033[0;36m"
LIGHT_GRAY = "\033[0;37m"
DARK_GRAY = "\033[1;30m"
LIGHT_RED = "\033[1;31m"
LIGHT_GREEN = "\033[1;32m"
YELLOW = "\033[1;33m"
LIGHT_BLUE = "\033[1;34m"
LIGHT_PURPLE = "\033[1;35m"
LIGHT_CYAN = "\033[1;36m"
LIGHT_WHITE = "\033[1;37m"
BOLD = "\033[1m"
FAINT = "\033[2m"
ITALIC = "\033[3m"
UNDERLINE = "\033[4m"
BLINK = "\033[5m"
NEGATIVE = "\033[7m"
CROSSED = "\033[9m"
END = "\033[0m"
    
INSTRUCTIONS = {
    "Gowin": {    
        0x02: "NOOP",
        0x03: "READ SRAM",
        0x05: "ERASE SRAM",    
        0x09: "XFER DONE",
        0x11: "READ ID CODE",
        0x12: "INIT ADDR",
        0x13: "READ USERCODE",
        0x15: "CONFIG ENABLE",
        0x17: "XFER WRITE",
        0x3a: "CONFIG DISABLE",
        0x3c: "RELOAD",
        0x41: "STATUS REGISTER"
    },
    "efinix": {
        0x02: "SAMPLE PRELOAD",
        0x00: "EXTEST",
        0x0f: "BYPASS",
        0x03: "IDCODE",
        0x04: "PROGRAM",
        0x07: "ENTERUSER",
        0x08: "USER1"
    }
}

# JTAG states
JTAG_STATES = {
    # name, (next state if tms = 0, next state if TMS 1)
    "TEST LOGIC RESET": ("RUN TEST IDLE", None),
    "RUN TEST IDLE": (None, "SELECT DR SCAN"),
    
    "SELECT DR SCAN": ("CAPTURE DR", "SELECT IR SCAN"),
    "CAPTURE DR": ("SHIFT DR", "EXIT1 DR"),
    "SHIFT DR": (None, "EXIT1 DR"),
    "EXIT1 DR": ("PAUSE DR", "UPDATE DR"),
    "PAUSE DR": (None, "EXIT2 DR"),
    "EXIT2 DR": ("SHIFT DR", "UPDATE DR"),
    "UPDATE DR": ("RUN TEST IDLE", "SELECT DR SCAN"),

    "SELECT IR SCAN": ("CAPTURE IR", "TEST LOGIC RESET"),
    "CAPTURE IR": ("SHIFT IR", "EXIT1 IR"),
    "SHIFT IR": (None, "EXIT1 IR"),
    "EXIT1 IR": ("PAUSE IR", "UPDATE IR"),
    "PAUSE IR": (None, "EXIT2 IR"),
    "EXIT2 IR": ("SHIFT IR", "UPDATE IR"),
    "UPDATE IR": ("RUN TEST IDLE", "SELECT DR SCAN")
}

# JTAG state machine
jtag_state = "TEST LOGIC RESET"
TMS = None
IR = None
DR = None
DR_REPLY = None
SHIFT_CNT = [0,0]

# keep track of detected device to be able to parse further
# communication correctly
device = None

def instruction_parse(ir, dlen):
    global device
    print(YELLOW+"JTAG: IR loaded with",dlen,"bits:", hex(ir), end=" -> ")

    if not device:
        print("No device detected", END)
        return
    
    if not device[0] in INSTRUCTIONS:
        print("Unknown manufacturer", device[0], END)
        return

    if dlen != device[3]:
        print(LIGHT_RED+"Warning: Cmd len", dlen, "!=", device[3],",",end=" ")
        print(hex(ir),"->",hex(ir & (1<<device[3])-1),END,end="")
        ir = ir & (1<<device[3])-1

    if not ir in INSTRUCTIONS[device[0]]:
        print("Unknown instruction", hex(ir), "for", device[0], END)
        return
    
    print(device[0], INSTRUCTIONS[device[0]][ir]+END)    
            
def parse_id_code(data):            
    global device
    
    if len(data) < 4:
        print("JTAG: unexpected IDCODE length")
    else:
        idcode = int.from_bytes(bytes(data), byteorder='little', signed=False)
        idcode &= 0xefffffff  # mask last bit transmitted
                    
        print("JTAG: IDCODE", hex(idcode))
        manufacturer = (idcode >> 1) & 0x7ff
        if idcode in PARTS:
            device = PARTS[idcode]
            print("JTAG: Manufacturer:", device[0])
            print("JTAG: Family:", device[1])
            print("JTAG: Model:", device[2])
            print("JTAG: IR length:", device[3])

        else:
            device = None
            print("JTAG: Manufacturer: ", hex((idcode >> 1) & 0x7ff))
            print("JTAG: Part:", hex((idcode >> 12) & 0xffff))
            print("JTAG: Version:",  (idcode >> 28) & 0xf)
                    
def gowin_status_parse(status):
    GOWIN_STATUS_BITS = {      
        0: "CRC_ERROR",
        1: "BAD_COMMAND",
        2: "ID_VERIFY_FAILED",
        3: "TIMEOUT",
        5: "MEMORY_ERASE",
        6: "PREAMBLE",
        7: "SYSTEM_EDIT_MODE",
        8: "PRG_SPIFLASH_DIRECT",
        10: "NON_JTAG_CNF_ACTIVE",
        11: "BYPASS",
        12: "GOWIN_VLD",
        13: "DONE_FINAL",
        14: "SECURITY_FINAL",
        15: "READY",
        16: "POR",
        17: "FLASH_LOCK"
    }
    
    print("GOWIN: status", hex(status), end="=")
    status_list = []
    for i in reversed(range(20)):
        if i in GOWIN_STATUS_BITS and status & (1<<i):
            status_list.append(GOWIN_STATUS_BITS[i])
    print(",".join(status_list))
    
def data_out_parse(data):
    global SHIFT_CNT, IR

    if len(data) <= 16:    
        print("JTAG: DR data out", ' '.join(format(x, '02x') for x in data))
    else:
        print("JTAG: DR data out", ' '.join(format(x, '02x') for x in data[:16]), "total:", len(data))

    # the data received this way should actually match the length
    # if the data shifted in before    
    if 8*len(data) != SHIFT_CNT[1]:
        print(LIGHT_RED + "Warning: Expected", SHIFT_CNT[1], "bits in return, got", 8*len(data), END)
    
    # no IR written yet, so this is the idcode
    if IR == None:
        parse_id_code(data)
    else:
        # IR set, so parse device specific instructions            
        if device:
            if device[0] == "Gowin":
                if IR == 0x11:
                    parse_id_code(data)
                if IR == 0x41:
                    gowin_status_parse(int.from_bytes(bytes(data), byteorder='little', signed=False))

            elif device[0] == "efinix":
                if IR == 0x03:
                    parse_id_code(data)    

# TODO: rename DR_REPLY to data_in or so ... as it can also be used for reading the ir
def data_in_parse(dr, dlen):
    global DR_REPLY

    print("JTAG: DR loaded with", dlen, "("+ str(dlen//8)+"+"+str(dlen%8)+")", "bits", end="")
    if dlen <= 48: print(":", hex(dr))
    else:          print("")

    data_out_parse(DR_REPLY)
        
def jtag_parse(tms_in, tdi, reading):
    global jtag_state, IR, TMS, SHIFT_CNT, DR, DR_REPLY

    # print(PURPLE+"TDI", tdi, "TMS", tms_in, END)
        
    # ---------------- TDI -------------------
    
    # handle TDI first as TMS state changes do not take
    # effect before the next cycle

    if jtag_state == "SHIFT IR":
        IR |= tdi << SHIFT_CNT[0]
        
    if jtag_state == "SHIFT DR":
        DR |= tdi << SHIFT_CNT[0]
        
    if jtag_state.startswith("SHIFT "):
        # print(CYAN+"<R"+str(SHIFT_CNT)+">", END)
        # SHIFT_CNT[0] counts the bits actually written
        SHIFT_CNT[0] += 1

        # SHIFT_CNT[1] counts the bits read and expected to be returned via BULK OUT
        if reading:
            # print(CYAN+"<R"+str(SHIFT_CNT[1])+">", END)
            SHIFT_CNT[1] += 1
       
    # ---------------- TMS -------------------
    
    if not jtag_state in JTAG_STATES:
        print("Don't know how to proceed from", jtag_state)
        sys.exit(-1)

    state = JTAG_STATES[jtag_state]
    if tms_in != None:
        TMS = tms_in  # remember new TMS state
        
    if state[TMS]:
        # it's actually unexpected that an implicit TMS value
        # changes the state. So let's keep an eye on that
        if tms_in == None:
            print("Implicit JTAG state change")
            sys.exit(1)

        jtag_state = state[TMS]
        print("JTAG: TDI", tdi, "TMS", tms_in, "->", jtag_state)

        # entering UPDATE DR state?
        # (checking TMS is not needed dur to state[TMS] being None)
        if jtag_state == "UPDATE DR": data_in_parse(DR, SHIFT_CNT[0])
            
        # entering UPDATE IR state?
        # (checking TMS is not needed due to state[TMS] being None)
        if jtag_state == "UPDATE IR": instruction_parse(IR, SHIFT_CNT[0])
            
        # just reached a SHIFT state?
        if jtag_state.startswith("SHIFT "):
            # print(CYAN, "<CLR",SHIFT_CNT,">", END)
            SHIFT_CNT = [0,0]        
        if jtag_state == "SHIFT IR":
            IR = 0
        if jtag_state == "SHIFT DR":
            DR = 0
            DR_REPLY = []

# The parts list only contains a few parts I've ever seen. Feel free to add more
# e.g. from openFPGAloader/src/part.hpp
PARTS = {
    # Efinix Trion
    0x00000001: ("efinix", "Trion", "T4/T8", 4),
    0x00210a79: ("efinix", "Trion", "T8QFP144/T13/T20", 4),
    0x00220a79: ("efinix", "Trion", "T55/T85/T120",     4),
    0x00240a79: ("efinix", "Trion", "T20BGA324/T35",    4),
    # Gowin GW1
    0x0100481b: ("Gowin", "GW1N",   "GW1N(R)-9C", 8),
    0x0100581b: ("Gowin", "GW1N",   "GW1NR-9",    8),
    0x0900281B: ("Gowin", "GW1N",   "GW1N-1",     8),
    0x0120681b: ("Gowin", "GW1N",   "GW1N-2",     8),
    0x0100381B: ("Gowin", "GW1N",   "GW1N-4",     8),
    0x0100681b: ("Gowin", "GW1NZ",  "GW1NZ-1",    8),
    0x0300181b: ("Gowin", "GW1NS",  "GW1NS-2C",   8),
    0x0100981b: ("Gowin", "GW1NSR", "GW1NSR-4C",  8),    
    # Gowin GW2
    0x0000081b: ("Gowin", "GW2A",   "GW2A(R)-18(C)", 8),    
    # Gowin GW5
    0x0001081b: ("Gowin", "GW5AST", "GW5AST-138", 8),
    0x0001181b: ("Gowin", "GW5AT",  "GW5AT-138",  8)
}

def parse_bulk_data_in(data, cmd = None):
    global IR, DR_REPLY

    if len(data) < 2: return
    
    # the first two bytes always seem to be 0xxx 0x60 and carrying the line status
    # print("MPSSE: FTDI header", hex(data[0]), hex(data[1]))

    # only the FTDI header?
    if len(data) < 3: return

    # skip FTDI header
    data = data[2:]    
    print(GREEN+"< TDO", ' '.join(format(x, '02x') for x in data), END)

    # simply append this data to the DR_REPLY register
    DR_REPLY += data

def parse_mpsse_shift(cmd, data):
    global SHIFT_CNT
    
    if cmd & 2:
        # bits        
        length = data[0] + 1
        data = data[1:]
    else:
        # bytes        
        length = (data[0] + 256 * data[1] + 1) * 8
        data = data[2:]

    bit_str = []
    if cmd & 0x01: bit_str.append("W-VE")
    else:          bit_str.append("W+VE")
    if cmd & 0x02: bit_str.append("BIT")
    else:          bit_str.append("BYTE")
    if cmd & 0x04: bit_str.append("R-VE")
    else:          bit_str.append("R+VE")
    if cmd & 0x08: bit_str.append("LSB")
    else:          bit_str.append("MSB")
    if cmd & 0x10: bit_str.append("W-TDI")
    if cmd & 0x20: bit_str.append("R-TDO")
    if cmd & 0x40: bit_str.append("W-TMS")
    print("SHIFT", ",".join(bit_str), "LEN="+str(length), "("+str(length//8)+"+"+str(length%8)+")" )
    
    # check if TMS is to be controlled, then
    # TDI is encoded in first bit (MSB)
    if cmd & 0x40: # <W-TMS>
        print(GREEN+"> TMS", ' '.join(format(x, '02x') for x in data[:(length+7)//8]), END)

        # we never expect TDI writing to be enabled as well
        if cmd & 0x10: # <W-TDI>
            print("TDI writing in TMS mode")
            sys.exit(-1)

        # we never expect TMS to sent in byte mode
        if not cmd & 2:
            print(LIGHT_RED + "BYTE mode in TMS mode" + END)

        # TDI is encoded in MSB
        tdi = (data[0] & 128)>>7
        
        # this will keep TDI static and process TMS and as a result
        # will change the state
        for b in range(length):
            jtag_parse( (data[0]>>b)&1, tdi, (cmd&0x20)==0x20 )
            
        # skip the one data byte
        data = data[1:]
    elif cmd & 0x10: #  <W-TDI>
        # regular TDI mode. This will not change the state of TMS
        if (length+7)//8 <= 32: print(GREEN+"> TDI", ' '.join(format(x, '02x') for x in data[:(length+7)//8]), END)
        else:                   print(GREEN+"> TDI["+str(length)+"]", ' '.join(format(x, '02x') for x in data[:32]), "...", END)
            
        # captured data may actually be too short
        while length and len(data):
            for b in range(8 if length > 8 else length):
                jtag_parse( None, (data[0]>>b)&1, (cmd&0x20)==0x20 )

            if length > 8: length -= 8
            else:          length = 0
            data = data[1:]

        # this should actually never happen
        if length:
            print("[missing "+str(length)+" bits]")
            sys.exit(-1)
    else:
        # if not data is to be written, then no data is transferred at all
        # print("<no data to be written>")
        # increase the SHIFT_CNT[0], anyways as we'll compare that with the
        # length of data transferred via bulk out
        # print(CYAN+"SHIFT_CNT += ", length, END)
        SHIFT_CNT[1] += length
            
    # we always expect W-VE- when writing with <W-TDI> or <W-TMS> bits set
    if cmd & (0x10 | 0x40) and not cmd & 0x01:
        print(LIGHT_RED + "Warning: Unexpected W+VE" + END)

    # we always expect R-VE+ when reading with <R-TDO>
    if cmd & 0x20 and cmd & 0x04:
        print(LIGHT_RED + "Warning: Unexpected R-VE" + END)

    if not cmd & 0x08:
        print(LIGHT_RED + "Warning: Unexpected MSB first" + END)

    # return data with consumed bytes removed
    return data

def mpsse_cmd_is_complete(data):
    # check if data contains a complete mpsse command
    
    # nothing has been parsed, return
    if len(data) == 0:
        return False
    
    # make sure there's a full command in buffer
    # there a MPSSE commands of fixed length and of varible length
    if not data[0] & 0x80:
        # any shift command needs at least a total of three bytes
        if len(data) < 3:
            return False

        # byte length shifts have a variable length
        if not data[0] & 2:
            length = data[1] + 256 * data[2] + 1

            # the length only matters for the data size if data is to be written
            # with <W-TDI> or <W-TMS> bits set
            if data[0] & (0x10 | 0x40) and len(data) < 3+length:
                return False
    else:
        CMD_XTRA_LEN = { 0x86: 2, 0x8e: 1, 0x8f: 2, 0x9c: 2, 0x9d: 2, 0x9e: 2 }
        if data[0] in CMD_XTRA_LEN:
            if CMD_XTRA_LEN[data[0]]+1 > len(data):
                return None        

    return True

def parse_bulk_out(data):

    # parse mpsse command byte
    print("MPSSE: CMD "+hex(data[0])+" -> ", end="")

    # shift command
    if not data[0] & 0x80:
        # parse_mpsse_shift will return -1 if more data is needed for parsing        
        return parse_mpsse_shift(data[0], data[1:])
    
    elif data[0] == 0x80 or  data[0] == 0x82:
        iostr = ""
        for i in range(8):
            if(data[2] & (0x80>>i)): iostr = iostr + "O"
            else:                    iostr = iostr + "I"
                
        print("Set Data Bits","low" if data[0] == 0x80 else "high","byte value", hex(data[1]), "direction", iostr)
        return data[3:]
                
    elif data[0] == 0x84:
        print("Connect loopback")
        return data[1:]

    elif data[0] == 0x85:
        print("Disconnect loopback")
        return data[1:]

    elif data[0] == 0x86:
        divisor = data[1] + 256*data[2]
        rate = 12000000 / ((1+divisor) * 2)
        print("Set TCK/SK Divisor to", divisor, "=", str(rate/1000000)+"/"+str(5*rate/1000000), "Mhz")
        return data[3:]

    elif data[0] == 0x87:
        print("Flush")
        return data[1:]

    elif data[0] == 0x8a:
        print("Disable div by 5 (60MHz master clock)")
        return data[1:]
        
    elif data[0] == 0x8b:
        print("Enable div by 5 (12MHz master clock)")
        return data[1:]

    elif data[0] == 0x8c:
        print("Enable 3 Phase Data Clocking")
        return data[1:]

    elif data[0] == 0x8d:
        print("Disable 3 Phase Data Clocking")
        return data[1:]

    elif data[0] == 0x8e:
        print("Clock For",data[1],"bits with no data transfer")
        return data[2:]

    elif data[0] == 0x8f:
        n = data[1]*256 + data[2]
        print("Clock For",n,"x8 bits with no data transfer")
        return data[3:]

    elif data[0] == 0x94:
        print("Clk continuously and Wait On I/O High")
        return data[1:]

    elif data[0] == 0x95:
        print("Clk continuously and Wait On I/O Low")
        return data[1:]

    elif data[0] == 0x96:
        print("Turn On Adaptive clocking")
        return data[1:]

    elif data[0] == 0x97:
        print("Turn Off Adaptive clocking")
        return data[1:]

    elif data[0] == 0x9C:
        n = data[1]*256 + data[2]        
        print("Clock For",n,"x 8 bits with no data transfer or Until GPIOL1 is High")
        return data[3:]

    elif data[0] == 0x9D:
        n = data[1]*256 + data[2]        
        print("Clock For",n,"x 8 bits with no data transfer or Until GPIOL1 is Low")
        return data[3:]

    elif data[0] == 0x9E:
        print("Set I/O to only drive on a ‘0’ and tristate on a ‘1’")
        return data[3:]

    else:
        print("Unknown command", hex(data[0]))
    return None

# a buffer, to keep data until a command is complete
bulk_out_buffer = None

def parse_line(line):
    global bulk_out_buffer

    parts = line.split()

    if len(parts) < 4:
        print("short line")
        return
    
    urb = parts[0]   # we actually don't care for that
    ts = parts[1]    # timestamp in microseconds
    et = parts[2]    # event type (Submission, Callback, Error)
    addr = parts[3]
    
    # parse the address
    addr_parts = addr.split(":")
    if len(addr_parts) != 4:
        print(CYAN+"USB: unexpected address length", addr, END)
        return

    # Control, Ziso, Interrupt, Bulk _i_n/_o_ut
    addr_type = addr_parts[0].lower()
    addr_bus =  int(addr_parts[1])
    addr_dev =  int(addr_parts[2])
    addr_ep  =  int(addr_parts[3])

    # this is not the device you are looking for ...
    if addr_bus != BUS or addr_dev != DEV:        
        return
    
    dir = "in" if addr_type[1] == 'i' else "out"
        
    if addr_type[0] == 'c':
        if dir == "out" and et.lower() == "s":
            # control out
            # print("control", line)

            if int(parts[5], 16) == 0x40:
                # https://github.com/lipro/libftdi/blob/master/src/ftdi.h
                RT = {
                    0x00: "Reset",
                    0x01: "Modem control",
                    0x02: "Set flow control",
                    0x03: "Set baud rate",
                    0x04: "Set data characteristics",
                    0x05: "Poll modem status",
                    0x06: "Set event character",
                    0x07: "Set error character",
                    0x09: "Set latency timer",
                    0x0a: "Get latency timer",
                    0x0b: "Set bitmode",
                    0x0c: "Read pins",
                    0x90: "Read eeprom",
                    0x91: "Write eeprom",
                    0x92: "Erase eeprom"
                }
                
                req = int(parts[6], 16)                
                print(CYAN+"USB Vendor request", hex(req), end=" ")
                if req in RT:
                    # print(line)
                    print(RT[req], end="")

                    # parse some requests further
                    value = int(parts[7], 16)
                    index = int(parts[8], 16)
                    
                    if req == 0x00 and value < 3:
                        print(" ->", ["Reset SIO", "Purge RX", "Purge TX"][value], end="")
                    elif req == 0x03:
                        divisor = value + 65536 * index                        
                        print(" -> divisor="+hex(divisor), 12000000/value, end="")
                    elif req == 0x09:
                        print(" -> latency="+str(value), end="")
                    elif req == 0x0b:
                        BM = { 0:"RESET", 1:"BITBANG", 2:"MPSSE", 4:"SYNCBB", 8:"MCU", 16:"OPTO", 32:"CBUS", 64:"SYNCFF", 128:"FT1284" }
                        if value>>8 in BM: mode = BM[value>>8]
                        else:              mode = hex(value>>8)
                        print(" -> mode="+mode, "mask="+hex(value & 0xff), end="")

                print(END)
            
        return
        
    if addr_type[0] == 'b':
        urb_status = int(parts[4])
        length = int(parts[5])
        if length and  parts[6] == "=":
            data = "".join(parts[7:])
            
            databytes = []
            for b in range(len(data)//2):
                databytes.append(int(data[2*b:2*b+2], 16))
            data = databytes                
        else:
            data = None

        if data and dir == "in":
            print(CYAN+"USB BULK IN", ' '.join(format(x, '02x') for x in data), END)
            parse_bulk_data_in(data)
        
        if data and dir == "out":
            # submissions only capture up to 32 bytes. Fill it up with x00 bytes hoping that this
            # is still parsable. This may (and does) cause trouble at the end of long transfers if the
            # missing parts are supposed to be further commands and not just data
            if length > len(data):
                # print(LIGHT_RED, "Missing", length-len(data), "bytes. Filling up with 00", END)
                data += [0]*(length-len(data))
            
            if len(data) <= 32:
                print(CYAN+"USB BULK OUT", ' '.join(format(x, '02x') for x in data), END)
            else:
                print(CYAN+"USB BULK OUT["+str(len(data))+"]", ' '.join(format(x, '02x') for x in data[:32]), "...", END)

            # There's unused data from previous transfer(s)? Prepend it. This actually barely happens in real
            # life.
            if bulk_out_buffer:
                data = bulk_out_buffer + data
                bulk_out_buffer = None

            while data and mpsse_cmd_is_complete(data):
                data = parse_bulk_out(data)

            if data:
                bulk_out_buffer = data
                    
with open(FILE, "r") as f:
    for line in f:
        line = line.strip()
        try:
            parse_line(line)
        except Exception as e:
            print("While parsing", line)
            print("Exception:", e)
            sys.exit(-1)
