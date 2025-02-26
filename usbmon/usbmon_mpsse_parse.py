#!/usr/bin/python3
# usbmon_mpsse_parse.py

# This script parses the Linux USBMon output generated while taking a trace of a FTDI MPSSE run.
# It's primary intention is to capture and analyze the traffic between closed source FPGA programmer
# software and a FPGA board using a FTDI FT232/2232/4232 or similar for configuration upload

# Related docs
# https://www.ftdichip.com/Documents/AppNotes/AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
# https://github.com/lipro/libftdi/tree/master/src
# https://github.com/radupotop/usbmon
# https://github.com/lipro/libftdi/blob/master/src/ftdi.h

# TODO: handle both ports seperately

import sys

print("USBMON MPSSE parser")

def usage():
    print("Usage:",sys.argv[0],"[-n|-p=<file>|-v=<file>] <trace> <bus>:<dev>")
    print("  -n        no colorized output")
    print("  -p=<file> write programming data to <file>")
    print("  -v=<file> verify programming data from <file>")

    print("")
    print("  Get trace from usbmon for e.g. bus 1:")
    print("  $ sudo modprobe usbmon")
    print("  $ sudo cat /sys/kernel/debug/usb/usbmon/1u > usbmon_trace.txt")
    print("  or use the usbmon command line utility to capture e.g. 4096 bytes")
    print("  $ sudo usbmon -fu -i 1 -s 4096 > usbmon_trace.txt")
    sys.exit(-1)

# parse command line
option_color = True        # use colors by default
option_prog_file = None    # 
option_verify_file = None  # 

index = 1
while index < len(sys.argv) and sys.argv[index][0] == "-":
    if sys.argv[index][1]     == "n": option_color = False
    elif sys.argv[index][1:3] == "p=": option_prog_file = sys.argv[index][3:]
    elif sys.argv[index][1:3] == "v=": option_verify_file = sys.argv[index][3:]
    else: usage()    
    index += 1

# we need to more non-option parameters
if len(sys.argv) < 2+index: usage()
    
FILE = sys.argv[index]
BUS = int(sys.argv[index+1].split(":")[0])
DEV = int(sys.argv[index+1].split(":")[1])

if option_color:
    # some ansi color codes for output
    RED = "\033[0;31m"; GREEN = "\033[0;32m"; BROWN = "\033[0;33m"; BLUE = "\033[0;34m"; PURPLE = "\033[0;35m";
    CYAN = "\033[0;36m"; LIGHT_GRAY = "\033[0;37m"; DARK_GRAY = "\033[1;30m"; LIGHT_RED = "\033[1;31m";
    LIGHT_GREEN = "\033[1;32m"; YELLOW = "\033[1;33m"; LIGHT_BLUE = "\033[1;34m"; LIGHT_PURPLE = "\033[1;35m";
    LIGHT_CYAN = "\033[1;36m"; LIGHT_WHITE = "\033[1;37m"; END = "\033[0m";
else:
    # no colors
    RED = ""; GREEN = ""; BROWN = ""; BLUE = ""; PURPLE = ""; CYAN = ""; LIGHT_GRAY = ""; DARK_GRAY = "";
    LIGHT_RED = ""; LIGHT_GREEN = ""; YELLOW = ""; LIGHT_BLUE = ""; LIGHT_PURPLE = ""; LIGHT_CYAN = "";
    LIGHT_WHITE = ""; END = "";

# open programming data file if needed
if option_prog_file:
    option_prog_file = open(option_prog_file, "wb")

if option_verify_file:
    option_verify_file = open(option_verify_file, "rb")

def hexlimit(title, data, maxlen=32):
    if title: retval = title + "[" + str(len(data)) + "]: "
    else: retval = ""
    
    if len(data) <= maxlen: retval += ' '.join(format(x, '02x') for x in data)
    else:                   retval += ' '.join(format(x, '02x') for x in data[:maxlen]) + " ..."
    return retval
    
# JTAG instructions as used by the different FPGA vendors
INSTRUCTIONS = {
    "Gowin": {    
        0x02: "NOOP", 0x03: "READ SRAM", 0x05: "ERASE SRAM",0x09: "XFER DONE",
        0x11: "READ ID CODE", 0x12: "INIT ADDR", 0x13: "READ USERCODE",
        0x15: "CONFIG ENABLE", 0x17: "XFER WRITE", 0x3a: "CONFIG DISABLE",
        0x3c: "RELOAD", 0x41: "STATUS REGISTER" },
    "efinix": {
        0x02: "SAMPLE PRELOAD", 0x00: "EXTEST", 0x0f: "BYPASS", 0x03: "IDCODE",
        0x04: "PROGRAM", 0x07: "ENTERUSER", 0x08: "USER1", 0x09: "USER2",
        0x0a: "USER3", 0x0b: "USER4" }
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

port_mode = None

# JTAG state machine
jtag = {
    "state": "TEST LOGIC RESET",        # state of JTAG state machine
    "shift_cnt": { "out": 0, "in": 0 }, # counter for in/out shifting

    "tms": None,                        # current state of TMS signal
    "ir": None,                         # jtag instruction register written by host
    "dr": None,                         # jtag data register written by host

    # jtag data returned by device
    "dr_return": { "data": [], "length": 0, "expect": [], "premature": [] }
}

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
        print(hex(ir),"->",hex(ir & (1<<device[3])-1),END + YELLOW,end="")
        ir = ir & (1<<device[3])-1

    if not ir in INSTRUCTIONS[device[0]]:
        print("Unknown instruction", hex(ir), "for", device[0], END)
        return
    
    print(device[0], INSTRUCTIONS[device[0]][ir]+END)    
            
def parse_id_code(data):            
    global device
    
    if len(data) < 4:
        print(LIGHT_RED + "JTAG: unexpected IDCODE length", len(data), END)
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
    
    print(LIGHT_PURPLE + "GOWIN: status", hex(status), end="=")
    status_list = []
    for i in reversed(range(20)):
        if i in GOWIN_STATUS_BITS and status & (1<<i):
            status_list.append(GOWIN_STATUS_BITS[i])
    print(",".join(status_list) + END)

def efinix_sample_preload_parse(data):
    # reverse all bytes (due to lsb first transmission)
    for i in range(len(data)):
        byte = data[i]
        byte = ((byte & 0x55) << 1) | ((byte & 0xaa) >> 1)
        byte = ((byte & 0x33) << 2) | ((byte & 0xcc) >> 2)
        byte = ((byte & 0x0f) << 4) | ((byte & 0xf0) >> 4)
        data[i] = byte

    print(LIGHT_PURPLE + "Efinix: SAMPLE PRELOAD");

    addr = 0
    while(len(data)):
        print(format(addr, '04x'), end=": ")
        bytes2print = 16 if len(data)>16 else len(data)
        for b in range(bytes2print):
            print(format(data[b], '02x'), end=" ")
        for b in range(16-bytes2print):            
            print("   ", end="")

        for b in range(bytes2print):
            if data[b] >= 32 and data[b] < 127:
                print(chr(data[b]), end="")
            else:
                print(".", end="")

        print("")
        data = data[16:]
        addr += 16
        
    print(END)
    
def data_out_parse(reply):
    global jtag

    print("JTAG: "+hexlimit("DR data out", reply["data"]))

    # the data received this way should actually match the length
    # if the data shifted in before    
    if reply["length"] != jtag["shift_cnt"]["out"]:
        print(LIGHT_RED + "Warning: Expected", jtag["shift_cnt"]["out"], "bits in return, got", reply["length"], END)
    
    # no IR written yet, so this is the idcode
    if jtag["ir"] == None:
        parse_id_code(reply["data"])
    else:
        # IR set, so parse device specific instructions            
        if device:
            if device[0] == "Gowin":
                if jtag["ir"] == 0x11:
                    parse_id_code(reply["data"])
                if jtag["ir"] == 0x41:
                    gowin_status_parse(int.from_bytes(bytes(reply["data"]), byteorder='little', signed=False))

            elif device[0] == "efinix":
                if jtag["ir"] == 0x03:
                    parse_id_code(reply["data"])
                if jtag["ir"] == 0x02:
                    efinix_sample_preload_parse(reply["data"])

# state needed when verifying the data we see as "program" payload with data from a reference file
verify = { "putback": None, "index": 0, "mismatch": 0 }

def data_in_parse(dr, dlen):
    global INSTRUCTIONS, jtag, verify, option_verify_file

    # if dlen%8 == 1 it's very likely, that this is the extra bit generated when leaving 'SHIFT DR' state
    
    print("JTAG: DR loaded with", dlen, "("+ str(dlen//8)+"*8+"+str(dlen%8)+")", "bits: " + hexlimit(None,jtag["dr"]))

    # if the current ir is something with "program" then write the data out
    if device and device[0] in INSTRUCTIONS and jtag["ir"] in INSTRUCTIONS[device[0]]:
        if "bypass" in  INSTRUCTIONS[device[0]][jtag["ir"]].lower():
            # TODO: check for correct return data in bypass mode
            # hmmm ... the efinity programmer never leaves the shift dr state for this ...
            print(LIGHT_GREEN+"Bypass", jtag["dr"], dr, END)

        if "program" in  INSTRUCTIONS[device[0]][jtag["ir"]].lower():
            print(YELLOW+"Programmed", device[0], "device with", dlen, "bits.", END)
            
            # adjust bit order from LSB to MSB
            for byte in jtag["dr"]:
                byte = ((byte & 0x55) << 1) | ((byte & 0xaa) >> 1)
                byte = ((byte & 0x33) << 2) | ((byte & 0xcc) >> 2)
                byte = ((byte & 0x0f) << 4) | ((byte & 0xf0) >> 4)

                # simply write all bytes to file
                if option_prog_file:
                    option_prog_file.write(bytes([byte]))

                # the verify option is somewhat special as it tries to skip and report zero
                # bytes that have been inserted by the programmer tool as e.g. efinity does
                if option_verify_file:
                    if verify["putback"] != None:
                        vbyte = verify["putback"]
                        verify["putback"] = None
                    else:
                        # Read byte from file to compare to. use -1 if end of file reached
                        # since -1 will never match. This will be reported at the end
                        readbyte = option_verify_file.read(1)
                        if len(readbyte): vbyte = int.from_bytes(readbyte)
                        else:             vbyte = -1
                        
                    if vbyte != byte:
                        verify["mismatch"] += 1
                        verify["putback"] = vbyte
                    else:
                        if verify["mismatch"]:
                            print(LIGHT_RED+"Verify mismatch", verify["mismatch"], "at", verify["index"]-verify["mismatch"], END)
                            verify["mismatch"] = 0

                    verify["index"] += 1

            if verify["mismatch"]:
                print(LIGHT_RED+"Verify overrun", verify["mismatch"], "at", verify["index"]-verify["mismatch"], END)
    
    data_out_parse(jtag["dr_return"])
        
def jtag_parse(tms_in, tdi, reading):
    global jtag

    # print(PURPLE+"TDI", tdi, "TMS", tms_in, END)
        
    # ---------------- TDI -------------------
    
    # handle TDI first as TMS state changes do not take
    # effect before the next cycle

    if jtag["state"] == "SHIFT IR":
        jtag["ir"] |= tdi << jtag["shift_cnt"]["in"]
        
    if jtag["state"] == "SHIFT DR":
        # DR will grow pretty much, so we handle this as a array of bytes
        if jtag["shift_cnt"]["in"] & 7 == 0:
            # first bit of byte
            jtag["dr"].append(0)
            jtag["dr"][-1] |= tdi
        else:
            jtag["dr"][-1] |= tdi << (jtag["shift_cnt"]["in"]&7)
        
    if jtag["state"].startswith("SHIFT "):
        # jtag["shift_cnt"]["in"] counts the bits actually written
        jtag["shift_cnt"]["in"] += 1

        # jtag["shift_cnt"]["out"] counts the bits read and expected to be returned via BULK OUT
        if reading:
            jtag["shift_cnt"]["out"] += 1
       
    # ---------------- TMS -------------------
    
    if not jtag["state"] in JTAG_STATES:
        print("Don't know how to proceed from", jtag["state"])
        sys.exit(-1)

    state = JTAG_STATES[jtag["state"]]
    if tms_in != None:
        jtag["tms"] = tms_in  # remember new TMS state

    if jtag["tms"] != None and state[jtag["tms"]]:
        # it's actually unexpected that an implicit TMS value
        # changes the state. So let's keep an eye on that
        if tms_in == None:
            print("Implicit JTAG state change")
            sys.exit(1)

        jtag["state"] = state[jtag["tms"]]
        print("JTAG: TDI", tdi, "TMS", tms_in, "->", jtag["state"])

        # entering UPDATE DR state?
        # (checking TMS is not needed dur to state[TMS] being None)
        if jtag["state"] == "UPDATE DR": data_in_parse(jtag["dr"], jtag["shift_cnt"]["in"])
            
        # entering UPDATE IR state?
        # (checking TMS is not needed due to state[TMS] being None)
        if jtag["state"] == "UPDATE IR": instruction_parse(jtag["ir"], jtag["shift_cnt"]["in"])
            
        # just reached a SHIFT state?
        if jtag["state"].startswith("SHIFT "):
            # print(LIGHT_PURPLE + "RESET jtag["shift_cnt"]" + END)
            jtag["shift_cnt"]["in"] = 0        
            jtag["shift_cnt"]["out"] = 0        
        if jtag["state"] == "SHIFT IR":
            jtag["ir"] = 0
        if jtag["state"] == "SHIFT DR":
            jtag["dr"] = []
            jtag["dr_return"]["data"] = []
            jtag["dr_return"]["length"] = 0

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

def check_for_expected_data(data = []):
    global jtag

    # Prepend any data that have been received before we've seen the
    # request for it. This happens for some odd reason like e.g. a
    # race condition in the usbmon
    if jtag["dr_return"]["premature"]:
        data = jtag["dr_return"]["premature"] + data
        jtag["dr_return"]["premature"] = []

    # no data to be processed, not even "premature" data
    if not data: return        
    
    #print(LIGHT_PURPLE + ">>> INCOMING <<<")

    # the shift commands
    #print(hexlimit("data", data))
    # print("expect:", jtag["dr_return"]["expect"], "shift_cnt:", jtag["shift_cnt"], jtag["dr_return"]["length"])

    # in some traces we've seen the reply before the request ...
    #if not len(jtag["dr_return"]["expect"]):
    #    print(LIGHT_RED + "Warning: no data expected!" + END + LIGHT_PURPLE)
    
    # walk over all expected bit patterns and parse incoming
    # data according to this. This would normally be done by the
    # application (e.g. openfpgaloader) itself. But since we've
    # observed the actual requests we can correctly parse
    # these as well.
    
    # process all data bytes
    # print("got", len(data))
    while len(jtag["dr_return"]["expect"]) and len(data):
        # check if there are enough data bytes left
        bits2read = jtag["dr_return"]["expect"][0]
        if bits2read > len(data)*8:
            bits2read = len(data)*8
        
        # walk over all bits
        for b in range(bits2read):
            if jtag["dr_return"]["length"]&7 == 0: jtag["dr_return"]["data"].append(0)
            # copy bit
            if data[0] & (0x80>>(b&7)): jtag["dr_return"]["data"][-1] |= (0x80>>(jtag["dr_return"]["length"]&7))
            # fully read one source byte?
            if b&7 == 7: data = data[1:]
            jtag["dr_return"]["length"] += 1

        # could the expect be satisfied?
        if bits2read == jtag["dr_return"]["expect"][0]:
            # if jtag["dr_return"]["expect"][0] wasn't a multiple of 8 then there's still unused bits in the last
            # data byte. This needs to be discarded.
            if jtag["dr_return"]["expect"][0] & 7: data = data[1:]
            
            # expected data has been handled
            jtag["dr_return"]["expect"] = jtag["dr_return"]["expect"][1:]
            # print("all consumed!")
        else:
            jtag["dr_return"]["expect"][0] -= bits2read
            # print("left", jtag["dr_return"]["expect"][0])
            
    if len(data):
        jtag["dr_return"]["premature"].extend(data)
        
    # print(hexlimit("DR return"+"("+str(jtag["dr_return"]["length"])+" bits)", jtag["dr_return"]["data"]), END)

# bulk data in the the reply from the FTDI chip
def parse_bulk_data_in(data, cmd = None):
    if len(data) < 2: return
    
    # the first two bytes always seem to be 0xxx 0x60 and carrying the line status
    # print("MPSSE: FTDI header", hex(data[0]), hex(data[1]))

    # only the FTDI header?
    if len(data) < 3: return

    # skip FTDI header
    data = data[2:]    
    print(GREEN+hexlimit("<TDO ", data), END)

    check_for_expected_data(data)

def parse_mpsse_shift(cmd, data):
    global jtag
    
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
    print("SHIFT", ",".join(bit_str), "LEN="+str(length), "("+str(length//8)+"*8+"+str(length%8)+")" )

    # maintain a list of bytes expected to be read
    if cmd & 0x20:
        # print(LIGHT_PURPLE + "READ", length, END)
        jtag["dr_return"]["expect"].append(length)
        check_for_expected_data([])        
    
    # check if TMS is to be controlled, then
    # TDI is encoded in first bit (MSB)
    if cmd & 0x40: # <W-TMS>
        print(GREEN+hexlimit("> TMS", data[:(length+7)//8]), END)

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
        print(GREEN+hexlimit(">TDI", data[:(length+7)//8]), END)
            
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
        # increase the jtag["shift_cnt"]["in"], anyways as we'll compare that with the
        # length of data transferred via bulk out
        jtag["shift_cnt"]["out"] += length
            
    # we always expect W-VE- when writing with <W-TDI> or <W-TMS> bits set
    if cmd & (0x10 | 0x40) and not cmd & 0x01:
        print(LIGHT_RED + "JTAG Warning: Unexpected W+VE. JTAG should write on the falling edge!" + END)

    # we always expect R-VE+ when reading with <R-TDO>
    if cmd & 0x20 and cmd & 0x04:
        print(LIGHT_RED + "JTAG Warning: Unexpected R-VE. JTAG should read on the rising edge!" + END)

    if not cmd & 0x08:
        print(LIGHT_RED + "JTAG Warning: Unexpected MSB first" + END)

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

def parse_bulk_out_mpsse(data):
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
                
        print("Set Data Bits","low" if data[0] == 0x80 else "high","byte value", hex(data[1]), "direction", hex(data[2])+"="+iostr)
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

def parse_bulk_out_bitbang(data):
    print(LIGHT_PURPLE + hexlimit("BITBANG", data) + END)
    
def parse_bulk_out(data):
    global port_mode
    
    # check if we are in MPSSE mode
    if port_mode and port_mode.lower() == "mpsse":
        while data and mpsse_cmd_is_complete(data):
            data = parse_bulk_out_mpsse(data)
        return data
    elif port_mode and port_mode.lower() == "bitbang":
        parse_bulk_out_bitbang(data)
    else:
        print(LIGHT_RED+"Warning: not in mpsse mode", END)
        
    return None
    
# a buffer, to keep data until a command is complete
bulk_out_buffer = None
truncated_data = False

def parse_line(line):
    global bulk_out_buffer, truncated_data, port_mode

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
#        if dir == "in":
#            print("Control in", line)
            
        if dir == "out" and et.lower() == "s":
            # control out. Vendor?
            if int(parts[5], 16) == 0x40:
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
                value = int(parts[7], 16)
                index = int(parts[8], 16)
                    
                print(CYAN+"USB Vendor request", hex(req), "value:"+str(value), "index:"+str(index), end=" ")
                if req in RT:
                    # print(line)
                    print(RT[req], end="")

                    if req == 0x00 and value < 3:
                        print(" ->", ["Reset SIO", "Purge RX", "Purge TX"][value], end="")
                    elif req == 0x03:
                        divisor = value + 65536 * index                        
                        print(" -> divisor="+hex(divisor), 12000000/value, end="")
                    elif req == 0x09:
                        print(" -> latency="+str(value), end="")
                    elif req == 0x0b:
                        BM = { 0:"RESET", 1:"BITBANG", 2:"MPSSE", 4:"SYNCBB", 8:"MCU", 16:"OPTO", 32:"CBUS", 64:"SYNCFF", 128:"FT1284" }
                        if value>>8 in BM: port_mode = BM[value>>8]
                        else:              port_mode = hex(value>>8)
                            
                        print(" -> mode="+port_mode, "mask="+hex(value & 0xff), end="")
                        
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
            print(CYAN+hexlimit("USB EP"+str(addr_ep)+" BULK_IN", data), END)
            parse_bulk_data_in(data)
        
        if data and dir == "out":
            # usbmon be default captures up to 32 bytes. Fill it up with x00 bytes hoping that this
            # is still parsable. This may (and does) cause trouble at the end of long transfers if the
            # missing parts are supposed to be further commands and not just data
            if length > len(data):
                # print(LIGHT_RED, "Missing", length-len(data), "bytes. Filling up with 00", END)
                data += [0]*(length-len(data))
                truncated_data = True

            print(CYAN+hexlimit("USB EP"+str(addr_ep)+" BULK OUT", data)+END)

            # There's unused data from previous transfer(s)? Prepend it. This actually barely happens in real
            # life.
            if bulk_out_buffer:
                data = bulk_out_buffer + data
                bulk_out_buffer = None

            data = parse_bulk_out(data)

            if data:
                bulk_out_buffer = data

# open trace file and run over it
with open(FILE, "r") as f:
    for line in f:
        line = line.strip()
        parse_line(line)
#        try:
#            parse_line(line)
#        except Exception as e:
#            print("While parsing", line)
#            print("Exception:", e)
#            sys.exit(-1)

if option_prog_file: option_prog_file.close()

if option_verify_file:
    option_verify_file.close()

if truncated_data:
    print(LIGHT_RED + "Warning, the captured data was truncated and some parsing may have been incorrect.", END)
