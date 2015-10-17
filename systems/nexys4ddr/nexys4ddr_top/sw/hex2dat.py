import sys

def hex2dat_(hex_name):
    hex_file = open(hex_name,'r')
    lines = hex_file.readlines()
    hex_file.close()
    for l in lines:
        l = l.rstrip()
        if (not l.startswith(':')):
            continue
        l = l.lstrip(':')
        length = int(l[0:2], 16)
        address = int(l[2:6],16)
        cmd = int(l[6:8],16)
        if (len(l) > 0 and cmd == 0):
            val = 0
            for i in xrange(0,length):
                byte = int(l[8+2*i:10+2*i], 16)
                val = val + byte * (256 ** (3 - (address % 4)))
                #print hex(address) + ' ' + hex(address) + ' ' + hex(byte) + ' ' + hex(val)
                if (address % 4 == 3 or i == length-1):
                    #print '-- ' + hex(address) + ' ' + hex(address) + ' ' + hex(byte) + ' ' + hex(val)
                    #w(address-3, val)
                    print ("{:08X}_{:08X}".format(address-3, val))
                    val = 0
                address = address + 1
                #w(address, val)

def hex2dat(hex_name):
    hex_file = open(hex_name,'r')
    lines = hex_file.readlines()
    hex_file.close()
    for l in lines:
        l = l.rstrip()
        if (not l.startswith(':')):
            continue
        l = l.lstrip(':')
        length = int(l[0:2], 16)
        address = int(l[2:6],16)
        cmd = int(l[6:8],16)
        if (len(l) > 0 and cmd == 0):
            i = 0
            while i < length:
                val = 0
                for j in xrange(0,4):
                    if i == length:
                        break
                    byte = int(l[8+2*i:10+2*i], 16)
                    val = val + byte * (256 ** (3 - j))
                    #print l + '\n  ' + hex(address) + ' ' + hex(i) + ' ' + hex(j) + ' ' + hex(byte) + ' ' + hex(val)
                    i = i + 1
                print ("{:08X}_{:08X}".format(address, val))
                address = address + 4
                #w(address, val)

hex_name = sys.argv[1]
hex2dat (hex_name)

