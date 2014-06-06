# by Andrew Fishberg et al. Spring 2014

pktID = 0
pktTYPE = 0
pktMAG = 0

BTN_TYPE = 1 #const for btn
PAD_TYPE = 2 #const for pad

def update_D(D, msg):
    """ takes in any object D with the fields
        D.BTNS  and D.PADS
        and updates those fields with the game controller's current values
    """
    mkPacket(msg)
    #print pktID, pktTYPE, pktMAG
    if pktTYPE == BTN_TYPE: #if it is btn
        D.BTNS[pktID] = pktMAG
    elif pktTYPE == PAD_TYPE: #if it is pad
        D.PADS[pktID] = pktMAG#scaleMag(pktMAG)

def mkPacket(hex_str):
    """ parses the hex string sent from the controller's publisher """
    global pktID, pktTYPE, pktMAG

    pktID = int(hex_str[2:4], 16)
    pktTYPE = int(hex_str[4:6], 16)
    pktMAG = toSignedInt(int(hex_str[6:], 16))

    if (pktTYPE == PAD_TYPE):
        pktMAG = scaleMag(pktMAG)

    #pkt = (pktID, pktTYPE, pktMAG)

def toSignedInt(hex):
    """ converts from hex to signed integer form... 
        Go cs105 !
    """
    if (hex & 0x8000):
        hex = -0x10000 + hex
    return hex

def scaleMag(mag):
    """ scales a 16-bit signed integer, mag, 
        to a floating-point value from -1 to 1 
    """
    return float(mag) / (2 ** 15 - 1)

def packet_type_string(pkt):
    """ keeping track of the different packet types """
    output = ''
    if pktTYPE == 1:
        output += 'BTN'
    elif pktTYPE == 2:
        output += 'PAD'
    else:
        output += str(pktTYPE)

    return output

def packet_name(pkt):
    """ helper function for different packet types """
    output = ''
    output += packet_type(pkt)
    if pId < 10:
        output += '0'
    output += str(pId)

    return output