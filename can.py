import can
#Need to define 2 paramter CAN_RTR_REMOTE and CAN_RTR_DATA
def BSP_CAN_Init(self):
    try:
	    bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
    except OSError:
	    print('Cannot find PiCAN board.')
	    GPIO.output(led,False)
	    exit()
def CAN_TxHeaderTypeDef(self):
    IDE = 0
    StdId = 0
    RTR = 0
    DLC = 0
    TransmitGlobalTime

def BSP_CAN_FillTxMailbox(_pdata):
    _header = CAN_TxHeaderTypeDef()
    _header.IDE = CAN_ID_STD
    _header.StdId = ((uint32_t)(_pdata[2] & 0x07)) << 8
    if (_pdata[2] & 0x04):
         _header.RTR = CAN_RTR_REMOTE
    else:
        _header.RTR = CAN_RTR_DATA                 
    _header.DLC   = (_pdata[4] & 0x0f)
    if _pdata[0] == 0x01:
    can.Message()    
    return retval