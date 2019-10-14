import can
import thread
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
    msg = can.Message(arbitration_id=_header.StdId,data=_pdata[5],extended_id=False)    
    retval = bus.send(msg)
    return retval
def BSP_CAN_Scan_BroadCast(self):
    temp[8]
    _txHeader = CAN_TxHeaderTypeDef()
      _txHeader.StdId = 0x2f0
    _txHeader.ExtId = 0x00
    _txHeader.IDE   = CAN_ID_STD
    _txHeader.RTR   = CAN_RTR_REMOTE
    _txHeader.DLC   = 6
    _txHeader.TransmitGlobalTime = DISABLE
    msg = can.Message(arbitration_id=_txHeader.StdId,data=temp,extended_id=False)    
    retval = bus.send(msg)
    if not retval:
        return 0
    thread.sleep(200 + devInfoMgt.broadInterval * 14)
    for j in range(0,14):
        if (devInfoMgt.dNodeEnable[j] != 0):
            if (devInfoMgt.dNodeOnline[j] >= CAN_NODE_OK):
                devInfoMgt.mcbInfo.canNodeStatus += 1
            else:
                _txHeader.StdId = j
                msg = can.Message(arbitration_id=_txHeader.StdId,data=temp,extended_id=False)    
                retval = bus.send(msg)
                if not retval:
                    continue
                thread.sleep(150)
                if (devInfoMgt.dNodeOnline[j] >= CAN_NODE_OK):
                    devInfoMgt.mcbInfo.canNodeStatus += 1
                else:
                    numNodeOffline += 1
def BSP_CAN_StartHB_BroadCast(self):
    _temp[8] = {0}
    _txHeader = CAN_TxHeaderTypeDef()
    /*1. find header file */
    _txHeader.StdId = 0x2f0
    _txHeader.ExtId = 0x00
    _txHeader.IDE   = CAN_ID_STD
    _txHeader.RTR   = CAN_RTR_DATA
    _txHeader.DLC   = 2; /*ask for 6 data from DCB nodes*/
    _txHeader.TransmitGlobalTime = DISABLE
    _temp[0] = (uint8_t)devInfoMgt.hbPeriod
    _temp[1] = (uint8_t)(devInfoMgt.hbPeriod >> 8)
    msg = can.Message(arbitration_id=_header.StdId,data=_pdata[5],extended_id=False)    
    retval = bus.send(msg)
    return retval
def BSP_CAN_FreshLLD(def):
    temp[8]
    _txHeader = CAN_TxHeaderTypeDef()
    /* 0: prepare message*/
    _txHeader.StdId = 0x000
    _txHeader.ExtId = 0x00
    _txHeader.IDE   = CAN_ID_STD
    _txHeader.RTR   = CAN_RTR_REMOTE
    _txHeader.DLC   = 8; /*ask for 6 data from DCB nodes*/

    _txHeader.TransmitGlobalTime = DISABLE
    for i in range(0,14):
        if evInfoMgt.dNodeOnline[i] != 0:
             /* 1. send instruction No.1*/
            _txHeader.StdId = (i << 4) | 0x201
            canCmdIdFlag = _txHeader.StdId | 0x400; /*equel to the receive message id*/
            msg = can.Message(arbitration_id=_txHeader.StdId,data=temp,extended_id=False)    
            retval = bus.send(msg) 
            _timeOutNum = 10
            while (canCmdIdFlag != 0) and (_timeOutNum != 0):
                thread.sleep(5)
                _timeOutNum -= 1
            if _timeOutNum == 0:
                devInfoMgt.dcbInfo[i].basicInfo.busStatus = CAN_NODE_NOReply; /*Error flag*/
                continue
            _txHeader.StdId = (i << 4) | 0x202
            canCmdIdFlag = _txHeader.StdId | 0x400; /*equel to the receive message id*/
            msg = can.Message(arbitration_id=_txHeader.StdId,data=temp,extended_id=False)    
            retval = bus.send(msg)
             _timeOutNum = 10
            while (canCmdIdFlag != 0) and (_timeOutNum != 0):
                thread.sleep(5)
                _timeOutNum -= 1
            if _timeOutNum == 0:
                devInfoMgt.dcbInfo[i].basicInfo.busStatus = CAN_NODE_NOReply; /*Error flag*/