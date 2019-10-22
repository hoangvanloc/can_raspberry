import can
import time
numNodeOffline = 0
CAN_NODE_OK = 1
CAN_NODE_NOReply = 0
class CAN_TxHeaderTypeDef():
    IDE = False      #is extended_id? true for Use extend ID
    StdId = 0        #arbitration_id
    ExtId = 0        #Extend ID. If IDE = true.
    RTR = False      #is_remote_frame
    DLC = None       #Remote frames can only be transmitted with a data length code (DLC) identical to the DLC of the corresponding data frame
    TransmitGlobalTime = 0.0#Timestamp
class CAN_RxHeaderTypeDef():
    IDE = False      #is extended_id? true for Use extend ID
    StdId = 0        #arbitration_id
    ExtId = 0        #Extend ID. If IDE = true.
    RTR = False      #is_remote_frame
    DLC = None       #Remote frames can only be transmitted with a data length code (DLC) identical to the DLC of the corresponding data frame
    TransmitGlobalTime = 0.0#timestamp
class BasicInfor:
    busStatus = []
class DCBInfor():
    basicInfo = BasicInfor()
    addStart = []
    itemRcd = []
    sumLen = 0
class MCBInfor():
    canNodeStatus = 0
class DeviceInforManager():
    dNodeEnable = []
    dNodeOnline = []
    dNodeHBFlag = []
    dcbInfo = DCBInfor()
    mcbInfo = MCBInfor()
    hbPeriod = 1000
devInfoMgt = DeviceInforManager()
#Need to define 2 paramter CAN_RTR_REMOTE and CAN_RTR_DATA

bus = can.interface.Bus(channel='can0', bustype='socketcan_native')


def BSP_CAN_FillTxMailbox(_pdata):
    retval = 0
    _header = CAN_TxHeaderTypeDef()
    _header.IDE = False 
    _header.StdId = (_pdata[2] & 0x07) << 8
    if (_pdata[2] & 0x04):
         _header.RTR = True
    else:
        _header.RTR = False                 
    _header.DLC   = (_pdata[4] & 0x0f)
    if _pdata[0] == 0x01:
        msg = can.Message(is_remote_frame=_header.RTR,arbitration_id=_header.StdId,data=_pdata[5],extended_id=_header.IDE,dlc=_header.DLC)    
        retval = bus.send(msg)
    if retval < 0:
        return 0
    return 1
def BSP_CAN_Scan_BroadCast(self):
    temp = []
    _txHeader = CAN_TxHeaderTypeDef()
    _txHeader.StdId = 0x2f0
    _txHeader.ExtId = 0
    _txHeader.IDE   = False
    _txHeader.RTR   = True
    _txHeader.DLC   = 6
    _txHeader.TransmitGlobalTime = 0.0
    msg = can.Message(is_remote_frame=_txHeader.RTR,arbitration_id=_txHeader.StdId,data=temp,dlc=_txHeader.DLC,extended_id=False)    
    retval = bus.send(msg)
    if retval < 0:
        return 0
    time.sleep((200+14)/1000)
    
    for j in range(0,14):
        if (devInfoMgt.dNodeEnable[j] != 0):
            if (devInfoMgt.dNodeOnline[j] >= CAN_NODE_OK):
                devInfoMgt.mcbInfo.canNodeStatus += 1
            else:
                _txHeader.StdId = j
                msg = can.Message(is_remote_frame=_txHeader.RTR,arbitration_id=_txHeader.StdId,data=temp,dlc=_txHeader.DLC,extended_id=False)    
                retval = bus.send(msg)
                if retval < 0:
                    continue
                time.sleep(150/1000)
                if (devInfoMgt.dNodeOnline[j] >= CAN_NODE_OK):
                    devInfoMgt.mcbInfo.canNodeStatus += 1
                else:
                    numNodeOffline += 1
    if retval < 0:
        return 0
    return 1
def BSP_CAN_StartHB_BroadCast(self):
    _temp = []
    _txHeader = CAN_TxHeaderTypeDef()
    #/*1. find header file */
    _txHeader.StdId = 0x2f0
    _txHeader.ExtId = 0x00
    _txHeader.IDE   = False
    _txHeader.RTR   = False
    _txHeader.DLC   = 2 #/*ask for 6 data from DCB nodes*/
    _txHeader.TransmitGlobalTime = 0.0
    _temp[0] = devInfoMgt.hbPeriod
    _temp[1] = (devInfoMgt.hbPeriod >> 8)
    msg = can.Message(arbitration_id=_txHeader.StdId,data=_temp,dlc=_txHeader.DLC,extended_id=False)    
    retval = bus.send(msg)
    return retval
def BSP_CAN_FreshLLD(self):
    _temp =  []
    _txHeader = CAN_TxHeaderTypeDef()
    #/* 0: prepare message*/
    _txHeader.StdId = 0
    _txHeader.ExtId = False
    _txHeader.IDE   = False
    _txHeader.RTR   = True
    _txHeader.DLC   = 8 #/*ask for 6 data from DCB nodes*/
    _txHeader.TransmitGlobalTime = 0.0
    for i in range(0,14):
        if devInfoMgt.dNodeOnline[i] != 0:
            #/* 1. send instruction No.1*/
            _txHeader.StdId = (i << 4) | 0x201
            canCmdIdFlag = _txHeader.StdId | 0x400# /*equel to the receive message id*/
            msg = can.Message(is_remote_frame= _txHeader.RTR,arbitration_id=_txHeader.StdId,data=_temp,dlc=_txHeader.DLC,extended_id=False) 
            retval = bus.send(msg) 
            _timeOutNum = 10
            while (canCmdIdFlag != 0) and (_timeOutNum != 0):
                time.sleep(5)
                _timeOutNum -= 1
            if _timeOutNum == 0:
                devInfoMgt.dcbInfo[i].basicInfo.busStatus = CAN_NODE_NOReply #/*Error flag*/
                continue
            _txHeader.StdId = (i << 4) | 0x202
            canCmdIdFlag = _txHeader.StdId | 0x400 #/*equel to the receive message id*/
            msg = can.Message(is_remote_frame= _txHeader.RTR,arbitration_id=_txHeader.StdId,data=_temp,dlc=_txHeader.DLC,extended_id=False)   
            retval = bus.send(msg)
            _timeOutNum = 10
            while (canCmdIdFlag != 0) and (_timeOutNum != 0):
                time.sleep(5)
                _timeOutNum -= 1
            if _timeOutNum == 0:
                devInfoMgt.dcbInfo[i].basicInfo.busStatus = CAN_NODE_NOReply# /*Error flag*/
    if retval < 0:
        return 0
    return 1

