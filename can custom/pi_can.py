import can
import time
import enum
numNodeOffline = 0

CAN_ID_EXT = True
CAN_RTR_REMOTE = True

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
class Node_ST(enum.Enum):
    NST_Init       = 0
    NST_Ready      = 1
    NST_CReady     = 2
    NST_CScanning  = 3
    NST_CScanError = 4     #/*CAN haven't gotten any response*/
    NST_CScanUCplt = 5     #/* CAN got responses, but incomplete */
    NST_CScanCplt  = 6     #/* CAN have find all nodes*/
    NST_WaitGUI    = 8     #/* wait GUI connect ok*/
    NST_Working    = 16    #/* Start to work*/
    NST_CError     = 0x0E  #/* CAN bus error*/
    NST_HWError    = 0xE0  #/* MCB hardware error */
    NST_Reset      = 0xA0  #/* MCB force to reset itself */
class CAN_ST(enum.Enum):
    CAN_NODE_RESET   = 0
    CAN_NODE_ERROR   = 1
    CAN_NODE_NOReply = 8
    CAN_NODE_OK      = 16
    CAN_NODE_ISOLATE = 3

class GUI_ST(enum.Enum):
    GUI_ST_RESET     = 0
    GUI_ST_LostCnct  = 1
    GUI_ST_Cnct      = 2
    GUI_ST_ReCnct    = 4
    GUI_ST_ShakeHand = 8   
class Node_BasicInfo():
    busStatus = 0    #/* offline/online and enable/isolate */
    devStatus = 0     #/* running, stop, error, update or other status*/
    foreTask = 0      #/* the type/id of task which is running(foreground task)*/
    foreTaskSt = 0    #/*the fore task is running or stop*/ 
    fTskStatus = {0,0}#List contain unsigned char #/* foreTask state: circle/phase*/
class MCB_BasicInfo():
    guiStatus  = 0   # /* the connect situtus with GUI Machine(by UART now) */
    canNodeStatus  = 0# /* the  number of DCB nodes online */
    devStatus = 0    #/* running, stop, error, update or other status*/
    foreTask = 0     #/* the type/id of task which is running(foreground task)*/
    foreTaskSt = 0    #/*the fore task is running or stop*/ 
    fTskStatus = {0,0} #list contain unsigned char #/* foreTask state: circle/phase*/
class Node_ItemRecord():
    offset = 0
    size = 0
class MCB_MemMgt():
    # uint8_t *addStart;
    sumLen = 0
    itemRcd = [] #list contain Node_ItemRecord_Typedef
class Node_Info():
    #int8_t *addStart;             # /*Starting address of Node Information Buffer*/
    sumLen = 0                    # /*the total size of Node Information Buffer*/
    basicInfo = Node_BasicInfo()    #/*the basic information of node*/
    itemRcd = [] # list contain Node_ItemRecord  #/*the information record for all nodes*/
class DeviceInforManagerment():
    dNodeEnable = []    #List contain Node Enable
    dNodeOnline = []    #List contain Node Online
    broadInterval = 1000
    hbPeriod = 1000
    dNodeHBFlag = []
    dcbInfo = [] #list contain Node_Info()
    mcbInfo = MCB_BasicInfo()
    mcbMemMgt = MCB_MemMgt()

   
devInfoMgt = DeviceInforManagerment()
#Need to define 2 paramter CAN_RTR_REMOTE and CAN_RTR_DATA

bus = can.interface.Bus(channel='can0', bustype='socketcan_native')


def BSP_CAN_FillTxMailbox(_pdata = []):
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
    #/*1. find header file */
    _txHeader = CAN_TxHeaderTypeDef()
    _txHeader.StdId = 0x2f0
    _txHeader.ExtId = 0
    _txHeader.IDE   = False
    _txHeader.RTR   = True
    _txHeader.DLC   = 6
    _txHeader.TransmitGlobalTime = 0.0
    #/*2. send out broadcast message*/
    msg = can.Message(is_remote_frame=_txHeader.RTR,arbitration_id=_txHeader.StdId,data=temp,dlc=_txHeader.DLC,extended_id=False)    
    retval = bus.send(msg)
    if retval < 0:
        return 0
    #/*3. delay and wait. In this stage, task will automaticess rx message */
    time.sleep(200 + devInfoMgt.broadInterval * 14) # /*The total wait time and 200ms delay*/
    
    for j in range(0,14):
        if (devInfoMgt.dNodeEnable[j] != 0):
            if (devInfoMgt.dNodeOnline[j] >= CAN_ST.CAN_NODE_OK):
                devInfoMgt.mcbInfo.canNodeStatus += 1
            else:
                _txHeader.StdId = j
                msg = can.Message(is_remote_frame=_txHeader.RTR,arbitration_id=_txHeader.StdId,data=temp,dlc=_txHeader.DLC,extended_id=False)    
                retval = bus.send(msg)
                if retval < 0:
                    continue
                time.sleep(0.15)
                if (devInfoMgt.dNodeOnline[j] >= CAN_ST.CAN_NODE_OK):
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
                time.sleep(0.005)
                _timeOutNum -= 1
            if _timeOutNum == 0:
                devInfoMgt.dcbInfo[i].basicInfo.busStatus = CAN_ST.CAN_NODE_NOReply #/*Error flag*/
                continue
            _txHeader.StdId = (i << 4) | 0x202
            canCmdIdFlag = _txHeader.StdId | 0x400 #/*equel to the receive message id*/
            msg = can.Message(is_remote_frame= _txHeader.RTR,arbitration_id=_txHeader.StdId,data=_temp,dlc=_txHeader.DLC,extended_id=False)   
            retval = bus.send(msg)
            _timeOutNum = 10
            while (canCmdIdFlag != 0) and (_timeOutNum != 0):
                time.sleep(0.005)
                _timeOutNum -= 1
            if _timeOutNum == 0:
                devInfoMgt.dcbInfo[i].basicInfo.busStatus = CAN_ST.CAN_NODE_NOReply# /*Error flag*/
    if retval < 0:
        return 0
    return 1

