import threading, time 
import Queue
import pi_can as can_bus
import can
import pi_led as led
import sys
# First install Serial sudo apt-get install python-serial
import serial
port = serial.Serial("/dev/ttyAMA0", baudrate=115200, timeout=3.0)
uTxMQ = Queue.Queue(maxsize=0) 
cTxMQ = Queue.Queue(maxsize=0) 
localMQ = Queue.Queue(maxsize=0) 
devInfoMutex = threading.Lock()
dcbMiInfoMutex = threading.Lock()
mcbInfoMutex = threading.Lock()
uRxSemphore = threading.Semaphore()
cRxF0Semphore = threading.Semaphore()
cRxF1Semphore = threading.Semaphore()
PWA11_FCC_MASK = 0xE0
PWA11_FCC_CMSG = 0x00
PWA11_FCC_LUMSG = 0x20
PWA11_PT_MASK = 0x03
PWA11_PT_FCF = 0x00
PWA11_PT_RCF = 0x01
PWA11_PT_NMT = 0x02
PWA11_PT_MSF = 0x03
uRxBuf = []
Default_DCBNodeEnable = [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
Default_BroadcastInterval = 10; #/*unit is ms*/
Default_HeatbeatPeriod   = 50; #/* uint is 100ms */
def DefaultTskFunc():
    can_bus.devInfoMgt.mcbInfo.devStatus = can_bus.Node_ST.NST_CScanning
    if not can_bus.BSP_CAN_Scan_BroadCast():
        print("Broadcast Error!")
    else:
        if can_bus.numNodeOffline == 0:
            can_bus.devInfoMgt.mcbInfo.devStatus = can_bus.Node_ST.NST_CScanCplt
        else:
            can_bus.devInfoMgt.mcbInfo.devStatus = can_bus.Node_ST.NST_CScanUCplt
    #   led.BSP_LED2_ON()
    #    time.sleep(0.001)
    #2. Obtain nodes basic-information And Initialize buffer
    can_bus.BSP_CAN_FreshLLD()
    #allocate memory for DCB(LLDs) mirror state*/
    #BSP_DevInfoMgt_AllocateDCBs() 
    if not can_bus.BSP_CAN_StartHB_BroadCast():
        print('Error Can Start Heart Beat BroadCast Error!')
 #   cNMTimer = threading.Timer(200 * can_bus.devInfoMgt.hbPeriod/1000, cNMT_Callback)
 #   cNMTimer.start()
  #  /* 4. Connect with GUI Machine and Shake hands*/
  #/*Connect should be launched by GUI Machine. And before that, MCB will wait to connect. */
 #   uRxBuf = port.read(17) #Read 17 byte from uart
 #   print('Start to connect with GUI Machine...\r\n')
 #   can_bus.devInfoMgt.mcbInfo.devStatus = can_bus.GUI_ST.NST_WaitGUI
 #   cnctGUIFlag = 0 
 #   while cnctGUIFlag == 0:
 #       led.BSP_STLED_Toggle()
 #       time.sleep(0.5)
 #   can_bus.devInfoMgt.mcbInfo.guiStatus = can_bus.GUI_ST.GUI_ST_Cnct
 #   #/*start to wait shakehands message from GUI Machine*/
 #   while cnctGUIFlag < 2:
 #       led.BSP_STLED_Toggle()
 #       time.sleep(0.1)
  #  can_bus.devInfoMgt.mcbInfo.guiStatus = can_bus.GUI_ST.GUI_ST_ShakeHand
  #  led.BSP_STLED_ON()
    can_bus.devInfoMgt.mcbInfo.devStatus = can_bus.Node_ST.NST_Working
    while True:
        time.sleep(0.1)
#> |0x11|IDE|0x0A0 or 0x1A0 |DATA|0~8|No Meaning| Turn on BlueLight
#> |0x11|IDE|0x0A2 or 0x1A2 |DATA|0~8|No Meaning| Turn off BlueLight
data_on = [0x11,0,0,0xA0,0,1] #turn on led
data_off = [0x11,0,0,0xA2,0,1]#Turn off led
def uRxTskFunc():
    while True:
        uRxSemphore.acquire()
        cTxMQ.put(data_on)
        time.sleep(1)
        cTxMQ.put(data_off)
        time.sleep(1)
        #if parseU2Rx() != 0: 
        #    if uRxBuf[2] == 0x11:
        #        if uRxBuf[4] & PWA11_FCC_MASK == PWA11_FCC_CMSG:
        #            cTem_Buf = []
        #            for i in range(0,13):
        #                cTem_Buf.append(uRxBuf[i+2])
        #            cTxMQ.put(cTem_Buf)
        #        if uRxBuf[4] & PWA11_FCC_MASK == PWA11_FCC_LUMSG:
        #            cTem_Buf = []
        #            for i in range(0,13):
        #                cTem_Buf.append(uRxBuf[i+2])
        #           localMQ.put(cTem_Buf)
        
def cRxTaskFunc():
    _dBuf = []
    for i in range(0,8):
        _dBuf.append(0)
    while True:
        cRxF1Semphore.acquire()
        message = can_bus.bus.recv()	# Wait until a message is received.
        _cmdCode = message.arbitration_id & 0xFF #Std ID
        _len     = message.dlc & 0x0F
        _nodeId  = message >> 4
        _cmdCode &= 0x0F
        _dBuf = message.data
        if _cmdCode == 2:
            print('Receive Response! Data is: {}'.format(_dBuf[5]))
def cRxF0TskFunc():
    _dBuf = []
    for i in range(0,8):
        _dBuf.append(0)
    p2ar = []
    for i in range(0,15):
        p2ar.append(0)
    _rHeader = can_bus.CAN_RxHeaderTypeDef()
    cRxF0Semphore.acquire()
    _rHeader = can_bus.bus.recv()
    p2ar[6]       = _rHeader.DLC & 0x0F
    p2ar[5]       = _rHeader.StdId & 0xFF
    p2ar[4]       = (_rHeader.StdId >> 8) & 0x07
    if _rHeader.IDE == can_bus.CAN_ID_EXT:
        p2ar[4] |= 0x08
    if _rHeader.RTR == can_bus.CAN_RTR_REMOTE:
        p2ar[4] |= 0x10
        for i in range(0,8):
            p2ar[i+7] = _dBuf[i]
    uTxMQ.put(p2ar) 
def cRxF1TskFunc():
    _dBuf = []
    for i in range(0,8):
        _dBuf.append(0)
    while True:
        cRxF1Semphore.acquire()
        message = can_bus.bus.recv()	# Wait until a message is received.
        _cmdCode = message.arbitration_id & 0xFF #Std ID
        _len     = message.dlc & 0x0F
        _nodeId  = message >> 4
        _cmdCode &= 0x0F
        _dBuf = message.data
    #Show debug here 
        _frameType = (message.arbitration_id >> 8) & 0x07
        if _frameType != 2:
            print('maybe CAN file setup error\r\n')
    #/*setup the specific node heart beat flag. Once find any frames from the specific nodes*/
        if _nodeId > 0 and _nodeId != 15:
             #/*set heartbeat flag*/
            if can_bus.devInfoMgt.dNodeHBFlag[_nodeId - 1] < 0xFF:
                can_bus.devInfoMgt.dNodeHBFlag[_nodeId - 1] += 1
            #/*should add a wr-lock in case of memory conflict*/
            #/*update node information struct*/
            if _cmdCode == 0:

                 #/*update basic information*/
                if _len >=6:
                    can_bus.devInfoMgt.dcbInfo[_nodeId].basicInfo.busStatus = _dBuf[0]
                    can_bus.devInfoMgt.dcbInfo[_nodeId].basicInfo.devStatus = _dBuf[1]
                    can_bus.devInfoMgt.dcbInfo[_nodeId].basicInfo.foreTask = _dBuf[2]
                    can_bus.devInfoMgt.dcbInfo[_nodeId].basicInfo.foreTaskSt = _dBuf[3]
                    can_bus.devInfoMgt.dcbInfo[_nodeId].basicInfo.fTskStatus[0] = _dBuf[4]
                    can_bus.devInfoMgt.dcbInfo[_nodeId].basicInfo.fTskStatus[1] = _dBuf[5]
            elif _cmdCode == 1:
            #/*fill the item record zone no.1*/
                if _len == 8:
                    _checkSum = 0
                    for i in range(0,7):
                        _checkSum += _dBuf[i]
                    if _checkSum == _dBuf[7]:
                        can_bus.devInfoMgt.dcbInfo[_nodeId].sumLen = _dBuf[0]
                        for j in range(0,6):
                            can_bus.devInfoMgt.dcbInfo[_nodeId].itemRcd[j].size = _dBuf[1 + j]
            elif _cmdCode == 2:
            #/*fill the item record zone no.2 */
                if _len == 8:
                    _checkSum = 0
                    for i in range(0,7):
                        for j in range(0,7):
                            can_bus.devInfoMgt.dcbInfo[_nodeId].itemRcd[6 + j].size = _dBuf[j]
            else:
            #/*update the information at mirror buffer*/
                if _len == can_bus.devInfoMgt.dcbInfo[_nodeId].itemRcd[_cmdCode].size:
                    for i in range(0,_len):
                        can_bus.devInfoMgt.dcbInfo[_nodeId].addStart.insert(can_bus.devInfoMgt.dcbInfo[_nodeId].itemRcd[_cmdCode].offset + i,_dBuf[i]) 
def uTxTskFunc():
    p2mq = can_bus.uTxMQTypedef
    while True:
        if not uTxMQ.empty(): 
            p2mq = uTxMQ.get()
            print('Receive data from FIFO 0:{}'.format(p2mq))
            #BSP_PackUTxMsg(p2mq)
def cTxTskFunc():
    p2mq = can_bus.cTxMQTypedef
    while True:
        if not cTxMQ.empty():  
            p2mq = cTxMQ.get()
            print('Send out data:{}'.format(p2mq))
            if can_bus.BSP_CAN_FillTxMailbox(p2mq) == 1:
                print('Send out data success!')
            else:
                print('Send out data fail')
        time.sleep(0.001)
def localTskFunc():
    p2mq = can_bus.localMQTypedef
    while True:
        if not localMQ.empty():
            p2mq = localMQ.get()
        time.sleep(0.001)
def cNMT_Callback():
    #Define code here
    while True:
        time.sleep(1)

def KernelStart():    
    t = threading.Thread(target = DefaultTskFunc)
    t.start()
    #t2 = threading.Thread(target = uRxTskFunc)
    #t2.start()
    #t3 = threading.Thread(target = cRxF0TskFunc)
    #t3.start()
    t4 = threading.Thread(target = cRxF1TskFunc)
    t4.start()
    #t5 = threading.Thread(target = uTxTskFunc)
    #t5.start()
    t6 = threading.Thread(target = cTxTskFunc)
    t6.start()
    #t7 = threading.Thread(target = localTskFunc)
    #t7.start()

def BSP_DevInfoMgt_Init():
    #/*setup information*/
    can_bus.devInfoMgt.broadInterval = Default_BroadcastInterval
    can_bus.devInfoMgt.hbPeriod      = Default_HeatbeatPeriod
    for i in range(0,14):
        can_bus.devInfoMgt.dNodeEnable.insert(i,Default_DCBNodeEnable[i])
        can_bus.devInfoMgt.dNodeOnline.insert(i,0) # /* offline now */
        can_bus.devInfoMgt.dNodeHBFlag.insert(i,0)
    #/*Initial MCB status*/
    can_bus.devInfoMgt.mcbInfo.devStatus     = 1# /* 1- Init*/
    can_bus.devInfoMgt.mcbInfo.canNodeStatus = 0# /* haven't got any DCB nodes*/
    can_bus.devInfoMgt.mcbInfo.guiStatus     = 0# /* haven't connect with GUI Machine */
    can_bus.devInfoMgt.mcbInfo.foreTask      = 0# /* no any task */
    can_bus.devInfoMgt.mcbInfo.fTskStatus.insert(0,0)# /* no task*/
    can_bus.devInfoMgt.mcbInfo.fTskStatus.insert(1,0)# /* no task*/

    #/*Initial DCB Mirror status*/
    for j in range(0,14):
        can_bus.devInfoMgt.dcbInfo[j].sumLen   = 0

        can_bus.devInfoMgt.dcbInfo[j].basicInfo.busStatus = 0 #/* offline */
        can_bus.devInfoMgt.dcbInfo[j].basicInfo.devStatus = 0 #/* reset */
        can_bus.devInfoMgt.dcbInfo[j].basicInfo.foreTask  = 0 #/* no task */

        can_bus.devInfoMgt.dcbInfo[j].basicInfo.fTskStatus.insert(0,0)
        can_bus.devInfoMgt.dcbInfo[j].basicInfo.fTskStatus.insert(1,0)

        for k in range(0,13):
            can_bus.devInfoMgt.dcbInfo[j].itemRcd[k].offset = 0
            can_bus.devInfoMgt.dcbInfo[j].itemRcd[k].size   = 0
        #/*Calloc memory for MCB*/
        # /*1. check if the length is same*/
        temp = 0
        for k in range(0,13):
            temp +=  can_bus.devInfoMgt.mcbMemMgt.itemRcd[k].size

        if temp !=  can_bus.devInfoMgt.mcbMemMgt.sumLen: 
            print("MCB Memory configration error\r\n")

 #           Error_Handler()
 # }
 #   devInfoMgt.mcbMemMgt.addStart = (uint8_t *)calloc(temp, sizeof(uint8_t));
        for i in range(0,temp):
            can_bus.devInfoMgt.mcbMemMgt.addStart.insert(i,0)
 # if (devInfoMgt.mcbMemMgt.addStart == NULL)
 #   Error_Handler();
 # /*format position*/
    temp = 0
    for k in range(0,13):
        can_bus.devInfoMgt.mcbMemMgt.itemRcd[k].offset = temp
        can_bus.devInfoMgt.mcbMemMgt.itemRcd[k].size
#Init 
BSP_DevInfoMgt_Init()
can_bus.devInfoMgt.mcbInfo.devStatus = can_bus.Node_ST.NST_Ready       
#can_bus.devInfoMgt.mcbInfo.guiStatus = can_bus.GUI_ST_RESET
can_bus.devInfoMgt.mcbInfo.devStatus = can_bus.Node_ST.NST_CReady
KernelStart()    
