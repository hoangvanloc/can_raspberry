import threading, time
import pi_can as can_bus
import can
import pi_led as led
# First install Serial sudo apt-get install python-serial
import serial
port = serial.Serial("/dev/ttyAMA0", baudrate=115200, timeout=3.0)
uTxMQ = Queue.Queue() 
cTxMQ = Queue.Queue() 
localMQ = Queue.Queue() 
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
#cNMTimer = threading.Timer(2.0, cNMT_Callback) /*In original => osTimerPeriodic, Here we set is 2 seconds*/
def DefaultTskFunc(self):
    devInfoMgt.mcbInfo.devStatus = NST_CScanning
    if not can_bus():
        print("Brocast Error!")
    else:
        if can_bus.numNodeOffline == 0:
            devInfoMgt.mcbInfo.devStatus = NST_CScanCplt
        else:
            devInfoMgt.mcbInfo.devStatus = NST_CScanUCplt
        led.BSP_LED2_ON()
        time.sleep(100)
    #2. Obtain nodes basic-information And Initialize buffer
    can_bus.BSP_CAN_FreshLLD()
    #allocate memory for DCB(LLDs) mirror state*/
    #  BSP_DevInfoMgt_AllocateDCBs() => we don't have document to port*/
    if not can_bus.BSP_CAN_StartHB_BroadCast():
        print('Error Can Start Heart Beat BroadCas Error!')
    cNMTimer = time.Timer(200 * devInfoMgt.hbPeriod/1000, cNMT_Callback)
    cNMTimer.start()
  #  /* 4. Connect with GUI Machine and Shake hands*/
  #/*Connect should be launched by GUI Machine. And before that, MCB will wait to connect. */
    uRxBuf = port.read(17) #Read 17 byte from uart
    print('Start to connect with GUI Machine...\r\n')
    devInfoMgt.mcbInfo.devStatus = NST_WaitGUI
    cnctGUIFlag                  = 0; 
    while cnctGUIFlag == 0:
         led.BSP_STLED_Toggle()
         time.sleep(500)
    devInfoMgt.mcbInfo.guiStatus = GUI_ST_Cnct
    #/*start to wait shakehands message from GUI Machine*/
    while cnctGUIFlag < 2:
         led.BSP_STLED_Toggle()
         time.sleep(100)
    devInfoMgt.mcbInfo.guiStatus = GUI_ST_ShakeHand
    led.BSP_STLED_ON()
    devInfoMgt.mcbInfo.devStatus = NST_Working
    while True:
        time.sleep(100)
def uRxTskFunc(self):
    while True:
        uRxSemphore.acquire()
        if parseU2Rx() != 0: #We cannot find this function
            if uRxBuf[2] == 0x11:
                if uRxBuf[4] & PWA11_FCC_MASK == PWA11_FCC_CMSG:
                    cTem_Buf = []
                    for i in range(0,13):
                        cTem_Buf[i] = uRxBuf[i+2]
                    cTxMQ.put(cTem_Buf)
                if uRxBuf[4] & PWA11_FCC_MASK == PWA11_FCC_LUMSG:
                    cTem_Buf = []
                    for i in range(0,13):
                        cTem_Buf[i] = uRxBuf[i+2]
                    localMQ.put(cTem_Buf)
def cRxF0TskFunc(self): 
    _dBuf=[]
    p2ar = [15]
    cRxF0Semphore.acquire()
    can_massage = can_bus.bus.recv()
    p2ar[6]       = can_massage.dlc & 0x0F
    p2ar[5]       = can_massage.arbitration_id & 0xFF
    p2ar[4]       = ((can_massage.arbitration_id >> 8) & 0x07)
    if _rHeader.IDE == CAN_ID_EXT:
        p2ar[4] |= 0x08
    if _rHeader.RTR == CAN_RTR_REMOTE:
        p2ar[4] |= 0x10
        for i in range(0,8):
            p2ar[i+7] = _dBuf[i]
    uTxMQ.put(p2ar) 
def cRxF1TskFunc(self):
    _dBuf=[]
    _rHeader=CAN_RxHeaderTypeDef()
    while True:
        cRxF1Semphore.acquire()
        _rHeader = can_bus.bus.recv()	# Wait until a message is received.
        _dBuf = _rHeader.data
        _cmdCode = _rHeader.arbitration_id & 0xFF
        _len     = _rHeader.dlc & 0x0F
        _nodeId  = _cmdCode >> 4
        _cmdCode &= 0x0F
    #Show debug here
        _frameType = (_rHeader.arbitration_id >> 8) & 0x07
        if _frameType != 2:
            print('maybe CAN file setup error\r\n')
    #/*setup the specific node heart beat flag. Once find any frames from the specific nodes*/
        if _nodeId > 0 and _nodeId != 15:
            if devInfoMgt.dNodeHBFlag[_nodeId - 1] < 0xFF:
                devInfoMgt.dNodeHBFlag[_nodeId - 1] += 1
            if _cmdCode == 0:
                if _len >=6:
                    devInfoMgt.dcbInfo[_nodeId].basicInfo.busStatus = _dBuf 
            elif _cmdCode == 1:
                if _len == 8:
                    _checkSum = 0
                    for i in range(0,7):
                        _checkSum += _dBuf[i]
                    if _checkSum == _dBuf[7]:
                        devInfoMgt.dcbInfo[_nodeId].sumLen = _dBuf[0]
                        for j in range(0,6):
                            devInfoMgt.dcbInfo[_nodeId].itemRcd[j].size = _dBuf[1 + j]
            elif _cmdCode == 2:   
                if _len == 8:
                    _checkSum = 0
                    for i in range(0,7):
                        for j in range(0,7):
                            devInfoMgt.dcbInfo[_nodeId].itemRcd[6 + j].size = _dBuf[j]
            else:
                if _len == devInfoMgt.dcbInfo[_nodeId].itemRcd[_cmdCode].size:
                    for i in range(0,_len):
                        devInfoMgt.dcbInfo[_nodeId].addStart[ devInfoMgt.dcbInfo[_nodeId].itemRcd[_cmdCode].offset + i] = _dBuf[i]
def uTxTskFunc(self):
    while True:
        if not uTxMQ.empty(): 
            p2m1 = uTxMQ.get()
            # BSP_PackUTxMsg(p2mq)  ? where is this funtion?
def cTxTskFunc(self):
    while True:
        if not cTxMQ.empty():
            p2mq = cTxMQ.get()
        can_bus.BSP_CAN_FillTxMailbox(p2mq)
        time.sleep(1)
def localTskFunc(self):
    while True:
        if not localMQ.empty():
            p2mq = localMQ.get()
        time.sleep(1)
def cNMT_Callback(self):
    #Define code here


if __name__ == "__main__":
    led.LED_Init()
    can_bus.BSP_CAN_Init()
    t = threading.Thread(target = DefaultTskFunc)
    t.start()
    t2 = threading.Thread(target = uRxTskFunc)
    t2.start()
    t3 = threading.Thread(target = cRxF0TskFunc)
    t3.start()
    t4 = threading.Thread(target = cRxF1TskFunc)
    t4.start()
    t5 = threading.Thread(target = uTxTskFunc)
    t5.start()
    t6 = threading.Thread(target = cTxTskFunc)
    t6.start()
    t7 = threading.Thread(target = localTskFunc)
    t7.start()
    while True:
        

    