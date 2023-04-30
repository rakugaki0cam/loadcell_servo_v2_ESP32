#
# Electric Tama Bou
#  Nukidan force measurement
#
# loadcell   HX711  500gf
# servo motor SG92R
#
# ESP32
#
# 2022.11.27
# 2023.01.05            SDcard add
# 2023.03.25            No SDcard  Build board（some GPIO pin position　change）
# 2023.03.26 ver.2.30   re-add SDcard
# 2023.03.27 ver.2.31   Get Time from WiFi -> SDcard time stamp
# 2023.03.28 ver.2.32   filename <- date&time add
# 2023.03.29 ver.2.32b  delete Japanese Font ->  can not use in pymakr
# 2023.04.01 ver.2.33   SD data .txt -> .csv
# 2023.04.15 ver.2.34   measure x:7.05mm -> 7.72mm
# 2023.04.24 ver.2.35   measure x:7.72mm -> 8.50mm
# 2023.04.29 ver.2.40   Stack M5 CORE2へUART送信してグラフと荷重を表示
#
#


import math
import uos
import ntptime
import utime
from machine import Pin, PWM, SDCard, RTC, UART
import wifiid       # wifiid.py - WIFI_SS_ID, WIFI_PASSWORD

# PIN init
#servo
servo1 = PWM(Pin(4), freq=50)   # PWM freq 1~1000Hz (50Hz = 20msec)
servo1.duty_u16(74*64)          # Neutral Pos
# hx711 Loadcell ADconverter
hx711Clock = Pin(14, Pin.OUT)
hx711Clock.off()
hx711Data = Pin(27, mode=Pin.IN, pull=None)
# sw & led
sw1 = Pin(13, mode = Pin.IN, pull = Pin.PULL_UP)
blueLed = Pin(2, Pin.OUT)
blueLed.on()
# SD card
cd = Pin(22, mode = Pin.IN, pull = Pin.PULL_UP)    #Card Detect pin
sd = SDCard(slot=2, width=1, sck=Pin(18), mosi=Pin(23), miso=Pin(19), cs=Pin(5))

uart1 = UART(1,baudrate = 115200, tx = 25, rx = 26)   # 25(white), 26(yellow) -> Stack M5 CORE2 [GROVE 1(white):rx, 2(yellow):tx, 4(black):GND]

print()
print("***** Denki Tamabo *************************")
uart1.write("*** Denki Tamabo ***")

# tx test ######################
#while True:
#    if uart1.any():
#        #chr = uart1.readline()
#        chr = uart1.read(10)
#        print(chr)
#        print(int(chr)+1)
#    pass

## WiFi ###
def wifiConnect():
    import network
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect(wifiid.WIFI_SS_ID, wifiid.WIFI_PASSWORD)
        while not wlan.isconnected():
            pass
    print('network config:', wlan.ifconfig())

wifiConnect()
# time  
ntptime.settime()
rtc = RTC()
JST_OFFSET = 9 * 60 * 60
(year, month, day, hour, min, sec, wd, yd) = utime.localtime(utime.time() + JST_OFFSET)
rtc.datetime((year, month, day, wd, hour, min, sec, 0))
#print('RTC:', rtc.datetime())
txtTimeFileName = "{:4d}{:02d}{:02d}".format(year, month, day) + "-{:02d}{:02d}{:02d}".format(hour, min, sec)
print(f"{year:4d}-{month:02d}-{day:02d} {hour:02d}:{min:02d}:{sec:02d} (JST)")
print()

# *** switch & LED ********************************************
# Power on LED
blueLed.on()
utime.sleep_ms(500)
blueLed.off()
utime.sleep_ms(300)
blueLed.on()
utime.sleep_ms(100)
blueLed.off()
utime.sleep_ms(100)
blueLed.on()
utime.sleep_ms(100)
blueLed.off()


# *** SD card *************************************************
def sdMount():
    uos.mount(sd, '/sd')
    #print(uos.listdir('/sd'))
    #print(fileName)
    sdf = open(fileName, 'w')
    sdf.write(f"*** DENKI TAMABO **************\n")
    sdf.write(f"\n")
    sdf.close()
    print('SD card mount OK')
    print('filename: ' + fileName)

fileName = '/sd/tamabo' + txtTimeFileName + '.csv'
if (cd.value() == 0):
    detectSd = 1
    sdMount()
else:
    print('!!!!!!!!!! NO SD card !!!!!!!!!!!!!!!!!!!!')
    detectSd = 0

print()        


# *** loadcell init *******************************************
# reset
hx711Clock.on()
utime.sleep_ms(1)
hx711Clock.off()
utime.sleep_ms(1)

V_OUT = 0.000728            # sensor Voltage@maxLoad [V/V]  spec:0.65~0.95mV
MAX_LOAD = 500.0            # sensor rating [g]
#ADConverter config
ADC_GAIN = 128              # A/D　gain
R1 = 12000                  # Register1 [Ω] for 3.3V
R2 = 8200                   # Register2 [Ω]
V_BG = 1.25                 # analog reference [V]
AVDD = V_BG * (R1+R2)/R2    # analog max voltage AVDD [V]
ADC_1BIT = AVDD / 2**24     # max voltage / 24bit [V/bit]
SCALE = V_OUT / MAX_LOAD * AVDD     # loadcell scale [V/gf]
K1 = ADC_1BIT / (SCALE * ADC_GAIN)
#
K = MAX_LOAD / (V_OUT * 2**24 * ADC_GAIN)   # gf/data1bit

print('# loadcell')
print(f'loadcell:         {MAX_LOAD:4.0f} gf (MAX)')
print(f'AVDD:            {AVDD:5.3f} V')
print(f'sensorV:       {(V_OUT*1000):7.3f} V/V @Max load')
print(f'k:          {K*1000:10.4f} mgf/bit (+-23bit)')

if(detectSd == 1):
    f = open(fileName, 'a')
    f.write('# loadcell\n')
    f.write(f'loadcell:,{MAX_LOAD:4.0f},gf (MAX)\n')
    f.write(f'AVDD:,{AVDD:5.3f},V\n')
    f.write(f'sensorV:,{(V_OUT*1000):7.3f},V/V @Max load\n')
    f.write(f'k:,{K*1000:10.4f},mgf/bit (+-23bit)\n')
    f.close()

# analog value
def digiVtoWeight(adcV, zeroV):
    wgf = (adcV - zeroV) * K
    return wgf

# digtal data
def readAdc():
    #ret: adValue - signed 23bit

    adValue = 0
    while True:
        # ADc wait
        if hx711Data.value() == 0:
            utime.sleep_us(120)      # remove 78us pulse noise   100->120 2023.4.29
            if hx711Data.value() == 0:
                break

    for _ in range(24):
        hx711Clock.on()
        #utime.sleep_us(1)  # no delay -> over 10us 
        hx711Clock.off()
        adValue = adValue << 1
        adValue += hx711Data.value()

    hx711Clock.on()
    hx711Clock.off()
    if (adValue & 0x00800000) != 0:
        # minus value (24bit)
        adValue = (adValue ^ 0xfffffe) * -1
    # print(f"{adValue:06x}", end = '  ')  # debug
    return adValue

def averageData(n, dotDisp):
    # ret: ave - signed 23bit
    dataSum = 0
    for _ in range(n):
        dataSum += readAdc()
        if dotDisp == 1:
            print('.', end = "")
    ave = int(dataSum / n)
    # print("average = 0x{:06x}".format(ave), end = '  ')  # debug
    return ave

def tareZero():
    # zero set
    print('zero set ', end = '')
    dataZero = averageData(10, 1)
    print(' OK!')
    return dataZero

#test
def testLoadcell():
    # TEST loadcell continuous read
    zeroV = tareZero()
    while True:
        wgf = digiVtoWeight(readAdc(), zeroV)
        print(f"{wgf:7.2f}gf")


# *** servo init **********************************************
# global
ARM_R = 8.5     # servo horn length[mm]
START_DEG = -30	# start angle[°]
END_DEG = 32	# end angle[°]
INC_DEG = 2     # increment angle[°]
DEG_OFFSET = 6  # offset at neutral position[°]
STEP_BY_DEG = ARM_R * math.sin(math.radians(1))     # mm/deg
START_POS = ARM_R * math.sin(math.radians(START_DEG))   #mm
print('# servo')
print(f'Degree step:     {INC_DEG:5.2f} deg')
print(f'1step:           {STEP_BY_DEG*INC_DEG:5.3f} mm')

if(detectSd == 1):
    f = open(fileName, 'a')
    f.write('# servo\n')
    f.write(f'Degree step:,{INC_DEG:5.2f},deg\n')
    f.write(f'1step:,{STEP_BY_DEG*INC_DEG:5.3f},mm\n')
    f.close()

def servoMoveDeg(deg1):
    servo1.duty(servoDegtoHex(deg1))

def servoDegtoHex(setDeg):
    if (setDeg < (START_DEG-1)) or (setDeg > (END_DEG+1)):
        # out of range
        print("servo angle error!!!")
        return -9.9
    v0_u16 = 74*64        # 1.45msec = 7.25%
    vn90_u16 = 120*64      # 2.4msec = 12.0%
    vp90_u16 = 30*64       # 0.5msec = 2.5%

    degP = setDeg + DEG_OFFSET                   # servo angle
    value_u16 = -degP * (vn90_u16 - vp90_u16) / 180 + v0_u16
    #print(value_u16, end=' ')  # debug
    return int(value_u16)

def servoPosMn(deg1):
    # servo degree -> Position X
    Pos = ARM_R * math.sin(math.radians(deg1))  #mm
    return Pos - START_POS

def posXtoDeg(posX):
    # Position X -> servo degree
    x = posX + START_POS 
    deg =  math.degrees(math.asin(x / ARM_R))  #degree
    return deg

def pwmDuty(percent):
    value = 65536 * percent / 100  # duty 0~65535  : 20msec/65536=0.305usec : servo dead utime 1usec=>3.3
    return int(value)

def retStartDeg():
    sa = servoDegtoHex(END_DEG)
    se = servoDegtoHex(START_DEG)
    if sa > se:
        inc = -1
    else:
        inc = 1
    #print(sa,se,inc)
    for hex in range(sa, se, inc):
        servo1.duty_u16(hex)
        utime.sleep_us(500)

# --- test -----
def servoPos():
    # TEST Servo Position
    print('SERVO TEST   position check  push button to next position')

    while True:
        print('zero position')
        servo1.duty_u16(servoDegtoHex(0))           # servo neutral position
        utime.sleep(1)
        while True:
            if tSecWait(1) == 1:
                break

        print('start position')
        servo1.duty_u16(servoDegtoHex(START_DEG))    # servo start position
        utime.sleep(1)
        while True:
            if tSecWait(1) == 1:
                break

        print('end position')
        servo1.duty_u16(servoDegtoHex(END_DEG))      # servo end position
        utime.sleep(1)
        while True:
            if tSecWait(1) == 1:
                break

def servoMove():
    # TEST servo move
    print('SERVO TEST   push button to 1step rotate')
    posDeg = 0.0  # start angle
    degInc = 0.5  # increment step
    d = 0
    while True:
        d = servoDegtoHex(posDeg)     # return val = -9.9 in err, stop program by "typeError"
        servo1.duty_u16(d)
        print(f"{posDeg:5.1f}deg   duty={(d/655.36):5.2f}%  ({d:4d}/65536)")
        utime.sleep_ms(150)

        while True:
            if sw1.value() == 0:
                break
        posDeg += degInc
        if (posDeg >= END_DEG) or (posDeg <= START_DEG):
            degInc = -degInc


# *** Input , Data Save subroutine **************************************************
def tSecWait(t):
    # wait to push button (~ t sec)
    for _ in range(t * 10):
        utime.sleep_ms(100)
        if uart1.any():
            if (uart1.read(3)).decode() is "BTZ":
                return 1
        if sw1.value() == 0:
            return 1
    return 0


def dataSave(da):
    # data save -> SD card
    if(detectSd == 0):
        print('!!! No SD card !!! can not data save.')
        return
    
    sdf = open(fileName, 'a')
    sdf.write(f"No.,time[sec] ,arm[deg] ,x_pos[mm] ,force[gf]\n")
    for d in da:
        sdf.write(f"{d[0]:3d},   {d[1]:7.3f},   {d[2]:6.2f},   {d[3]:7.3f},  {d[4]:8.2f}\n")
    sdf.write(f"\n")
    sdf.close()


# *** Measure *************************************************************************
def measureOnce():
    global data
    global ave

    utime.sleep_ms(200)
    zeroOffset = tareZero()  # set zero every time
    utime.sleep_ms(100)
    
    retStartDeg()
    utime.sleep_ms(700)

    deg = START_DEG
    t0 = utime.ticks_us()
    i = 0
    xx = []
    ff = []

###
#    print('push button to start')   #ノズルを落ち着かせたい時の特別シーケンス
#    while True:
#        if sw1.value() == 0:
#            break
###   

    print('    time[sec]     angle[deg]   x_pos[mm]            force[gf]')
    while deg < END_DEG:
        t1 = utime.ticks_us()
        cyctime = (t1 -t0)/1000000
        print(f" {cyctime:7.2f} sec    ", end = "") # time

        posMm = servoPosMn(deg)
        servo1.duty_u16(servoDegtoHex(deg))
        print(f"{deg:7.2f} deg   ", end = "")  # degree ###############
        print(f"{posMm:6.2f} mm    ", end = "")    # position x

        utime.sleep_ms(20)        # servo time lag (receive PWM signal & moving)
        utime.sleep_ms(WAIT_FOR)

        adV = averageData(NUM_AVE, 1)            # measurement cycle 80Hz　(Actual measurement　10.6msec)
        weight = digiVtoWeight(adV, zeroOffset)
        print(f"   {weight:6.1f} gf  ", end = "")      # weight
        #simple bar graph
        for _ in range(int(weight/10)):
            print("*", end = "")
        print()    
        uart1.write(f"PGZ {posMm:6.3f} {weight:6.1f}")    # graph data -> M5

        data.append([numMeas, cyctime, deg, posMm, weight])
        xx.append(posMm)
        ff.append(weight)

        ave[i] += weight
        deg += INC_DEG
        i += 1
    return xx, ff

def warmUpMove():
    #pull fast, slow return 
    print('warm up ', end="")
    for i in range(6):
        print(f"{i+1:1d}.", end="")
        deg = START_DEG
        utime.sleep_ms(300)

        while deg < END_DEG:
            servo1.duty_u16(servoDegtoHex(deg))
            deg += INC_DEG
            utime.sleep_us(100)        # servo time lag (receive PWM signal & moving)

        utime.sleep_ms(700)
        print(".", end="")
        retStartDeg()
        print(".", end="")
        utime.sleep_ms(100)

    # 0 -> end pos
    for deg in range(START_DEG, END_DEG):
        servo1.duty_u16(servoDegtoHex(deg))
        utime.sleep_ms(20)    
    print()    


def dataAveSave(xx, ff, sum):
    # data average save -> SD card
    max = 0
    global NUM_MEAS
    print()
    print(f"x_pos[mm],", end="")
    for n in range(NUM_MEAS):
        print(f" Fn{n+1:1d}[gf],", end="")
    print(f" average[gf]  ")

    if(detectSd == 1):
        #SDcard open
        sdf = open(fileName, 'a')
        sdf.write(f"x_pos[mm],")
        for n in range(NUM_MEAS):
            sdf.write(f"Fn{n+1:1d}[gf],")
        sdf.write(f"average[gf]\n")

    for i, x in enumerate(xx):
        ave = sum[i] / NUM_MEAS
        if ave > max:
            max = ave
        print(f"  {x:7.3f},", end="")
        for n in range(NUM_MEAS):
            print(f"{ff[n][i]:8.2f},", end="")
        print(f"{ave:8.2f}   ", end="")

        #simple bar graph
        for _ in range(int(ave/10)):
            print("*", end = "")
        print()
       
        if(detectSd == 1):
            sdf.write(f"{x:7.3f},")
            for n in range(NUM_MEAS):
                sdf.write(f"{ff[n][i]:8.2f},")
            sdf.write(f"{ave:8.2f}\n")
 
    if(detectSd == 1):    
        sdf.write(f"\n")
        sdf.close()
    return max


# *** test test test ******************************************
#ZeroOffset = tare()    # Loadcell Zero set
#testLoadcell()         # loadcell Continuous test
#servoPos()             # servo position
#servoMove()            # servo move test
#
#data = [[1, 1.0, 1.0, 1.0],[2, 1.2, 2.0, 2.5]]
#dataSave(data)         #data save test


# ***  MAIN ***************************************************
numMeas = 1     # measurement number
WAIT_FOR = 240  # msec @wait for stop BB
NUM_AVE = 5     # average　sample number
SERVO_DELAY = 20    # servo PWM 50Hz
TIME_ADC = 10.6 * (NUM_AVE - 1)     # ADconvert time
NUM_MEAS = 4        # repeat measure
warmupFlag = True

print('# timing')
print(f'wait for stop:     {WAIT_FOR:3d} msec @1step')
print(f'servo delay:       {SERVO_DELAY:3d} msec')
print(f'average num:        {NUM_AVE:2d} times')
print(f'ADC time:        {TIME_ADC:5.1f} msec')
print(f'1step time:      {((WAIT_FOR+SERVO_DELAY+TIME_ADC)/1000):5.3f} sec')

if(detectSd == 1):
    f = open(fileName, 'a')
    f.write('# timing\n')
    f.write(f'wait for stop:,{WAIT_FOR:3d},msec @1step\n')
    f.write(f'servo delay:,{SERVO_DELAY:3d},msec\n')
    f.write(f'average num:,{NUM_AVE:2d},times\n')
    f.write(f'ADC time:,{TIME_ADC:5.1f},msec\n')
    f.write(f'1step time:,{((WAIT_FOR+SERVO_DELAY+TIME_ADC)/1000):5.3f},sec\n')
    f.write(f"\n")
    f.close()

# 0 -> end pos
for deg in range(0, END_DEG):
    servo1.duty_u16(servoDegtoHex(deg))
    utime.sleep_ms(20)
utime.sleep_ms(500)



while True:

    if warmupFlag is True:
        uart1.write("WUZ")      # warmup  -> M5
    else:
        uart1.write("WNZ")      # no warmup -> M5


    # -> start pos
    for deg in range(END_DEG, START_DEG, -1):
        servo1.duty_u16(servoDegtoHex(deg))
        utime.sleep_ms(20)
    posDeg = START_DEG

    # ready LED on
    blueLed.on()
    zeroOffset = tareZero()  # set zero

    print()
    print('push button to start measurement')
    uart1.write("push button to start")
    utime.sleep_ms(300)


    # weight display
    uart1.write("ULZ")      # unlock -> M5
    while True:
        if sw1.value() == 0:
            break
        adV = averageData(10, 0)            # measurement cycle 80Hz　(Actual measurement　10.6msec)
        weight = digiVtoWeight(adV, zeroOffset)
        uart1.write(f"AAZ {weight:6.1f}")      # weight -> M5
        #utime.sleep_ms(100)        # servo time lag (receive PWM signal & moving)
        if uart1.any():
            mesType = (uart1.read(3)).decode()
            #print("type:"+mesType, end="")
            if mesType is "BTZ":
                break
            elif mesType is "WUZ":
                warmupFlag = True
                print("Worm up ON")
            elif mesType is "WNZ":
                warmupFlag = False
                print("Worm up OFF")
            elif mesType is "XPZ":
                mes = (uart1.read(5)).decode()
                #print(mes, end="")
                posXmm = float(mes) / 10
                #print(f" posX:{posXmm:6.2f}mm ", end="")
                d = posXtoDeg(posXmm)
                h = servoDegtoHex(d)     # return val = -9.9 in err, stop program by "typeError"
                #print(f"ded:{d:6.2f}deg  hex:{h:04x}")
                servo1.duty_u16(h)
        utime.sleep_ms(1)     # servo time lag (receive PWM signal & moving)


    utime.sleep_ms(100)     # servo time lag (receive PWM signal & moving)
    uart1.write("CLR")      # graph clear -> M5
    utime.sleep_ms(100)
    uart1.write("LOZ")      # lock -> M5
    utime.sleep_ms(100)
    
    if warmupFlag is True:
        uart1.write("Warm up ")
        warmUpMove()
    else:
        retStartDeg()
        print('No warm up')

    print(f'#{numMeas:3d} ')
    # start measurement
    blueLed.off()
    uart1.write("In progress")
    ave = 65*[0]    #配列数65
    x = []
    f = []
    for i in range(NUM_MEAS):
        #measure  mesN times
        data = []
        utime.sleep_ms(200)
        x, fi = measureOnce()
        f.append(fi)
        numMeas += 1
        #dataSave(data)  
    fMax = dataAveSave(x, f, ave)
    uart1.write(f"MAZ {fMax:4.0f}gf")      # MAX  -> M5
    print(f' Nukidan Max = {fMax:8.2f}gf ')
    print()

    # next
    print('push button to next (adjust next setting)')
    utime.sleep_ms(100)
    uart1.write("push button to next")

    while True:
        blueLed.on()
        if tSecWait(1) == 1:
            break
        blueLed.off()
        if tSecWait(1) == 1:
            break

        if ((cd.value() == 0) and (detectSd == 0)):
            detectSd = 1
            sdMount()

    if numMeas >= 99:
        print('too many data ---> restart')
        if(detectSd == 1):
            uos.umount('/sd')
        break

    # UART clear
    if uart1.any():
        dummy = uart1.read()



