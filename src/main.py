# 電気玉棒
# 抜弾抵抗測定
#
# loadcell   HX711  500gf
# servo motor SG92R
#
# ESP32
#
# 2022.11.27
# 2023.01.05          SDcard 追加
# 2023.03.25          SDcardなし　基板制作（一部ピン変更）
# 2023.03.26 ver.2.30 やっぱりSDcard あり
# 2023.03.27 ver.2.31 WiFiより時刻を取得->SDcardへの保存ファイルのタイムスタンプが入る
# 2023.03.28 ver.2.32 ファイル名に日時を入れてユニークに
#
#


import math
import uos
import ntptime
import utime
from machine import Pin, PWM, SDCard, RTC
import wifiid

print()
print("***** Denki Tamabo *************************")

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
sw1 = Pin(13, mode = Pin.IN, pull = Pin.PULL_UP)
blueLed = Pin(2, Pin.OUT)
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
cd = Pin(22, mode = Pin.IN, pull = Pin.PULL_UP)    #SDカードが入っていないとエラーで止まるのでcdピン追加
sd = SDCard(slot=2, width=1, sck=Pin(18), mosi=Pin(23), miso=Pin(19), cs=Pin(5))
# Hard ResetしてESP32内のプログラムを実行ではOKだけど、VScodeからのrunだとエラーになって止まる???　OSError: (-259, 'ESP_ERR_INVALID_STATE')
if (cd.value() == 0):
    detectSd = 1
    uos.mount(sd, '/sd')
    #print(uos.listdir('/sd'))
    fileName = '/sd/tamabo' + txtTimeFileName + '.txt'
    #print(fileName)
    f = open(fileName, 'w')
    f.write(f"*** DENKI TAMABO **************\n")
    f.write(f"\n")
    f.close()
    print('SD card mount OK')
    print('filename: ' + fileName)
else:
    print('!!!!!!!!!! NO SD card !!!!!!!!!!!!!!!!!!!!')
    detectSd = 0

print()        


# *** loadcell init *******************************************
# hx711 Loadcell ADconverter
hx711Clock = Pin(14, Pin.OUT)
hx711Data = Pin(27, mode=Pin.IN, pull=None)     # ESP32
# reset
hx711Clock.on()
utime.sleep_ms(1)
hx711Clock.off()
utime.sleep_ms(1)

V_out = 0.000728            # センサ出力電圧@maxLoad [V/V]  spec:0.65~0.95mV 校正対象
maxLoad = 500.0             # センサ定格 [g]
#ADConverter config
ADCGain = 128               # A/Dゲイン
R1 = 12000                 # R1抵抗値 [Ω] 3.3V版に改造
R2 = 8200                  # R2抵抗値 [Ω]
Vbg = 1.25                 # アナログリファレンス電圧 [V]
aVdd = Vbg * (R1+R2)/R2    # アナログ電圧フルスケールAVDD [V]
ADC1bit = aVdd / 2**24     # フルスケールを24bitで割る [V]
Scale = V_out / maxLoad * aVdd  # ロードセル特性　[V/gf]
k1 = ADC1bit / (Scale * ADCGain) #計算の係数
#
k = maxLoad / (V_out * 2**24 * ADCGain) #式を整理 gf/data1bit

print('# loadcell')
print(f'loadcell:         {maxLoad:4.0f} gf (MAX)')
print(f'AVDD:            {aVdd:5.3f} V')
print(f'sensorV:       {(V_out*1000):7.3f} V/V @Max load')
print(f'k:          {k*1000:10.4f} mgf/bit (+-23bit)')

if(detectSd == 1):
    f = open(fileName, 'a')
    f.write('# loadcell\n')
    f.write(f'loadcell:         {maxLoad:4.0f} gf (MAX)\n')
    f.write(f'AVDD:            {aVdd:5.3f} V\n')
    f.write(f'sensorV:       {(V_out*1000):7.3f} V/V @Max load\n')
    f.write(f'k:          {k*1000:10.4f} mgf/bit (+-23bit)\n')
    f.close()

# analog value
def digiVtoWeight(adcV, zeroV):
    wgf = (adcV - zeroV) * k
    return wgf

# digtal data
def readAdc():
    #ret: adValue - signed 23bit

    adValue = 0
    while True:
        # ADc wait
        if hx711Data.value() == 0:
            utime.sleep_us(100)      #78usほどの変なパルスがあるらしいので除去
            if hx711Data.value() == 0:
                break

    for _ in range(24):
        hx711Clock.on()
        #utime.sleep_us(1) #遅延いれなくても10usほど
        hx711Clock.off()
        adValue = adValue << 1
        adValue += hx711Data.value()

    hx711Clock.on()
    hx711Clock.off()
    if (adValue & 0x00800000) != 0:
        # マイナスの時の処理(24ビット数)
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
    # 風袋
    print('zero set ', end = '')
    dataZero = averageData(10, 1)
    print(' OK!')
    return dataZero

#test
def testLoadcell():
    # ロードセルの連続表示　テスト用
    zeroV = tareZero()
    while True:
        wgf = digiVtoWeight(readAdc(), zeroV)
        print(f"{wgf:7.2f}gf")


# *** servo init **********************************************
servo1 = PWM(Pin(4), freq=50)  # PWM freq 1~1000Hz
# servo 50Hz = 20msec
# global
armR = 8.5      # サーボホーンの腕の長さ[mm]
startDeg = -24	# 初期角度[°]
endDeg = 26	    # 終角度[°]
incDeg = 1.0    # 角度増分[°]
degOffset = 6   # サーボ中立電圧位置でのズレ[°]
stepbyDeg = armR * math.sin(math.radians(1))    # mm/deg
startPos = armR * math.sin(math.radians(startDeg))  #mm
print('# servo')
print(f'Degree step:     {incDeg:5.2f} deg')
print(f'1step:           {stepbyDeg*incDeg:5.3f} mm')

if(detectSd == 1):
    f = open(fileName, 'a')
    f.write('# servo\n')
    f.write(f'Degree step:     {incDeg:5.2f} deg\n')
    f.write(f'1step:           {stepbyDeg*incDeg:5.3f} mm\n')
    f.close()

def servoMoveDeg(deg1):
    servo1.duty(servoDegtoHex(deg1))

def servoDegtoHex(setDeg):
    if (setDeg < (startDeg-1)) or (setDeg > (endDeg+1)):
        # 可動範囲外の時リターン
        print("errorroror")
        return -9.9
    v0_u16 = 74*64        # 1.45msec = 7.25%
    vn90_u16 = 120*64      # 2.4msec = 12.0%
    vp90_u16 = 30*64       # 0.5msec = 2.5%

    degP = setDeg + degOffset                   # サーボへの指令角度
    value_u16 = -degP * (vn90_u16 - vp90_u16) / 180 + v0_u16
    #print(value_u16, end=' ')  # debug
    return int(value_u16)

def servoPosMn(deg1):
    Pos = armR * math.sin(math.radians(deg1))  #mm
    return Pos - startPos


def pwmDuty(percent):
    value = 65536 * percent / 100  # duty 0~65535  : 20msec/65536=0.305usec : servo dead utime 1usec=>3.3
    return int(value)

def servoPos():
    # サーボの位置確認
    print('SERVO TEST   position check  push button to next position')

    while True:
        print('zero position')
        servo1.duty_u16(servoDegtoHex(0))           # サーボ　ゼロ位置
        utime.sleep(1)
        while True:
            if tSecWait(1) == 1:
                break

        print('start position')
        servo1.duty_u16(servoDegtoHex(startDeg))    # サーボ　スタート位置
        utime.sleep(1)
        while True:
            if tSecWait(1) == 1:
                break

        print('end position')
        servo1.duty_u16(servoDegtoHex(endDeg))      # サーボ　エンド位置
        utime.sleep(1)
        while True:
            if tSecWait(1) == 1:
                break

def servoMove():
    # サーボを動かして見てみる
    print('SERVO TEST   push button to 1step rotate')
    posDeg = 0.0  # スタート角度
    degInc = 0.5  # 変化分
    d = 0
    while True:
        d = servoDegtoHex(posDeg)     #エラーの時-9.9で帰るのでtypeErrorで止まる
        servo1.duty_u16(d)
        print(f"{posDeg:5.1f}deg   duty={(d/655.36):5.2f}%  ({d:4d}/65536)")
        utime.sleep_ms(150)

        while True:
            if sw1.value() == 0:
                break
        posDeg += degInc
        if (posDeg >= endDeg) or (posDeg <= startDeg):
            degInc = -degInc


# *** 入力 セーブ等 サブ **************************************************
def tSecWait(t):
    #t秒のボタン入力待ち
    for _ in range(t * 10):
        utime.sleep_ms(100)
        if sw1.value() == 0:
            return 1
    return 0


def dataSave(da):
    # データを保存
    if(detectSd == 0):
        print('!!! No SD card !!! can not data save.')
        return
    
    f = open(fileName, 'a')
    #f = open('denkitamabou.txt', 'a') #ESPフラッシュへ書込
    f.write(f"No., time[sec], arm[deg], x_pos[mm], force[gf]\n")
    for d in da:
        f.write(f"{d[0]:3d},   {d[1]:7.3f},   {d[2]:6.2f},   {d[3]:7.3f},  {d[4]:8.2f}\n")
    f.write(f"\n")
    f.close()


# *** test test test ******************************************
#ZeroOffset = tare()    # 秤のゼロ合わせ
#testLoadcell()         # loadcell 連続表示テスト
#servoPos()             # サーボの位置
#servoMove()            # サーボを動かしてみる
#data = [[1, 1.0, 1.0, 1.0],[2, 1.2, 2.0, 2.5]]
#ataSave(data)              #データ保存


# ***  MAIN ***************************************************
numMeas = 1     # measurement number
waitFor = 240   # msec @wait for stop BB
numAve = 5      # average　sample number
servoDelay = 20 # servo PWM 50Hz
timeAdc = 10.6 * (numAve - 1)   # ADconvert time

print('# timing')
print(f'wait for stop:     {waitFor:3d} msec @1step')
print(f'servo delay:       {servoDelay:3d} msec')
print(f'average num:        {numAve:2d} times')
print(f'ADC time:        {timeAdc:5.1f} msec')
print(f'1step time:      {((waitFor+servoDelay+timeAdc)/1000):5.3f} sec')

if(detectSd == 1):
    f = open(fileName, 'a')
    f.write('# timing\n')
    f.write(f'wait for stop:     {waitFor:3d} msec @1step\n')
    f.write(f'servo delay:       {servoDelay:3d} msec\n')
    f.write(f'average num:        {numAve:2d} times\n')
    f.write(f'ADC time:        {timeAdc:5.1f} msec\n')
    f.write(f'1step time:      {((waitFor+servoDelay+timeAdc)/1000):5.3f} sec\n')
    f.write(f"\n")
    f.close()

utime.sleep_ms(500)


while True:
    data = []
    print()
    # ready LED on
    blueLed.on()
    #サーボ初期位置
    deg = servoDegtoHex(startDeg)
    servo1.duty_u16(deg)
    utime.sleep_ms(1000)
    # ここで重量表示できると良いかも

    print('push button to start measurement')
    while True:
        if sw1.value() == 0:
            break

    # 測定開始
    blueLed.off()
    zeroOffset = tareZero()  # 毎回ゼロ合わせ
    utime.sleep_ms(2)

    deg = startDeg
    t0 = utime.ticks_us()

    print(f'#{numMeas:3d} ')
    print('time[sec]  x_pos[mm]         force[gf]')
    while deg < endDeg:
        t1 = utime.ticks_us()
        cyctime = (t1 -t0)/1000000
        print(f" {cyctime:7.2f}     ", end = "") # time

        posMm = servoPosMn(deg)
        du_16 = servoDegtoHex(deg)
        servo1.duty_u16(du_16)
        #print(f"{deg:7.2f} deg   ", end = "")  # degree
        print(f"{posMm:6.2f}    ", end = "")    # position x

        utime.sleep_ms(20)        # サーボのタイムラグ PWM信号を受けるまでの時間と回す時間
        utime.sleep_ms(waitFor)

        adV = averageData(numAve, 1)            # 測定周期 80Hz(実測10.6msecくらい)
        weight = digiVtoWeight(adV, zeroOffset)

        print(f"   {weight:6.1f}")      # weight

        data.append([numMeas, cyctime, deg, posMm, weight])
        deg += incDeg

    numMeas += 1
    dataSave(data)

    # next
    print('push button to next')
    while True:
        blueLed.on()
        if tSecWait(1) == 1:
            break
        blueLed.off()
        if tSecWait(1) == 1:
            break

        if ((cd.value() == 0) and (detectSd == 0)):
            detectSd = 1
            uos.mount(sd, '/sd')
            fileName = '/sd/tamabo.txt'
            f = open(fileName, 'w')
            f.write(f"*** DENKI TAMABOU **************\n")
            f.write(f"\n")
            f.close()
            print('SD card mount OK')
            print('push button to next')


    if numMeas >= 99:
        print('too many data ---> restart')
        if(detectSd == 1):
            uos.umount('/sd')
        break



