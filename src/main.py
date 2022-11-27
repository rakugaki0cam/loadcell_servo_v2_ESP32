# loadcell   HX711  500gf
# servo motor SG92R
#
# ESP32
#
# 2022.11.27
#

import math
import utime
from machine import Pin, PWM

print("***** Load cell & servo test ***************************")

# *** switch & LED ********************************************
sw1 = Pin(5, mode = Pin.IN, pull = Pin.PULL_UP)
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


# *** loadcell init *******************************************
# hx711 Loadcell ADconverter
hx711Clock = Pin(14, Pin.OUT)
hx711Data = Pin(27, mode=Pin.IN, pull=None)     # ESP32
# reset
hx711Clock.on()
utime.sleep_ms(1)
hx711Clock.off()
utime.sleep_ms(1)

V_out = 0.000728        # センサ出力電圧 [V/V] @maxLoad 0.65~0.95mV
maxLoad = 500.0         # センサ定格 [g]
R1 = 12000              # R1抵抗値 [Ω] 3.3V版に改造
R2 = 8200               # R2抵抗値 [Ω]
Vbg = 1.25              # アナログリファレンス電圧 [V]
aVdd = Vbg * (R1+R2)/R2 # アナログ電圧フルスケールAVdd [V]
ADC1bit = aVdd / 2**24  # フルスケールを24bitで割る [V]
ADCGain = 128           # A/Dゲイン
Scale = V_out * aVdd / maxLoad  # ロードセル特性　[V/gf]
k = ADC1bit / (Scale * ADCGain) #計算の係数

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
            break

    for _ in range(24):
        hx711Clock.value(1)
        #utime.sleep_us(1) #遅延いれなくても10usほど
        hx711Clock.value(0)
        adValue = adValue << 1
        adValue += hx711Data.value()

    hx711Clock.value(1)
    hx711Clock.value(0)
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
servo1 = PWM(Pin(13), freq=50)  # PWM freq 1~1000Hz
# servo 50Hz = 20msec
# global
armR = 8.5      # サーボホーンの腕の長さ[mm]
startDeg = -24	# 初期角度[°]
endDeg = 28	    # 終角度[°]
incDeg = 2      # 角度増分[°]
degOffset = 6   # サーボ中立電圧位置でのズレ[°]
stepbyDeg = armR * math.sin(math.radians(1))    # mm/deg

def servoMoveDeg(deg1):
    
    servo1.duty(servoDegtoHex(deg1))

def servoDegtoHex(setDeg):
    if (setDeg < (startDeg-0.09)) or (setDeg > (endDeg+0.09)):
        # 可動範囲外の時リターン
        print("errorroror")
        return -9.9, -9.9
    v0 = 74         # 1.45msec = 7.25%
    vn90 = 120      # 2.4msec = 12.0%
    vp90 = 30       # 0.5msec = 2.5%

    v0_u16 = 74*64        # 1.45msec = 7.25%
    vn90_u16 = 120*64      # 2.4msec = 12.0%
    vp90_u16 = 30*64       # 0.5msec = 2.5%

    degP = setDeg + degOffset                   # サーボへの指令角度
    value = -degP * (vn90 - vp90) / 180 + v0    # 2度ごとにしか設定できない

    value_u16 = -degP * (vn90_u16 - vp90_u16) / 180 + v0_u16
    #print(value_u16, end=' ')  # debug
    return int(value), int(value_u16)


def pwmDuty(percent):
    value = 1024 * percent / 100  # duty 0~1024
    return int(value)

def servoPos():
    # サーボの位置確認
    print('SERVO TEST   position check  push button to next position')

    while True:
        print('zero position')
        servo1.duty(servoDegtoHex(0))           # サーボ　ゼロ位置
        utime.sleep(1)
        while True:
            if tSecWait(1) == 1:
                break

        print('start position')
        servo1.duty(servoDegtoHex(startDeg))    # サーボ　スタート位置
        utime.sleep(1)
        while True:
            if tSecWait(1) == 1:
                break

        print('end position')
        servo1.duty(servoDegtoHex(endDeg))      # サーボ　エンド位置
        utime.sleep(1)
        while True:
            if tSecWait(1) == 1:
                break

def servoMove():
    # サーボを動かして見てみる
    print('SERVO TEST   push button to 1step rotate')
    posDeg = 0.0  # スタート角度
    degInc = 0.5  # 変化分
    du_u16 = 0
    while True:
        #print(posDeg, end = " ")
        _, du_u16 = servoDegtoHex(posDeg)     #エラーの時-9.9で帰るのでtypeErrorで止まる
        #servo1.duty(du)      # サーボ　ゼロ位置
        #print(du_u16, end = " ")
        servo1.duty_u16(du_u16)
        #print(f"{posDeg:3d}deg   duty={(du/10.24):5.1f}%  ({du:4d}/1024)")     #ESP8266
        print(f"{posDeg:5.1f}deg   duty={(du_u16/655.36):5.2f}%  ({du_u16:4d}/65536)")
        utime.sleep_ms(2)

        while True:
            if sw1.value() == 0:
                break
        posDeg += degInc
        if posDeg >= endDeg:
            degInc = -0.5
        if posDeg <= startDeg:
            degInc = 0.5


# *** 入力等 サブ **************************************************
def tSecWait(t):
    #t秒のボタン入力待ち
    for _ in range(t * 10):
        utime.sleep_ms(100)
        if sw1.value() == 0:
            return 1
    return 0


# *** test ****************************************************
# ZeroOffset = tare()    # 秤のゼロ合わせ
# testLoadcell()         # loadcell 連続表示テスト
# servoPos()             # サーボの位置
# servoMove()            # サーボを動かしてみる


# ***  MAIN ***************************************************
nd = 0

utime.sleep_ms(500)
# ready LED on
blueLed.on()

while True:
    data = []

    #サーボ初期位置
    d, _ = servoDegtoHex(startDeg)
    servo1.duty(d)
    utime.sleep_ms(1000)
    # ここで重量表示できると良いかも

    print('push button to start measurement')
    while True:
        if sw1.value() == 0:
            break

    # 測定開始
    blueLed.value(0)
    zeroOffset = tareZero()  # 毎回ゼロ合わせ
    utime.sleep_ms(300)

    for deg in range(startDeg, endDeg+incDeg, incDeg):
        posMm = stepbyDeg * (deg - startDeg)
        # print(f"{deg:3d} deg   ", end = "")  # debug
        print(f"{posMm:5.1f} mm  ", end = "")
        d, _ = servoDegtoHex(deg)
        servo1.duty(d)
        utime.sleep_ms(5)           # サーボのタイムラグ 0.1sec/60° = 3.3msec＠4.8V
        utime.sleep_ms(100)         # 動かした後 落ち着かせる場合
        adV = averageData(3, 1)     # 測定周期 10Hz
        weight = digiVtoWeight(adV, zeroOffset)
        print(f" {weight:6.1f} gf")
        data.append([nd, deg, posMm, weight])

    nd += 1
    csv = 0
    if csv:
        # csv data print
        print('push button to print csv data')
        while True:
            if tSecWait(1) == 1:
                break
        print(f'data No.{nd:3d}')
        for _, _, _, w in data:
            print(f'{w:6.1f}')

    # next
    print('push button to next')
    while True:
        blueLed.on()
        if tSecWait(1) == 1:
            break
        blueLed.off()
        if tSecWait(1) == 1:
            break
    print()
    blueLed.on()

    if nd >= 99:
        break


# 保存データを見る
for nd, deg, posMm, weight in data:
    print(f"{nd:3d}   {deg:3d} deg   {posMm:5.1f} mm   {weight:8.2f} gf")



