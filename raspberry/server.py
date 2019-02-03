#!/usr/bin/env python
# -*- coding: utf-8 -*-

import base64
import hashlib
import math
import socket
import threading
import time

import RPi.GPIO as GPIO
from Adafruit_PWM_Servo_Driver import PWM


class Controller:
    def __init__(self):
        # 四驱
        self.PWMA = 18
        self.AIN1 = 27
        self.AIN2 = 22
        self.PWMB = 23
        self.BIN1 = 25
        self.BIN2 = 24
        # 超声波模块
        self.Gpin = 5
        self.Rpin = 6
        self.TRIG = 20
        self.ECHO = 21
        # 红外线模块
        self.SensorRight = 16
        self.SensorLeft = 12
        self.T_SensorLeft = 13
        self.T_SensorRight = 26
        # Initialise the PWM device using the default address
        # bmp = PWM(0x40, debug=True)
        self.pwm = PWM(0x40, debug=False)
        # Min pulse length out of 4096
        self.servoMin = 150
        # Max pulse length out of 4096
        self.servoMax = 600
        # PWM系数
        self.DC2VC = 0.65
        self.L_Motor = None
        self.R_Motor = None
        # 初始化
        self.setup()

    def setup(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)
        GPIO.setup(self.Gpin, GPIO.OUT)  # Set Green Led Pin mode to output
        GPIO.setup(self.Rpin, GPIO.OUT)  # Set Red Led Pin mode to output
        GPIO.setup(self.AIN2, GPIO.OUT)
        GPIO.setup(self.AIN1, GPIO.OUT)
        GPIO.setup(self.PWMA, GPIO.OUT)
        GPIO.setup(self.BIN1, GPIO.OUT)
        GPIO.setup(self.BIN2, GPIO.OUT)
        GPIO.setup(self.PWMB, GPIO.OUT)
        GPIO.setup(self.SensorRight, GPIO.IN)
        GPIO.setup(self.SensorLeft, GPIO.IN)
        GPIO.setup(self.T_SensorLeft, GPIO.IN)
        GPIO.setup(self.T_SensorRight, GPIO.IN)
        self.pwm.setPWMFreq(50)
        self.L_Motor = GPIO.PWM(self.PWMA, 100)
        self.L_Motor.start(0)
        self.R_Motor = GPIO.PWM(self.PWMB, 100)
        self.R_Motor.start(0)

    def setServoPulse(self, channel, pulse):
        pulseLength = 1000000.0  # 1,000,000 us per second
        pulseLength /= 50.0  # 60 Hz
        pulseLength /= 4096.0  # 12 bits of resolution
        pulse *= 1000.0
        pulse /= (pulseLength * 1.0)
        self.pwm.setPWM(channel, 0, int(pulse))

    # Angle to PWM

    def write(self, servonum, x):
        y = x / 90.0 + 0.5
        y = max(y, 0.5)
        y = min(y, 2.5)
        self.setServoPulse(servonum, y)

    def ahead(self, vc, t):
        dc = vc / self.DC2VC
        self.L_Motor.ChangeDutyCycle(dc)
        GPIO.output(self.AIN2, False)  # AIN2
        GPIO.output(self.AIN1, True)  # AIN1
        self.R_Motor.ChangeDutyCycle(dc)
        GPIO.output(self.BIN2, False)  # BIN2
        GPIO.output(self.BIN1, True)  # BIN1
        time.sleep(t)

    def back(self, vc, t):
        dc = vc / self.DC2VC
        self.L_Motor.ChangeDutyCycle(dc)
        GPIO.output(self.AIN2, True)  # AIN2
        GPIO.output(self.AIN1, False)  # AIN1
        self.R_Motor.ChangeDutyCycle(dc)
        GPIO.output(self.BIN2, True)  # BIN2
        GPIO.output(self.BIN1, False)  # BIN1
        time.sleep(t)

    def left(self, vc, t):
        dc = vc / self.DC2VC
        self.L_Motor.ChangeDutyCycle(dc)
        GPIO.output(self.AIN2, True)  # AIN2
        GPIO.output(self.AIN1, False)  # AIN1
        self.R_Motor.ChangeDutyCycle(dc)
        GPIO.output(self.BIN2, False)  # BIN2
        GPIO.output(self.BIN1, True)  # BIN1
        time.sleep(t)

    def right(self, vc, t):
        dc = vc / self.DC2VC
        self.L_Motor.ChangeDutyCycle(dc)
        GPIO.output(self.AIN2, False)  # AIN2
        GPIO.output(self.AIN1, True)  # AIN1
        self.R_Motor.ChangeDutyCycle(dc)
        GPIO.output(self.BIN2, True)  # BIN2
        GPIO.output(self.BIN1, False)  # BIN1
        time.sleep(t)

    def stop(self, t):
        self.L_Motor.ChangeDutyCycle(0)
        GPIO.output(self.AIN2, False)  # AIN2
        GPIO.output(self.AIN1, False)  # AIN1
        self.R_Motor.ChangeDutyCycle(0)
        GPIO.output(self.BIN2, False)  # BIN2
        time.sleep(t)

    def turnL(self, angle):
        self.stop(0.2)
        r = 8.5
        s = r * angle
        v = 30 * self.DC2VC
        t = s / v + 0.8
        self.L_Motor.ChangeDutyCycle(30)
        GPIO.output(self.AIN2, True)  # AIN2
        GPIO.output(self.AIN1, False)  # AIN1
        self.R_Motor.ChangeDutyCycle(30)
        GPIO.output(self.BIN2, False)  # BIN2
        GPIO.output(self.BIN1, True)  # BIN1
        time.sleep(t)
        self.stop(0.2)

    def turnR(self, angle):
        self.stop(0.2)
        r = 8.5
        s = r * angle
        v = 30 * self.DC2VC
        t = s / v + 1
        self.L_Motor.ChangeDutyCycle(30)
        GPIO.output(self.AIN2, False)  # AIN2
        GPIO.output(self.AIN1, True)  # AIN1
        self.R_Motor.ChangeDutyCycle(30)
        GPIO.output(self.BIN2, True)  # BIN2
        GPIO.output(self.BIN1, False)  # BIN1
        time.sleep(t)
        self.stop(0.2)

    def distance(self):
        GPIO.output(self.TRIG, 0)
        time.sleep(0.000002)
        GPIO.output(self.TRIG, 1)
        time.sleep(0.00001)
        GPIO.output(self.TRIG, 0)
        while GPIO.input(self.ECHO) == 0:
            a = 0
        time1 = time.time()
        while GPIO.input(self.ECHO) == 1:
            a = 1
        time2 = time.time()
        during = time2 - time1
        return during * 340 / 2 * 100

    def front_detection(self):
        self.write(0, 90)
        time.sleep(0.05)
        dis_f = self.distance()
        print "front distance: %f," % dis_f
        return dis_f

    def left_detection(self):
        self.write(0, 175)
        time.sleep(0.5)
        dis_l = self.distance()
        self.write(0, 90)
        time.sleep(0.5)
        print "left distance: %f," % dis_l
        return dis_l

    def right_detection(self):
        self.write(0, 5)
        time.sleep(0.5)
        dis_r = self.distance()
        self.write(0, 90)
        time.sleep(0.5)
        print "right distance: %f," % dis_r
        return dis_r

    def detection(self, angle):
        self.write(0, angle)
        time.sleep(0.5)
        dis = self.distance()
        print "%f distance: %f," % (angle, dis)
        return dis

    # 左灯没有障碍物
    def lc(self):
        b = GPIO.input(self.SensorLeft)
        if b:
            print "left: save\n"
        else:
            print "left: unsave\n"
        return b

    # 右灯没有障碍物
    def rc(self):
        b = GPIO.input(self.SensorRight)
        if b:
            print "right: save\n"
        else:
            print "right: unsave\n"
        return b

    def lbc(self):
        b = GPIO.input(self.T_SensorLeft)
        if b:
            print "left back: save\n"
        else:
            print "left back: unsave\n"
        return b

    def rbc(self):
        b = GPIO.input(self.T_SensorRight)
        if b:
            print "right back: save\n"
        else:
            print "right back: unsave\n"
        return b

    def auto_drive(self, vc):

        dt = 6 / vc
        l = self.lc()
        r = self.rc()
        lb = self.lbc()
        rb = self.rbc()
        h_dis = self.front_detection()
        if h_dis > 40 and l and r:
            self.ahead(vc, 0)
            return "!"
        else:
            self.stop(0.5)
            l_dis = self.left_detection()
            r_dis = self.right_detection()
            string = "前方 %f 发现障碍物, 左边距离:%f, 右边距离%f" % (h_dis, l_dis, r_dis)
            if (l_dis < 20 and r_dis < 20) or (l == False and r == False) and (lb == True and rb == True):
                self.back(vc, dt)
                return string + ",向后退!"
            elif l_dis >= r_dis and l == True:
                self.left(vc, dt)
                return string + ",向左转!"
            elif r_dis > l_dis and r == True:
                self.right(vc, dt)
                return string + ",向右转!"
            elif l_dis > r_dis:
                self.turnL(math.pi)
                return string + ",向左大转弯!"
            else:
                self.turnR(math.pi)
                return string + ",向右大转弯!"


global client
client = None


# 客户端处理线程
class websocket_thread(threading.Thread):
    def __init__(self, connection, controller):
        super(websocket_thread, self).__init__()
        self.connection = connection
        self.handshake()
        self.controller = controller
        global client
        if client is None:
            client = connection
            self.running = True
        else:
            self.notify("警告:当前已有一个连接,最多只能有一个人控制小车")
            connection.close()
            self.running = False

    def run(self):
        global client
        while self.running:
            try:
                s = self.connection.recv(1024)
                (success, method, parm) = self.parse_data(s)
                if not success:
                    if parm < 0:
                        print "disconnected"
                        client = None
                        break
                    # 错误的数据
                    else:
                        print "unknown data"
                else:
                    if method == "1":
                        vc = float(parm)
                        print "ahead"
                        self.controller.ahead(vc, 0)
                    elif method == "2":
                        print "back"
                        vc = float(parm)
                        self.controller.back(vc, 0)
                    elif method == "3":
                        print "left"
                        vc = float(parm)
                        self.controller.left(vc, 0)
                    elif method == "4":
                        print "right"
                        vc = float(parm)
                        self.controller.right(vc, 0)
                    elif method == "5":
                        print "stop"
                        self.controller.stop(0)
                    elif method == "6":
                        print "detection"
                        angle = float(parm)
                        dis = self.controller.detection(angle)
                        self.notify('距离 %f 度角的障碍物 %f cm' % (angle, dis))
                    elif method == "7":
                        print "auto_drive"
                        vc = float(parm)
                        res = self.controller.auto_drive(vc)
                        self.notify(res)
                    else:
                        print "unknown method"

            except socket.error, e:
                print "unexpected error: ", e

    def notify(self, message):
        # 通知客户端
        self.connection.send('%c%c%s' % (0x81, len(message), message))

    def handshake(self):
        data = self.connection.recv(1024)
        headers = self.parse_headers(data)
        token = self.generate_token(headers['Sec-WebSocket-Key'])
        self.connection.send('\
HTTP/1.1 101 WebSocket Protocol Hybi-10\r\n\
Upgrade: WebSocket\r\n\
Connection: Upgrade\r\n\
Sec-WebSocket-Accept: %s\r\n\r\n' % token)

    def parse_data(self, msg):
        # 断线
        if len(msg) == 0:
            return False, 0, -2
        if ord(msg[0]) & 0x0f == 8:
            return False, 0, -1
        v = ord(msg[1]) & 0x7f
        if v == 0x7e:
            p = 4
        elif v == 0x7f:
            p = 10
        else:
            p = 2
        mask = msg[p:p + 4]
        data = msg[p + 4:]
        data = ''.join([chr(ord(v) ^ ord(mask[k % 4])) for k, v in enumerate(data)])
        if data.startswith("@") and data.endswith("!"):
            array = data[1: -1].split(",")
            method = array[0]
            parm = array[1]
            return True, method, parm
        return False, 0, 0

    def parse_headers(self, msg):
        headers = {}
        header, data = msg.split('\r\n\r\n', 1)
        for line in header.split('\r\n')[1:]:
            key, value = line.split(': ', 1)
            headers[key] = value
        headers['data'] = data
        return headers

    def generate_token(self, msg):
        key = msg + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
        ser_key = hashlib.sha1(key).digest()
        return base64.b64encode(ser_key)


# 服务端
class websocket_server(threading.Thread):
    def __init__(self, port):
        super(websocket_server, self).__init__()
        self.port = port
        self.controller = Controller()

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('192.168.1.1', self.port))
        sock.listen(5)
        print 'web socket server started!'
        while True:
            connection, address = sock.accept()
            try:
                username = "ID" + str(address[1])
                thread = websocket_thread(connection, self.controller)
                thread.start()
                print "connecting request from %s" % username
            except socket.timeout:
                print 'web socket connection timeout!'


if __name__ == '__main__':
    server = websocket_server(3690)
    server.start()
