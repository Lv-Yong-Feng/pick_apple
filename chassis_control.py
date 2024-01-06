import threading
import serial
import binascii


class ChassisControl:

    def __init__(self, port='COMx'):

        self.ch1 = 0
        self.ch2 = 0
        self.ch3 = 0
        self.ch4 = 0
        self.wheel = 0
        self.sw_left = 0

        self.ser = serial.Serial(port=port, baudrate=9600, timeout=0.5)

        if self.ser.is_open:
            print("chassis port open success!")

            thread = threading.Thread(target=self.thread_receiving, daemon=True)
            thread.start()

        else:
            print("chassis port open error!")

    def thread_receiving(self):

        while True:
            while self.ser.read(1)[0] != 0x55: pass

            recv_data = self.ser.read(12)
            recv_data_hex = binascii.hexlify(recv_data)

            GetRealNum = lambda x: x if x < 32768 else (x - 65536)

            self.ch1 = GetRealNum(int(recv_data_hex[0:4], 16))
            self.ch2 = GetRealNum(int(recv_data_hex[4:8], 16))
            self.ch3 = GetRealNum(int(recv_data_hex[8:12], 16))
            self.ch4 = GetRealNum(int(recv_data_hex[12:16], 16))
            self.wheel = GetRealNum(int(recv_data_hex[16:20], 16))
            self.sw_left = int(recv_data_hex[20:22], 16)

            #.。 print("ch1:"+str(self.ch1)+" ch2:"+str(self.ch2)+" ch3:"+str(self.ch3)+" ch4:"+str(self.ch4)+" wheel:"+str(self.wheel)+" sw_left:"+str(self.sw_left))

    def run(self, vx, vy, vw):

        cmd = [0xAA, 0, 0, 0, 0, 0, 0, 0x00]

        cmd[1] = (vx >> 8) & 0xFF
        cmd[2] = vx & 0xFF
        cmd[3] = (vy >> 8) & 0xFF
        cmd[4] = vy & 0xFF
        cmd[5] = (vw >> 8) & 0xFF
        cmd[6] = vw & 0xFF
        cmd[7] = sum(cmd[1:7]) & 0xFF

        send_data = bytearray(cmd)
        self.ser.write(send_data)
