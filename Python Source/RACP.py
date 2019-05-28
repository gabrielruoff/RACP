import serial
import time

class Joint:

    position = 0

    def __init__(self, initPos):
        self.position = initPos

    def setPosition(self, pos):
        self.position = pos

class robotArm:

    def __init__(self, joints):
        self.Joints = joints
        print("joints: ", self.Joints)
        self.armPort = None

    def connect(self, port, baud=4800, timeout=5, bytesize=8, parity='N', stopbits=1):
        self.armPort = serial.Serial(port, baud, bytesize, parity, stopbits, timeout,)
        time.sleep(3)
        self.update()

    def update(self):

        data = ''

        print("Updating position")

        for j in self.Joints:

            if j != self.Joints[-1]:

                data += (str(j.position)).zfill(3) + ':'

            else:

                data += (str(j.position)).zfill(3) + ''

        print("Data:", data.encode(), "raw:", data)

        for character in data:
            print(character)
            self.armPort.write(bytes(character, "utf-8"))

        self.armPort.flush()

    def close(self):
        self.armPort.close()
