#!/usr/bin/env python
import time
from agro.porthandler import PortHandler
from textwrap import wrap

class Motor:
    def __init__(self, port: str, id: str, flag="release"):
        self.id_byte = id
        self.port_name = port
        self.connected = True
        self.turned_on = False

        self.max_angle = 35998
        self.max_speed = 720000
        self.max_torque = 18 #max is 20A(10s)
        self.max_pos = 16383
        self.max_acc = 50000

        self.client = PortHandler(port=port)
        if flag == "release":
            self.connect()

    def genPacket(self, input_string):
        return str(input_string + self.client.computeChecksum(input_string).upper())

    def connect(self):        
        self.client.send("3E 1F " + self.id_byte + " 00 5E")
        self.client.send("3E 12 " + self.id_byte + " 00 51")
        self.client.send("3E 16 " + self.id_byte + " 00 55")
        self.client.send("3E 14 " + self.id_byte + " 00 53")
        self.client.send("3E 10 " + self.id_byte + " 00 4F")
        # if not self.connected:
        #     print("Couldn't connect the motor!")
        #     self.__del__()
        #     exit(1)

    def disconnect(self):
        if self.connected:
            packet = "3E 11 " + self.id_byte + " 00 "
            packet += self.client.computeChecksum(packet).upper()
            self.client.send(packet)

    # 1- Implemented!!! Tested!!! Fine!!! 
    def getPID(self): 
        packet = "3E 30 " + self.id_byte + " 00 "
        packet += self.client.computeChecksum(packet).upper()
        pid_packet = self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 12))
        pid_data = [int(i,16) for i in pid_packet]

        angle = {"Kp": pid_data[5], "Ki": pid_data[6]}
        speed = {"Kp": pid_data[7], "Ki": pid_data[8]}
        torque = {"Kp": pid_data[9], "Ki": pid_data[10]}

        pid_info = {"angle": angle, "speed": speed, "torque": torque}
        return pid_info

    
    # 2- Implemented!!! Tested!!! Fine!!!
    def setPID_RAM(self, angle, speed, torque):
        flag = (angle["Ki"] <= 255 and angle["Ki"] >= 0 and angle["Kp"] <= 255 and angle["Kp"] >= 0)
        flag &= (speed["Ki"] <= 255 and speed["Ki"] >= 0 and speed["Kp"] <= 255 and speed["Kp"] >= 0)
        flag &= (torque["Ki"] <= 255 and torque["Ki"] >= 0 and torque["Kp"] <= 255 and torque["Kp"] >= 0)

        if flag:
            packet = "3E 31 " + self.id_byte + " 06 "
            packet += self.client.computeChecksum(packet).upper()

            p = self.hexToList(hex(angle["Kp"])[2:].upper(), byte_count=1) + " " + self.hexToList(hex(angle["Ki"])[2:].upper(), byte_count=1)
            p += " " + self.hexToList(hex(speed["Kp"])[2:].upper(), byte_count=1) + " " + self.hexToList(hex(speed["Ki"])[2:].upper(), byte_count=1)
            p += " " + self.hexToList(hex(torque["Kp"])[2:].upper(), byte_count=1) + " " + self.hexToList(hex(torque["Ki"])[2:].upper(), byte_count=1)
            
            
            packet += " " + p + " " + self.client.computeChecksum(p).upper()
            self.client.send(packet)
        else:
            print("Wrong input values for SetPID_RAM!")


    # 3- Implemented!!! Tested!!! Fine!!!
    def setPID_ROM(self, angle, speed, torque):
        flag = (angle["Ki"] <= 255 and angle["Ki"] >= 0 and angle["Kp"] <= 255 and angle["Kp"] >= 0)
        flag &= (speed["Ki"] <= 255 and speed["Ki"] >= 0 and speed["Kp"] <= 255 and speed["Kp"] >= 0)
        flag &= (torque["Ki"] <= 255 and torque["Ki"] >= 0 and torque["Kp"] <= 255 and torque["Kp"] >= 0)

        if flag:
            packet = "3E 32 " + self.id_byte + " 06 "
            packet += self.client.computeChecksum(packet).upper()
            p = ''
            p += " " + self.hexToList(hex(angle["Kp"])[2:].upper(), byte_count=1) + " " + self.hexToList(hex(angle["Ki"])[2:].upper(), byte_count=1)
            p += " " + self.hexToList(hex(speed["Kp"])[2:].upper(), byte_count=1) + " " + self.hexToList(hex(speed["Ki"])[2:].upper(), byte_count=1)
            p += " " + self.hexToList(hex(torque["Kp"])[2:].upper(), byte_count=1) + " " + self.hexToList(hex(torque["Ki"])[2:].upper(), byte_count=1)
            packet += " " + p + " " + self.client.computeChecksum(p).upper()
            self.client.send(packet)
        else:
            print("Wrong input values for SetPID_ROM!")
            exit(1)

    # 4- Implemented!!! Tested!!! Fine!!!
    def getACC(self):
        packet = "3E 33 " + self.id_byte + " 00 "
        packet += self.client.computeChecksum(packet).upper()
        respond = self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 10))
        return self.hexListToNum(respond[5:9], signed=True)//100

    # 5- Implemented!!! Tested!!! Fine!!!
    def setACC_RAM(self, acc):
        flag = (acc <= abs(self.max_acc))

        if flag:
            packet = "3E 34 " + self.id_byte + " 04 "
            packet += self.client.computeChecksum(packet).upper()
            p = self.numberToHexList(acc, 4)
            packet += " " + p + " " + self.client.computeChecksum(p).upper()
            self.client.send(packet)
        else:
            print("Wrong input value for SetACC_RAM!")
            print("Acceleration will be set to zero ...!")
            self.setACC_RAM(0)

    # 6- Implemented!!! Tested!!! Fine!!!
    def getEncoderData(self):
        packet = "3E 90 " + self.id_byte + " 00 "
        packet += self.client.computeChecksum(packet).upper()
        respond = self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 12))

        current_pos = self.hexListToNum(respond[5:7], signed=False)
        original_pos = self.hexListToNum(respond[7:9], signed=False)
        offset = self.hexListToNum(respond[9:11], signed=False)

        encoder = {"current_pos": current_pos, "original_pos": original_pos, "offset": offset}
        return encoder

    # 7- Implemented!!! Tested!!! Fine!!!
    def setEncoderOffset(self, encoder_offset):
        flag = (encoder_offset >= 0 and encoder_offset <= ((1<<14)-1))
        if flag:
            packet = "3E 91 " + self.id_byte + " 02 "
            packet += self.client.computeChecksum(packet).upper()
            
            p = self.hexToList(hex(encoder_offset)[2:].upper(), 2)
            packet += " " + p + " " + self.client.computeChecksum(p).upper()
            self.client.send(packet)
        else:
            print("Wrong input value for setEncoderOffset()!")
            print("Offset will be set to max")
            self.setEncoderOffset((1<<14)-1)
    
    # 8- Implemented!!! Tested!!! Fine!!!
    # NOTICE: This command will write zero point into ROM of the driver, 
    # multiple writing will affect the chip lifespan, which is not recommended for frequent use
    def resetEncoderOffset(self):
        packet = "3E 19 " + self.id_byte + " 00 "
        packet += self.client.computeChecksum(packet).upper()
        self.client.send(packet)

    # 9- Implemented!!! Tested!!! Fine!!!
    def getMultiLoopAngleData(self):
        packet = "3E 92 " + self.id_byte + " 00 "
        packet += self.client.computeChecksum(packet).upper()
        respond = self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 14))

        return self.hexListToNum(respond[5:13], signed=True) 

    # 10- Implemented!!! Tested!!! Fine!!!
    def getSingleLoopAngle(self):
        packet = "3E 94 " + self.id_byte + " 00 "
        print("sina", self.id_byte)
        packet += self.client.computeChecksum(packet).upper()
        respond = self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 10))

        return self.hexListToNum(respond[5:9], signed=False)

    # 11- Implemented!!! Tested!!! Fine!!!
    def getMotorState1(self):
        packet = "3E 9A " + self.id_byte + " 00 "
        packet += self.client.computeChecksum(packet).upper()
        respond = self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 13))

        temperature = int(respond[5], 16)
        voltage = self.hexListToNum(respond[7:9], signed=False)

        error = format(int(respond[11],16), '0>8b')[::-1]
        error_state = self.checkErrorState(error)

        return {"temperature":temperature, "voltage":voltage, "error_state":error_state}

    # 12- Implemented!!! Tested!!! Fine!!!
    def clearMotorError(self):
        packet = "3E 9B " + self.id_byte + " 00 "
        packet += self.client.computeChecksum(packet).upper()
        respond = self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 13))

        temperature = int(respond[5], 16)
        voltage = self.hexListToNum(respond[7:9], signed=False)
        error = format(int(respond[11],16), '0>8b')[::-1]
        error_state = self.checkErrorState(error)

        return {"temperature":temperature, "voltage":voltage, "error_state":error_state}

    # 13- Implemented!!! Tested!!! Fine!!!
    def getMotorState2(self):
        packet = "3E 9C " + self.id_byte + " 00 "
        packet += self.client.computeChecksum(packet).upper()
        respond = self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 13))

        temperature = int(respond[5], 16)
        torque = self.hexListToNum(respond[6:8], signed=True)
        speed = self.hexListToNum(respond[8:10], signed=True)
        pos = self.hexListToNum(respond[10:12], signed=False)

        return {"temperature":temperature, "torque":torque, "speed":speed, "pos":pos}

    # 14- Implemented!!! Tested!!! Fine!!!
    def getMotorState3(self):
        packet = "3E 9D " + self.id_byte + " 00 "
        packet += self.client.computeChecksum(packet).upper()
        respond = self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 13))

        temperature = int(respond[5], 16)
        phase_A_current = self.hexListToNum(respond[6:8], signed=True)
        phase_B_current = self.hexListToNum(respond[8:10], signed=True)
        phase_C_current = self.hexListToNum(respond[10:12], signed=True)

        return {"temperature":temperature, "phase_A_current":phase_A_current, "phase_B_current":phase_B_current, "phase_C_current":phase_C_current}


    # Additional- 
    def getState(self):
        state_1 = self.getMotorState1()
        state_2 = self.getMotorState2()
        state_3 = self.getMotorState3()
        if state_1['error_state'] != "Normal" :
            self.clearMotorError()
        return {'port':self.port_name,\
                'error_state':state_1['error_state'],\
                'temperature':state_2['temperature'],\
                'voltage':state_1['voltage'],\
                'torque':state_2['torque'],\
                'speed':state_2['speed'],\
                'pos':state_2['pos'],\
                "phase_A_current":state_3["phase_A_current"],\
                "phase_B_current":state_3["phase_B_current"],\
                "phase_C_current":state_3["phase_C_current"]
                }
        # return [self.port_name, state_1['error_state'], state_1['temperature'], state_1['voltage'], state_2['torque'],\
        #         state_2['speed'], state_2['pos'], state_3["phase_A_current"], state_3["phase_B_current"], state_3["phase_C_current"]]
        # s = State()
        # s.name = self.port_name
        # s.error_state = state_1['error_state']
        # s.temperature = state_1['temperature']
        # s.voltage = state_1['voltage']
        # s.torque = state_2['torque']
        # s.speed = state_2['speed']
        # s.pos = state_2['pos']
        # s.phase_A_current = state_3["phase_A_current"]
        # s.phase_B_current = state_3["phase_B_current"]
        # s.phase_C_current = state_3["phase_C_current"]
    
    # 15- Implemented!!! Tested!!! Fine!!!
    def turnOff(self):
        self.client.send("3E 80 " + self.id_byte + " 00 BF")

    
    # 16- Implemented!!! Tested!!! Fine!!!
    def stop(self):
        self.client.send("3E 81 " + self.id_byte + " 00 C0")

    # 17- Implemented!!! Tested!!! Fine!!!
    def turnOn(self):
        if not self.connected:
            print("ERROR: You should connect() first!\n")
            exit(1)
        # if not self.turned_on:
        self.client.send("3E 88 " + self.id_byte + " 00 C7")
        # if not self.client.client.read(5) == bytearray.fromhex("3E 88 01 00 C7"):
        self.turned_on = True

    # 19- Implemented!!! Tested!!! Fine!!!
    def torqueClosedLoop(self, current_val:int):
        # corresponding to the actual current range -32 upto 32
        flag = (abs(current_val) <= self.max_torque)

        if flag:
            packet = "3E A1 " + self.id_byte + " 02 "
            packet += self.client.computeChecksum(packet).upper()

            power_hx = self.numberToHexList(current_val*100, byte_count=2)
            packet += " " + power_hx + " " + self.client.computeChecksum(power_hx).upper()
            
            self.client.send(packet)
            # respond = self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 13))
            # temperture = int(respond[5], 16)
            # torque = self.hexListToNum(respond[6:8], signed=True)
            # speed = self.hexListToNum(respond[8:10], signed=True)
            # pos = self.hexListToNum(respond[10:12], signed=False)
            # return {"temperature":temperture, "torque": torque, "speed":speed, "pos":pos}
        else:
            print("Wrong input value for torqueClosedLoop()!")
            print("Torque will be set to zero ...!")
            self.torqueClosedLoop(0)

    # 20- Implemented!!! Tested!!! Fine!!!
    def speedClosedLoop(self, speed:float):
        flag = (abs(speed) <= self.max_speed-1)
        if flag:
            packet = "3E A2 " + self.id_byte + " 04 "
            packet += self.client.computeChecksum(packet).upper()

            speed_hx = self.numberToHexList(int(speed*1000), 4)
            packet += " " + speed_hx + " " + self.client.computeChecksum(speed_hx).upper()

            self.client.send(packet)
            # respond = self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 13))
            # speed = self.hexListToNum(respond[8:10], signed=True)
            # return (speed//100) if abs(speed) > 100 else speed

        else:
            print("Wrong input value for speedClosedLoop()!")
            print("Speed will be set to zero ...!")
            self.speedClosedLoop(0)
        # respond = self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 13))
        # temperture = int(respond[5], 16)
        # torque = self.hexListToNum(respond[6:8], signed=True)
        
        # pos = self.hexListToNum(respond[10:12],signed=False)
        # return {"temperature":temperture, "torque": torque, "speed":speed, "pos":pos}
        

    # 21- Implemented!!! Tested!!! Fine!!!
    def multiPosClosedLoop1(self, angle:int):
        flag = (abs(angle) <= (self.max_angle-1))
        if flag:
            packet = "3E A3 " + self.id_byte + " 08 "
            packet += self.client.computeChecksum(packet).upper()

            angle_hx = self.numberToHexList(angle, 8)
            packet += " " + angle_hx + " " + self.client.computeChecksum(angle_hx).upper()

            self.client.send(packet)
            # respond = self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 13))
            # temperture = int(respond[5], 16)
            # torque = self.hexListToNum(respond[6:8], signed=True)
            # speed = self.hexListToNum(respond[8:10], signed=True)
            # pos = self.hexListToNum(respond[10:12], signed=False)
            # return {"temperature":temperture, "torque": torque, "speed":speed, "pos":pos}
        else:
            print("Wrong input value for multiPosClosedLoop1()!")
            print("Angle will be set to zero ...!")
            self.multiPosClosedLoop1(0)
    
    # 22- Implemented!!! Tested!!! Fine!!!
    def multiPosClosedLoop2(self, angle:int, speed_limit:int):
        flag = (abs(angle) <= self.max_angle) and (abs(speed_limit) <= self.max_speed)
        packet = "3E A4 " + self.id_byte + " 0C "
        packet += self.client.computeChecksum(packet).upper()

        angle_hx = self.numberToHexList(angle, 8)
        speed_hx = self.numberToHexList(speed_limit, 4)
        p = angle_hx + " " + speed_hx
        packet += " " + p + " " + self.client.computeChecksum(p).upper()

        # print(packet)
        self.client.send(packet)
        # respond = self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 13))
        # temperture = int(respond[5], 16)
        # torque = self.hexListToNum(respond[6:8], signed=True)
        # speed = self.hexListToNum(respond[8:10], signed=True)
        # pos = self.hexListToNum(respond[10:12] signed=False)
        # return {"temperature":temperture, "torque": torque, "speed":speed, "pos":pos}

    # 23- Not working very well
    def singleTurnClosedLoop1(self, clockwise: bool, angle:int):
        flag = (abs(angle) <= self.max_angle)
        if flag: 
            packet = "3E A5 " + self.id_byte + " 04 "
            packet += self.client.computeChecksum(packet).upper()

            spin_direction_hx = "00" if (clockwise == True) else "01"
            angle_hx = self.numberToHexList(angle*100, byte_count=2)
            p = spin_direction_hx + " " + angle_hx + " 00 "
            packet += " " + p + self.client.computeChecksum(p).upper()

            # print(packet)
            self.client.send(packet)
        # respond = self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 13))
        # temperture = int(respond[5], 16)
        # torque = self.hexListToNum(respond[6:8], signed=True)
        # speed = self.hexListToNum(respond[8:10], signed=True)
        # pos = self.hexListToNum(respond[10:12], signed=True)

        # return {"temperature":temperture, "torque": torque, "speed":speed, "pos":pos}

    # 24- Not working very well
    def singleTurnClosedLoop2(self, clockwise: bool, angle: int, speed_limit: int):
        flag = (abs(angle) <= self.max_angle) and (abs(speed_limit) <= self.max_speed)
        
        if flag:
            packet = "3E A6 " + self.id_byte + " 08 "
            packet += self.client.computeChecksum(packet).upper()

            spin_direction_hx = "00" if (clockwise == True) else "01"
            angle_hx = self.numberToHexList(angle, byte_count=2)
            p = spin_direction_hx + " " + angle_hx + " 00 "
            p += self.numberToHexList(speed_limit*100, byte_count=4)
            packet += " " + p + " " + self.client.computeChecksum(p).upper()
            self.client.send(packet)
            # print(self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 13)))


    # 25- No use
    def incrementalClosedLoopSpeed1(self, angle_increment:int):
        packet = "3E A7 " + self.id_byte + " 03 "
        packet += self.client.computeChecksum(packet).upper()

        angle_hx = self.numberToHexList(angle, byte_count=4)
        packet += " " + angle_hx + self.client.computeChecksum(angle_hx).upper()

        respond = self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 13))
        print(respond)


    # 26- No use
    def incrementalClosedLoopSpeed2(self, angle_increment, max_speed):
        if abs(max_speed) >= 16:
            packet = "3E A8 " + self.id_byte + " 08 "
            packet += self.client.computeChecksum(packet).upper()

            angle_hx = self.numberToHexList(angle_increment, byte_count=4)
            speed_hx = self.numberToHexList(max_speed, byte_count=4)

            p = angle_hx + " " + speed_hx
            packet += " " + p + " " + self.client.computeChecksum(p).upper()

            respond = self.client.bytearray_to_hexadecimal(self.client.receive(packet, respond_packet_size = 13))
            if len(respond)>0:
                return {"temperature": int(respond[5], 16), "torque": self.hexListToNum(respond[6:7], signed=True), "speed": self.hexListToNum(respond[8:12], signed=True)}
            else:
                return {}


    # 27- DONE!!!
    def getModel(self):
        packet = "3E 12 " + self.id_byte + " 00 "
        checksum = self.client.computeChecksum(packet).upper()
        out_packet = packet + checksum
        model_packet = self.client.bytearray_to_hexadecimal(self.client.receive(out_packet, respond_packet_size = 48))

        model_info = model_packet[5:47]
        m_info = ''.join('{}'.format(chr(int(x,16))) for x in model_info)
        return m_info
    


    def hexListToNum(self, a, signed:bool):
        b = a.copy()
        b.reverse()
        num_h = ''.join(b)
        unsigned = int(num_h, 16)
        if not signed:
            return int(num_h, 16)
        else:
            bit_count = len(num_h)*4
            max_num = (1<<bit_count)
            mid_num = (1<<(bit_count-1))-1
            return -(max_num - unsigned) if unsigned > mid_num else unsigned

    def hexToList(self, num, byte_count):
        h = format(int(num ,16), '0>' + str(byte_count*2) + 'X')
        return ' '.join(wrap(h, 2))

    def numberToHexList(self, num, byte_count):
        h = format(num & ((1 << (byte_count*8))-1), '0>' + str(byte_count*2) + 'X')
        return ' '.join(wrap(h, 2)[::-1])

    def checkErrorState(self, error):
        error_state = ''
        if error[0] == 1 :
            error_state += "Low voltage protection, "
        elif error[3] == 1 :
            error_state += "Over temperature protection"
        else:
            error_state = "Normal"
        return error_state

    

    def __del__(self):
        self.client.clean_buffer()
        self.stop()
        self.turnOff()
        self.disconnect()