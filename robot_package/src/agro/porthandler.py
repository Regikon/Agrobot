#!/usr/bin/env python
import time, serial.rs485
from textwrap import wrap

class PortHandler():
    def __init__(self, port: str):
        self.client = serial.rs485.RS485(port='/dev/'+ port, baudrate = 115200, timeout = 0.5)
        self.client.rs485_mode=serial.rs485.RS485Settings(rts_level_for_tx=True, rts_level_for_rx=True, loopback=False, delay_before_tx=0.005, delay_before_rx=0.01)
        self.name = self.client.name
        self.clean_buffer()

    def send(self, command):
        # self.clean_buffer()
        self.client.reset_output_buffer()
        self.client.write(bytearray.fromhex(command))
        

    def receive(self, command, respond_packet_size):
        self.client.reset_input_buffer()
        self.send(command)
        return self.client.read(respond_packet_size)
    
    def computeChecksum(self, value:str):
        val = value.split()
        res = 0
        for v in val:
            res += int(v, 16)
        c = hex(res)[2:]
        return c[-2:] if len(c) > 2 else format(int(c,16), '0>2X')
    
    def bytearray_to_hexadecimal(self, list_val):
        result = ''.join('{:02x}'.format(x) for x in list_val)  
        return wrap(result.upper(), 2)
    
    # to clean the buffer
    def clean_buffer(self):
        self.client.reset_input_buffer()
        self.client.reset_output_buffer()

    
    def __del__(self):
        self.clean_buffer()
        self.client.close()