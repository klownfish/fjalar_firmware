import serial
from threading import Thread
import pandas as pd
import schema_pb2 as schema
from collections import defaultdict
from google.protobuf import json_format
import sys

# import hvplot.streamz
# from streamz.dataframe import DataFrame

ALIGNMENT_BYTE = 0x33
CRC_POLY= 0x1011
CRC_SEED = 0x35
BAUDRATE=1000000

class FjalarParser:
    def __init__(self, path):
        try:
            self.stream = serial.Serial(port=path, baudrate=BAUDRATE)
            self.stream_is_file = False
            print("opened serial")
        except:
            # self.stream_is_file = True
            # try:
            #     self.stream = open(path, "rb")
            #     print("opened file")
            # except:
            sys.exit()

        self.data = defaultdict(lambda: ([], []))
        self.reader_t = Thread(target=self.reader_thread)
        self.reader_t.start()

    def read(self, len):
        return self.stream.read(len)

    def write(self, buf):
        self.stream.write(buf)

    def write_message(self, msg):
        buf = msg.serializeToString()
        self.write(buf)

    def reader_thread(self):
        while True:
            alignment = self.read(1)[0]
            if (alignment != ALIGNMENT_BYTE):
                print("invalid byte")
                continue
            length = self.read(1)[0]
            data = self.read(length)
            crc = 0
            crc = self.read(1)[0]
            crc += self.read(1)[0] << 8
            all_bytes = bytes([alignment, length]) + data
            received_crc = self.crc16(CRC_POLY, CRC_SEED, all_bytes)
            if (crc != received_crc):
                print("invalid crc", received_crc, crc)
                continue
            msg = schema.FjalarMessage()
            msg.ParseFromString(data)
            time = msg.time / 1000
            print(length)
            data = json_format.MessageToDict(msg.data, including_default_value_fields=True)
            try:
                data = data[next(iter(data.keys()))] # ignore the message name
                for field in data.keys():
                    self.data[field][0].append(time)
                    self.data[field][1].append(data[field])
            except:
                print("invalid data", data)
    def crc16(self, poly, seed, buf):
        crc = seed
        for byte in buf:
            crc ^= (byte << 8)

            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ poly
                else:
                    crc = crc << 1

        return crc & 0xFFFF
