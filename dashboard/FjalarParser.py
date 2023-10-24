import serial
from threading import Thread
import pandas as pd
import schema_pb2 as schema
from collections import defaultdict, deque
from google.protobuf import json_format
import sys
import time

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
        self.message_que = deque(maxlen=100)
        self._reader_t = Thread(target=self._reader_thread)
        self._reader_t.start()

    def _read(self, len):
        return self.stream.read(len)

    def _write(self, buf):
        self.stream.write(buf)

    def _write_message(self, msg):
        msg_buf = msg.SerializeToString()
        output_buf = bytes([ALIGNMENT_BYTE, len(msg_buf)]) + msg_buf
        crc = self._crc16(CRC_POLY, CRC_SEED, output_buf)
        output_buf += bytes([crc & 0x00ff, (crc >> 8) & 0x00ff ])
        self._write(output_buf)

    def _reader_thread(self):
        while True:
            alignment = self._read(1)[0]
            if (alignment != ALIGNMENT_BYTE):
                print("invalid byte")
                continue
            length = self._read(1)[0]
            data = self._read(length)
            received_crc = 0
            received_crc = self._read(1)[0]
            received_crc += self._read(1)[0] << 8
            all_bytes = bytes([alignment, length]) + data
            crc = self._crc16(CRC_POLY, CRC_SEED, all_bytes)
            if (crc != received_crc):
                print("invalid crc", received_crc, crc)
                continue
            msg = schema.FjalarMessage()
            msg.ParseFromString(data)
            self.message_que.append(msg)
            time = msg.time / 1000
            data = json_format.MessageToDict(msg.data, including_default_value_fields=True)
            print(data)
            try:
                data = data[next(iter(data.keys()))] # ignore the message name
                for field in data.keys():
                    self.data[field][0].append(time)
                    self.data[field][1].append(data[field])
            except:
                print("invalid data", data)

    def _crc16(self, poly, seed, buf):
        crc = seed
        for byte in buf:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ poly
                else:
                    crc = crc << 1
        return crc & 0xFFFF

    def clear_flash(self):
        msg = schema.FjalarMessage()
        data_msg = schema.FjalarData()
        clear_flash_msg = schema.ClearFlash()
        data_msg.clear_flash.CopyFrom(clear_flash_msg)
        msg.data.CopyFrom(data_msg)
        self._write_message(msg)

    def enter_idle(self):
        msg = schema.FjalarMessage()
        data_msg = schema.FjalarData()
        enter_idle_msg = schema.EnterIdle()
        data_msg.enter_idle.CopyFrom(enter_idle_msg)
        msg.data.CopyFrom(data_msg)
        self._write_message(msg)

    def ready_up(self):
        msg = schema.FjalarMessage()
        data_msg = schema.FjalarData()
        ready_up_msg = schema.ReadyUp()
        data_msg.ready_up.CopyFrom(ready_up_msg)
        msg.data.CopyFrom(data_msg)
        self._write_message(msg)

    def toggle_sudo(self):
        msg = schema.FjalarMessage()
        data_msg = schema.FjalarData()
        set_sudo_msg = schema.SetSudo()
        set_sudo_msg.enabled = not self.data["sudo"][1][-1]
        data_msg.set_sudo.CopyFrom(set_sudo_msg)
        msg.data.CopyFrom(data_msg)
        self._write_message(msg)

    def read_flash(self, index, length):
        msg = schema.FjalarMessage()
        data_msg = schema.FjalarData()
        read_flash_msg = schema.ReadFlash()
        read_flash_msg.start_index = index
        read_flash_msg.length = length
        data_msg.read_flash.CopyFrom(read_flash_msg)
        msg.data.CopyFrom(data_msg)
        self._write_message(msg)

        start = time.time()
        index_to_read = 0

        while True:
            if time.time() - start > 1:
                return []
            if len(self.message_que) > 0:
                msg = self.message_que.pop()
                if msg.data.HasField("flash_data"):
                    if msg.data.flash_data.start_index == index and len(msg.data.flash_data.data) == length:
                        return msg.data.flash_data.data
                    else:
                        print(msg.data.flash_data.start_index, index, len(msg.data.flash_data.data), length)
                        print("got a different flash message somewhow")