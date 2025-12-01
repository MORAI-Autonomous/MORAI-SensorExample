import os
import platform
import socket
import struct
import time
from threading import Event, Lock, Thread

if platform.system() == 'Linux':
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = ''

import cv2
import numpy as np


class CAMConnector:
    def __init__(self, network_type):
        self.camClient = None
        self.networkType = network_type
        self.connChk = False
        self.recvChk = False
        self.event = Event()
        self.lock = Lock()

        self.TotalIMG = None

    def __del__(self):
        print('cam_del')

    def connect(self, host, port, topic):
        if self.networkType == 'UDP':
            try:
                self.camClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.camClient.setblocking(False)
                self.camClient.settimeout(1)
                self.camClient.bind((host,port))
                self.check_max_len()
                self.camRecvThread = Thread(target = self.loop, args=())
                self.camRecvThread.daemon = True
                self.camRecvThread.start()

            except Exception as e:
                print(f'cam_connect : {e}')

        else:
            import rospy
            from sensor_msgs.msg import CompressedImage
            self.camClient = rospy.Subscriber(topic, CompressedImage, self.camCB)
            try:
                rospy.wait_for_message(topic,CompressedImage,timeout=1)
            except rospy.exceptions.ROSException:
                pass

        self.connChk = True

    def camCB(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)
        with self.lock:
            self.TotalIMG = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.recvChk = True

    def disconnect(self):
        if self.networkType == 'UDP':
            if self.connChk:
                self.connChk = False
            if hasattr(self, 'camRecvThread') and self.camRecvThread.is_alive():
                self.event.set()
                self.camRecvThread.join()
            if self.camClient:
                self.camClient.close()
                self.camClient = None
        else:
            if self.camClient:
                self.camClient.unregister()
                self.camClient = None

    def check_max_len(self):
        idx_list = b''
        r_step = 0
        while r_step < int(10):
            UnitBlock, _sender = self.camClient.recvfrom(65000)
            idx_list += UnitBlock[3:7]
            r_step += 1

    def loop(self):
        while True:
            self.image()
            if self.event.is_set():
                break

    def image(self):
        TotalBuffer = b''
        num_block = 0
        while True:
            try:
                UnitBlock, _sender = self.camClient.recvfrom(65000)
                UnitIdx = struct.unpack('i',UnitBlock[11:15])[0]
                UnitSize = struct.unpack('i',UnitBlock[15:19])[0]
                UnitData = UnitBlock[19:-2]
                UnitTail = UnitBlock[-2:]

                if UnitTail == b'EI':
                    TotalBuffer += UnitData

                    decoded_img = cv2.imdecode(np.frombuffer(TotalBuffer, np.uint8), 1)
                    with self.lock:
                        self.TotalIMG = decoded_img
                        self.recvChk = True
                    TotalBuffer = b''
                    break
                else:
                    TotalBuffer += UnitData
            
            except socket.timeout:
                if self.recvChk:
                    continue
                else:
                    self.recvChk = False
                    break
            except Exception as e:
                print(f'cam_image : {e}')

            time.sleep(0.01)

    def getImg(self):
        with self.lock:
            return self.TotalIMG