import socket
import time
from threading import Thread,Event

import cv2
import numpy as np
import struct

class CAMConnector:
  def __init__(self, network_type):
    self.camClient = None    
    self.networkType = network_type
    self.connChk = False
    self.recvChk = False
    self.event = Event()

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
    
    np_arr = np.fromstring(data.data, np.uint8)
    self.TotalIMG = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    self.recvChk = True
    
  
  def disconnect(self):    
    if self.networkType == 'UDP':
      if self.connChk:
        self.connChk = False
        if self.camRecvThread.is_alive():
          self.event.set()
          self.camRecvThread.join()        
        self.camClient.close()
      
    else:
      self.camClient.unregister()

  def check_max_len(self):
    idx_list = b''
    r_step = 0
    while r_step < int(10):            
        UnitBlock, sender = self.camClient.recvfrom(65000)
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
        UnitBlock, sender = self.camClient.recvfrom(65000)
        UnitIdx = struct.unpack('i',UnitBlock[3:7])[0]
        UnitSize = struct.unpack('i',UnitBlock[7:11])[0]
        UnitTail = UnitBlock[-2:]

        if num_block == UnitIdx:
          TotalBuffer += UnitBlock[11:(11 + UnitSize)]
          num_block += 1   
        if UnitTail == b'EI' and num_block == (UnitIdx + 1):
          self.TotalIMG = cv2.imdecode(np.fromstring(TotalBuffer, np.uint8), 1)
          self.img_byte = np.array(cv2.imencode('.jpg', self.TotalIMG)[1]).tostring()                    
          TotalBuffer = b''
          self.recvChk = True
          break

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
    
    return self.TotalIMG