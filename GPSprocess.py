import socket
from threading import Thread, Event

import pynmea2

class GPSConnector:
  def __init__(self, network_type): 
    self.gpsClient = None
    self.networkType = network_type
    self.connChk = False        
    self.recvChk = False
    self.event = Event()

    self.pos_x = 126.773287
    self.pos_y = 37.229319
  
  def __del__(self):
    print('gps_del')

  def connect(self, host, port, topic):
    if self.networkType == 'UDP':
      try:
        self.gpsClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gpsClient.setblocking(False)
        self.gpsClient.settimeout(1)
        self.gpsClient.bind((host,port))       

        self.gpsRecvThread = Thread(target = self.position, args=())
        self.gpsRecvThread.setDaemon(True)
        self.gpsRecvThread.start()

      except Exception as e:
        print(f'gps_connect :{e}')
        
    else:
        from morai_msgs.msg import GPSMessage
        import rospy   
        self.gpsClient = rospy.Subscriber(topic ,GPSMessage, self.gpsCB)
        try:
          rospy.wait_for_message(topic,GPSMessage,timeout=1)
        except rospy.exceptions.ROSException:
          pass

    self.connChk = True

  def gpsCB(self,data):    
    self.pos_x = data.longitude
    self.pos_y = data.latitude
    self.recvChk = True

  def disconnect(self):
    if self.networkType == 'UDP':
      if self.connChk:
        self.connChk = False
        if self.gpsRecvThread.is_alive():
          self.event.set()
          self.gpsRecvThread.join()
        self.gpsClient.close()
      
    else:
      self.gpsClient.unregister()
    

  def position(self):
    while True:
      try:
        if self.event.is_set():
          break
        datas, host = self.gpsClient.recvfrom(8196)
        asciiDatas = datas.decode('ascii')
        gpdatas = asciiDatas.split('\r\n')
        
        gpgga = pynmea2.parse(gpdatas[0])

        lats = gpgga.latitude
        longs = gpgga.longitude
        self.pos_y = lats
        self.pos_x = longs #longitude is negaitve

        self.recvChk = True

      except socket.timeout:
        if self.recvChk:
          continue
        else:
          self.recvChk = False
          break
      except Exception as e:
        print(f'gps_poistion : {e}')


  def getPose(self):
    return (self.pos_x, self.pos_y)