import socket
from threading import Thread, Event


import struct

class IMUConnector:
  def __init__(self, network_type):
    self.imuClient = None    
    self.networkType = network_type
    self.connChk = False
    self.recvChk = False
    self.event = Event()

    self.imu_data = IMUINFO()
  
  def __del__(self):
    print('imu_del')

  def connect(self, host, port, topic):
    if self.networkType == 'UDP':
      try:
        self.imuClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.imuClient.setblocking(False)
        self.imuClient.settimeout(1)
        self.imuClient.bind((host,port))        

        self.imuRecvThread = Thread(target = self.imu, args=())
        self.imuRecvThread.daemon = True 
        self.imuRecvThread.start()

      except Exception as e:
        print(f'imu_connect : {e}')

    else:
        import rospy
        from sensor_msgs.msg import Imu
        self.imuClient = rospy.Subscriber(topic,Imu,self.imuCB)
        try:
          rospy.wait_for_message(topic,Imu,timeout=1)
        except rospy.exceptions.ROSException:
          pass

    self.connChk = True
          

  def imuCB(self,data):
    self.imu_data.orientation_x = data.orientation.x
    self.imu_data.orientation_y = data.orientation.y
    self.imu_data.orientation_z = data.orientation.z
    self.imu_data.orientation_w = data.orientation.w
    self.imu_data.angular_velocity_x = data.angular_velocity.x
    self.imu_data.angular_velocity_y = data.angular_velocity.y
    self.imu_data.angular_velocity_z = data.angular_velocity.z
    self.imu_data.linear_acceleration_x = data.linear_acceleration.x
    self.imu_data.linear_acceleration_y = data.linear_acceleration.y
    self.imu_data.linear_acceleration_z = data.linear_acceleration.z

    self.recvChk = True
  
  def disconnect(self):
    if self.networkType == 'UDP':
      if self.connChk:
        self.connChk = False
        if self.imuRecvThread.is_alive():
          self.event.set()
          self.imuRecvThread.join()
        self.imuClient.close()
    else:
      self.imuClient.unregister()
      self.imuClient = None

  def imu(self):    
    while True:
      try:
        if self.event.is_set():
          break
        raw_data, sender = self.imuClient.recvfrom(65535)
        header = raw_data[0:9].decode()
        if header == '#IMUData$':
            data_length = struct.unpack('i', raw_data[9:13])
            data = struct.unpack('10d', raw_data[25:105])
            self.imu_data.orientation_x = data[1]
            self.imu_data.orientation_y = data[2]
            self.imu_data.orientation_z = data[3]
            self.imu_data.orientation_w = data[0]
            self.imu_data.angular_velocity_x = data[4]
            self.imu_data.angular_velocity_y = data[5]
            self.imu_data.angular_velocity_z = data[6]
            self.imu_data.linear_acceleration_x = data[7]
            self.imu_data.linear_acceleration_y = data[8]
            self.imu_data.linear_acceleration_z = data[9]
            
            self.recvChk = True

      except socket.timeout:
        if self.recvChk:
          continue
        else:
          self.recvChk = False
          break
      except Exception as e:
        print(f'imu_data :{e}')
      
  def getIMU(self):
    return self.imu_data

class IMUINFO :
  def __init__(self):
    self.orientation_x = None
    self.orientation_y = None
    self.orientation_z = None
    self.orientation_w = None
    self.angular_velocity_x = None
    self.angular_velocity_y = None
    self.angular_velocity_z = None
    self.linear_acceleration_x = None
    self.linear_acceleration_y = None
    self.linear_acceleration_z = None 



  def getIMU(self):
    return self.imu_data