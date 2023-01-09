import socket
import numpy as np
from threading import Thread, Event


class LIDARConnector:
  def __init__(self, networktype):
    self.lidarClient = None    
    self.networkType = networktype
    self.connChk = False    
    self.recvChk = False
    self.event = Event()
    
    self.data_size = int(1206)
    self.channel = int(32)
    self.max_len = 300
    self.x = None
    self.y = None
    self.z = None
    self.Intensity = None
    self.VerticalAngleDeg = np.array([[-30.67,-9.33,-29.33,-8.0,-28.0,-6.67,-26.67,-5.33,-25.33,-4,-24,-2.67,-22.67,-1.33,-21.33,
                            0.0,-20.,1.33,-18.67,2.67,-17.33,4,-16,5.33,-14.67,6.67,-13.33,8,-12,9.33,-10.67,10.67]])

  def __del__(self):
    print("lidar_del")

  def connect(self, host, port, topic):
    if self.networkType == 'UDP':
      try:
        self.lidarClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lidarClient.setblocking(False)
        self.lidarClient.settimeout(1)
        self.lidarClient.bind((host,port))
        
        self.lidarRecvThread = Thread(target = self.lidar, args=())
        self.lidarRecvThread.daemon = True 
        self.lidarRecvThread.start()

      except Exception as e:        
        print('lidar_connect : {e}')
        

    else:
        import rospy
        from sensor_msgs.msg import PointCloud2        
        self.lidarClient = rospy.Subscriber(topic, PointCloud2, self.lidarCB)
        try:
          rospy.wait_for_message(topic,PointCloud2,timeout=1)
        except rospy.exceptions.ROSException:
          pass

    self.connChk = True

  def disconnect(self):
    if self.networkType == 'UDP':
      if self.connChk:
        self.connChk = False
        if self.lidarRecvThread.is_alive():
          self.event.set()
          self.lidarRecvThread.join()
        self.lidarClient.close()

    else:
      self.lidarClient.unregister()
  
  #ROS
  def lidarCB(self, data):
    import sensor_msgs.point_cloud2
    x = []
    y = []
    z = []
    for point in sensor_msgs.point_cloud2.read_points_list(data, skip_nans=True):
      y.append(point[0])
      x.append(-point[1])
      z.append(point[2])

    self.x = x
    self.y = y
    self.z = z
    self.recvChk = True
  
  #UDP
  def lidar(self):
    while True:
      
      try:
        if self.event.is_set():
          break

        Buffer = b''
        for _ in range(self.max_len):
            UnitBlock, _ = self.lidarClient.recvfrom(self.data_size)            
            Buffer += UnitBlock[:1200]

        Buffer_np = np.frombuffer(Buffer, dtype=np.uint8).reshape([-1, 100])

        if self.channel == 16:
            Azimuth = np.zeros((24 * self.max_len,))
            Azimuth[0::2] = Buffer_np[:,2].astype(np.float32) + 256*Buffer_np[:,3].astype(np.float32)
            Azimuth[1::2] = Buffer_np[:,2].astype(np.float32) + 256*Buffer_np[:,3].astype(np.float32) + 20
        else:
            Azimuth = Buffer_np[:,2] + 256*Buffer_np[:,3]
        
        Distance = (Buffer_np[:,4::3].astype(np.float32) + 256*Buffer_np[:,5::3].astype(np.float32))*2
        Intensity = Buffer_np[:,6::3].astype(np.float32)

        # reshape outputs based on 16 channels
        Azimuth = Azimuth.reshape([-1, 1])/100
        Distance = Distance.reshape([-1, self.channel])/1000
        Intensity = Intensity.reshape([-1])

        self.x, self.y, self.z = self.sph2cart(Distance, Azimuth)
        self.recvChk = True

      except socket.timeout:
        if self.recvChk:
          continue
        else:
          self.recvChk = False
          break
      except Exception as e :
        print(f'lidar_data : {e}')

  def sph2cart(self, R, a):
      x = R * np.cos(np.deg2rad(self.VerticalAngleDeg)) * np.sin(np.deg2rad(a))
      y = R * np.cos(np.deg2rad(self.VerticalAngleDeg)) * np.cos(np.deg2rad(a))
      z = R * np.sin(np.deg2rad(self.VerticalAngleDeg))
      
      return x.reshape([-1]), y.reshape([-1]), z.reshape([-1])


  def getLidar(self):
    return (self.x, self.y ,self.z, self.Intensity)