import socket
import numpy as np
from threading import Thread, Event
from multiprocessing import Process, Queue
from PyQt5.QtCore import QThread, pyqtSignal


class LIDARConnector(QThread):
	finished = pyqtSignal(int,object)

	def __init__(self):

		# self.lidar_queue = queue

		self.lidarClient = None    
		
		self.connChk = False    
		self.recvChk = False
		self.event = Event()
		
		self.data_size = int(1206)
		self.channel = int(32)
		self.max_len = 150
		self.x = None
		self.y = None
		self.z = None
		self.Intensity = None
		self.VerticalAngleDeg = np.array([[-30.67,-9.33,-29.33,-8.0,-28.0,-6.67,-26.67,-5.33,-25.33,-4,-24,-2.67,-22.67,-1.33,-21.33,
                            0.0,-20.,1.33,-18.67,2.67,-17.33,4,-16,5.33,-14.67,6.67,-13.33,8,-12,9.33,-10.67,10.67]])

	def __del__(self):
		print("lidar_del")

	
	def connect(self, networktype, host, port, topic):
		self.networkType = networktype

		if self.networkType == 'UDP':
			# try:
			self.lidarClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			self.lidarClient.setblocking(False)
			self.lidarClient.settimeout(1)
			self.lidarClient.bind((host,port))
			

			self.queue = Queue()

			parser_thread = Thread(target=self.data_parser)
			parser_thread.daemon = True 
			parser_thread.start() 

			recv_thread = Thread(target=self.recv_udp_data)
			recv_thread.daemon = True
			recv_thread.start()
			
			# precv_mp = Process(target=self.recv_udp_data,args=(self.queue,))
			# precv_mp.start()


			# except Exception as e:        
			# 	print('lidar_connect : {e}')

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
	def recv_udp_data(self):
		while 1:  
			buffer_chunks = []  
			for _ in range(self.max_len): #150 
				UnitBlock, _ = self.lidarClient.recvfrom(self.data_size)                    
				buffer_chunks.append(UnitBlock[:1200])
			self.Buffer = b''.join(buffer_chunks)    
			self.queue.put(self.Buffer)

	def data_parser(self):
		
		while 1:
			try:
								
				Buffer= self.queue.get()            
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
				
			
			except Exception as e :
				print(f'lidar_data : {e}')

	def sph2cart(self, R, a):
		x = R * np.cos(np.deg2rad(self.VerticalAngleDeg)) * np.sin(np.deg2rad(a))
		y = R * np.cos(np.deg2rad(self.VerticalAngleDeg)) * np.cos(np.deg2rad(a))
		z = R * np.sin(np.deg2rad(self.VerticalAngleDeg))
      
		return x.reshape([-1]), y.reshape([-1]), z.reshape([-1])


	def getLidar(self):
		return (self.x, self.y ,self.z, self.Intensity)