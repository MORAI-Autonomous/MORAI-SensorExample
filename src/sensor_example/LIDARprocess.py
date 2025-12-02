import socket
from queue import Queue, Empty
from threading import Thread

import numpy as np
from PySide2.QtCore import QObject, Signal


class LIDARConnector(QObject):
    # Signals
    pointCloudReady = Signal(object, object, object, object)  # x, y, z, intensity
    connectionStatusChanged = Signal(bool)  # True=connected, False=disconnected
    connectionError = Signal(str)  # Error message
    dataRateChanged = Signal(float)  # Updates per second
    pointCountChanged = Signal(int)  # Number of points in cloud

    def __init__(self):
        super().__init__()
        self.lidarClient = None
        self.connChk = False
        self.recvChk = False

        self.data_size = int(1206)
        self.channel = int(32)
        self.max_len = 150
        self.vertical_angle_deg = np.array([[-30.67,-9.33,-29.33,-8.0,-28.0,-6.67,-26.67,-5.33,-25.33,-4,-24,-2.67,-22.67,-1.33,-21.33,
                            0.0,-20.,1.33,-18.67,2.67,-17.33,4,-16,5.33,-14.67,6.67,-13.33,8,-12,9.33,-10.67,10.67]])

    def __del__(self):
        print("lidar_del")

    def connect_sensor(self, networktype, host, port, topic):
        self.networkType = networktype

        if self.networkType == 'UDP':
            try:
                self.lidarClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.lidarClient.setblocking(False)
                self.lidarClient.settimeout(1)
                self.lidarClient.bind((host,port))

                self.queue = Queue(maxsize=100)
                self.connChk = True

                self.recv_thread = Thread(target=self.recv_udp_data)
                self.recv_thread.daemon = True
                self.recv_thread.start()

                self.parser_thread = Thread(target=self.data_parser)
                self.parser_thread.daemon = True
                self.parser_thread.start()

                # precv_mp = Process(target=self.recv_udp_data,args=(self.queue,))
                # precv_mp.start()

            except Exception as e:
                error_msg = f'lidar_connect : {e}'
                print(error_msg)
                self.connectionError.emit(error_msg)
                return

        else:
            try:
                import rospy
                from sensor_msgs.msg import PointCloud2
                self.lidarClient = rospy.Subscriber(topic, PointCloud2, self.lidarCB)
                try:
                    rospy.wait_for_message(topic,PointCloud2,timeout=1)
                except rospy.exceptions.ROSException:
                    pass
            except ImportError:
                return

        self.connChk = True
        self.connectionStatusChanged.emit(True)

    def disconnect_sensor(self):
        if self.networkType == 'UDP':
            if self.connChk:
                self.connChk = False
                self.connectionStatusChanged.emit(False)
                # Give threads time to finish
                if hasattr(self, 'recv_thread'):
                    try:
                        self.recv_thread.join(timeout=1)
                    except:
                        pass
                if hasattr(self, 'parser_thread'):
                    try:
                        self.parser_thread.join(timeout=1)
                    except:
                        pass
                if self.lidarClient:
                    self.lidarClient.close()
                    self.lidarClient = None
        else:
            if self.lidarClient:
                self.lidarClient.unregister()
                self.lidarClient = None

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

        self.recvChk = True
        
        # Emit signal with point cloud data
        self.pointCloudReady.emit(x, y, z, None)
        self.pointCountChanged.emit(len(x))

    #UDP
    def recv_udp_data(self):
        while self.connChk:
            try:
                buffer_chunks = []
                for _ in range(self.max_len): #150
                    UnitBlock, _ = self.lidarClient.recvfrom(self.data_size)
                    buffer_chunks.append(UnitBlock[:1200])
                self.Buffer = b''.join(buffer_chunks)
                if not self.queue.full():
                    self.queue.put(self.Buffer)
            except socket.timeout:
                if self.recvChk:
                    continue
                else:
                    self.recvChk = False
                    break
            except Exception as e:
                if self.connChk:
                    error_msg = f'lidar_recv_udp: {e}'
                    print(error_msg)
                    self.connectionError.emit(error_msg)
                break

    def data_parser(self):
        while self.connChk:
            try:
                buffer= self.queue.get(timeout=0.1)
            except Empty:
                continue

            try:
                buffer_np = np.frombuffer(buffer, dtype=np.uint8).reshape([-1, 100])

                if self.channel == 16:
                    azimuth = np.zeros((24 * self.max_len,))
                    azimuth[0::2] = buffer_np[:,2].astype(np.float32) + 256*buffer_np[:,3].astype(np.float32)
                    azimuth[1::2] = buffer_np[:,2].astype(np.float32) + 256*buffer_np[:,3].astype(np.float32) + 20
                else:
                    azimuth = buffer_np[:,2] + 256*buffer_np[:,3]

                dist = (buffer_np[:,4::3].astype(np.float32) + 256*buffer_np[:,5::3].astype(np.float32))*2
                intensity = buffer_np[:,6::3].astype(np.float32)

                # reshape outputs based on 16 channels
                azimuth = azimuth.reshape([-1, 1])/100
                dist = dist.reshape([-1, self.channel])/1000
                intensity = intensity.reshape([-1])

                x, y, z = self.sph2cart(dist, azimuth)
                self.recvChk = True
                
                # Emit signals with point cloud data
                self.pointCloudReady.emit(x, y, z, intensity)
                self.pointCountChanged.emit(len(x))

            except Exception as e :
                error_msg = f'lidar_data : {e}'
                print(error_msg)
                self.connectionError.emit(error_msg)

    def sph2cart(self, R, a):
        x = R * np.cos(np.deg2rad(self.vertical_angle_deg)) * np.sin(np.deg2rad(a))
        y = R * np.cos(np.deg2rad(self.vertical_angle_deg)) * np.cos(np.deg2rad(a))
        z = R * np.sin(np.deg2rad(self.vertical_angle_deg))
        return x.reshape([-1]), y.reshape([-1]), z.reshape([-1])

    # NOTE: get method deprecated for Qt signals
    # def getLidar(self):
    #     with self.lock:
    #         return (self.x, self.y ,self.z, self.intensity)