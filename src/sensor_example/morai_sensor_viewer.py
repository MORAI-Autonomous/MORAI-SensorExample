#!/usr/bin/env python3

import os
import sys
import platform

if platform.system() == 'Linux':
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = ''
    os.environ.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)

    # If running over SSH without display, uncomment this:
    # os.environ['QT_QPA_PLATFORM'] = 'offscreen'

import json
import math
from multiprocessing import freeze_support

import cv2
import numpy as np
import requests
import pyvista as pv
from pyvistaqt import QtInteractor
from CAMprocess import CAMConnector
from form_ui import Ui_main_window
from GPSprocess import GPSConnector
from IMUprocess import IMUConnector
from LIDARprocess import LIDARConnector
from PySide2 import QtCore, QtGui, QtWidgets
from PySide2.QtCore import Qt
from PySide2.QtGui import QBrush, QColor, QImage, QPen, QPixmap
from PySide2.QtWidgets import QVBoxLayout


current_path = os.path.dirname(os.path.realpath(__file__))

class NetworkError(Exception):
    pass


def getTileNum(lat, lon, zn):
    lat_radians = lat/180*math.pi
    n = math.pow(2.0,zn)
    posX = (lon + 180)/360*n
    posY = (1.0 - math.asinh(math.tan(lat_radians))/math.pi)/2*n
    pixX, tileRow = math.modf(posX)
    pixY, tileCol = math.modf(posY)
    pixX = math.floor(pixX*256) + 256
    pixY = math.floor(pixY*256) + 207
    return ([int(tileCol), int(tileRow)], [int(pixX*(400/768)), int(pixY*(400/670))])


def getTile(znVal, rowVal, colVal):
    if znVal < 6:
        znVal = 6
    elif znVal > 19:
        znVal = 19
    znVal = 16

    wmtsAddr = 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/'+ str(znVal) + '/' + str(rowVal) + '/' + str(colVal)

    try:
        result = requests.get(wmtsAddr, timeout=0.5)
        return result
    except Exception:
        return None


class MainWindow(QtWidgets.QDialog):
    def __init__(self):
        super(MainWindow, self).__init__()

        self.ui = Ui_main_window()
        self.ui.setupUi(self)

        resourcePath = os.path.join(current_path, 'resource') + os.sep
        self.config_file = os.path.join(current_path, 'sensor_config.json')

        self.mutex = QtCore.QMutex()
        self.setWindowIcon(QtGui.QIcon(resourcePath+'MORAIicon.png'))

        # Widget 1: Camera view
        self.ui.CamView.setPixmap(QtGui.QPixmap())
        self.ui.CamView.resize(400,400)

        # Widget 2: GPS,IMU Map view
        self.mapScene = QtWidgets.QGraphicsScene(self)
        self.ui.MapView.setScene(self.mapScene)
        self.ui.MapView.show()

        # IMU arrow
        self.odomArrow = QtGui.QPixmap(resourcePath+'pin.png')
        self.odomArrow = self.odomArrow.scaled(17, 25, Qt.IgnoreAspectRatio)

        # Map tiling
        self.buffMapTile = None
        self.buffCenterPose = None
        self.cachedMapPixmap = None
        self.mapEgoColor = QPen(QColor(255,0,0))
        self.gpsLat = 0
        self.gpsLon = 0

        # Widget 3: Lidar 3D pointcloud with PyVista
        # hierarchy: lidar_plotter > lidar_actor > lidar_mesh
        self.lidar_plotter = None
        self.lidar_actor = None
        self.lidar_mesh = None

        # lidar data and status
        self.lidar_connected = False
        self.lidar_point_count = 0
        self.cached_lidar_data = None
        self.new_lidar_data = False
        pv.global_theme.allow_empty_mesh = True

        # Setup layout for LIDAR widget - clear any existing layout
        if self.ui.lidar_frame.layout():
            QtWidgets.QWidget().setLayout(self.ui.lidar_frame.layout())
        
        self.graph_container_layout = QVBoxLayout(self.ui.lidar_frame)
        
        # Initialize Lidar graph with delayed timer after window is shown and stable
        QtCore.QTimer.singleShot(500, self.initLidarGraph)

        self.ui.camera_comboBox.currentIndexChanged.connect(self.updateUi)
        self.ui.gps_comboBox.currentIndexChanged.connect(self.updateUi)
        self.ui.imu_comboBox.currentIndexChanged.connect(self.updateUi)
        self.ui.lidar_comboBox.currentIndexChanged.connect(self.updateUi)
        
        # Load saved configuration
        self.loadConfig()
        
        # Trigger once to set initial state
        self.updateUi()

        self.connected = False
        self.ui.ConnectButton.clicked.connect(self.connect)

    def getNetworkConfig(self):
        self.cameraNetworkType = self.ui.camera_comboBox.currentText()
        self.cameraIp = self.ui.camera_ip_textedit.toPlainText()
        try:
            self.cameraPort = int(self.ui.camera_port_textedit.toPlainText())
        except ValueError:
            self.cameraPort = 0
        self.cameraTopic = self.ui.camera_topic_textedit.toPlainText()

        self.gpsNetworkType = self.ui.gps_comboBox.currentText()
        self.gpsIp = self.ui.gps_ip_textedit.toPlainText()
        try:
            self.gpsPort = int(self.ui.gps_port_textedit.toPlainText())
        except ValueError:
            self.gpsPort = 0
        self.gpsTopic = self.ui.gps_topic_textedit.toPlainText()

        self.imuNetworkType = self.ui.imu_comboBox.currentText()
        self.imuIp = self.ui.imu_ip_textedit.toPlainText()
        try:
            self.imuPort = int(self.ui.imu_port_textedit.toPlainText())
        except ValueError:
            self.imuPort = 0
        self.imuTopic = self.ui.imu_topic_textedit.toPlainText()

        self.lidarNetworkType = self.ui.lidar_comboBox.currentText()
        self.lidarIp = self.ui.lidar_ip_textedit.toPlainText()
        try:
            self.lidarPort = int(self.ui.lidar_port_textedit.toPlainText())
        except ValueError:
            self.lidarPort = 0
        self.lidarTopic = self.ui.lidar_topic_textedit.toPlainText()

    def connect(self):
        try:
            if not self.connected:
                # Save current configuration before connecting
                self.saveConfig()
                
                if self.cameraNetworkType == 'ROS' or self.gpsNetworkType == 'ROS' or self.imuNetworkType =='ROS' or self.lidarNetworkType == 'ROS':
                    try:
                        import rospy
                        # Check if the node is already initialized to prevent crashing on reconnect
                        if not rospy.core.is_initialized():
                            rospy.init_node('morai_sensor_viewer', anonymous=True)
                    except ImportError:
                        QtWidgets.QMessageBox.critical(self, "Error", "ROS (rospy) module not found.")
                        return
                    except rospy.exceptions.ROSException as e:
                        print(f"ROS Initialization warning: {e}")

                # Sensor Connect
                self.cameraManager = CAMConnector(self.cameraNetworkType)
                self.cameraManager.connect(self.cameraIp, self.cameraPort, self.cameraTopic)

                self.gpsManager = GPSConnector(self.gpsNetworkType)
                self.gpsManager.connect(self.gpsIp, self.gpsPort, self.gpsTopic)

                self.imuManager = IMUConnector(self.imuNetworkType)
                self.imuManager.connect(self.imuIp, self.imuPort, self.imuTopic)

                self.lidarManager = LIDARConnector()
                # Move to main thread explicitly to prevent threading issues
                self.lidarManager.moveToThread(QtWidgets.QApplication.instance().thread())
                
                # Connect LIDAR signals
                self.lidarManager.pointCloudReady.connect(self.onLidarPointCloudReady)
                self.lidarManager.connectionStatusChanged.connect(self.onLidarConnectionChanged)
                self.lidarManager.connectionError.connect(self.onLidarError)
                self.lidarManager.pointCountChanged.connect(self.onLidarPointCountChanged)
                self.lidarManager.connect_sensor(self.lidarNetworkType,self.lidarIp,self.lidarPort,self.lidarTopic)

                # Verify that the connection is valid
                if not self.cameraManager.connChk or \
                    not self.gpsManager.connChk or \
                        not self.imuManager.connChk or \
                            not self.lidarManager.connChk :

                    errorMsg = ''
                    tail_formatMsg = ' not Connected'
                    if not self.cameraManager.connChk :
                        errorMsg += 'Camera' + tail_formatMsg + '\n'

                    if not self.gpsManager.connChk :
                        errorMsg += 'Gps' + tail_formatMsg + '\n'

                    if not self.imuManager.connChk :
                        errorMsg += 'Imu' + tail_formatMsg + '\n'

                    if not self.lidarManager.connChk :
                        errorMsg += 'Lidar' + tail_formatMsg + '\n'

                    errorMsg += 'Need to check sensor settings'
                    QtWidgets.QMessageBox.about(self, 'Error', errorMsg)
                    print('Connection Failed')
                    raise NetworkError

                else:
                    self.timer = QtCore.QTimer(self)
                    self.timer.setInterval(100)
                    self.timer.timeout.connect(self.updateScene)
                    self.timer.start()

                    self.connected = True
                    self.ui.ConnectButton.setText('Disconnect')

            else:
                self.connected = False
                self.timer.stop()
                self.ui.ConnectButton.setText('Connect')
                raise NetworkError

        except NetworkError:
            self.cameraManager.disconnect()
            self.gpsManager.disconnect()
            self.imuManager.disconnect()
            self.lidarManager.disconnect_sensor()

            del (self.cameraManager)
            del (self.gpsManager)
            del (self.imuManager)
            del (self.lidarManager)

    def saveConfig(self):
        """Save current UI configuration to JSON file."""
        try:
            config = {
                'camera': {
                    'network_type': self.ui.camera_comboBox.currentText(),
                    'ip': self.ui.camera_ip_textedit.toPlainText(),
                    'port': self.ui.camera_port_textedit.toPlainText(),
                    'topic': self.ui.camera_topic_textedit.toPlainText()
                },
                'gps': {
                    'network_type': self.ui.gps_comboBox.currentText(),
                    'ip': self.ui.gps_ip_textedit.toPlainText(),
                    'port': self.ui.gps_port_textedit.toPlainText(),
                    'topic': self.ui.gps_topic_textedit.toPlainText()
                },
                'imu': {
                    'network_type': self.ui.imu_comboBox.currentText(),
                    'ip': self.ui.imu_ip_textedit.toPlainText(),
                    'port': self.ui.imu_port_textedit.toPlainText(),
                    'topic': self.ui.imu_topic_textedit.toPlainText()
                },
                'lidar': {
                    'network_type': self.ui.lidar_comboBox.currentText(),
                    'ip': self.ui.lidar_ip_textedit.toPlainText(),
                    'port': self.ui.lidar_port_textedit.toPlainText(),
                    'topic': self.ui.lidar_topic_textedit.toPlainText()
                }
            }
            
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=4)
            print(f'Configuration saved to {self.config_file}')
        except Exception as e:
            print(f'Error saving configuration: {e}')

    def loadConfig(self):
        """Load configuration from JSON file and apply to UI."""
        try:
            if not os.path.exists(self.config_file):
                print('No configuration file found, using defaults')
                return
            
            with open(self.config_file, 'r') as f:
                config = json.load(f)
            
            self.applyConfig(config)
            print(f'Configuration loaded from {self.config_file}')
        except Exception as e:
            print(f'Error loading configuration: {e}')

    def applyConfig(self, config):
        """Apply loaded configuration to UI elements."""
        try:
            # Camera configuration
            if 'camera' in config:
                cam = config['camera']
                if 'network_type' in cam:
                    index = self.ui.camera_comboBox.findText(cam['network_type'])
                    if index >= 0:
                        self.ui.camera_comboBox.setCurrentIndex(index)
                if 'ip' in cam:
                    self.ui.camera_ip_textedit.setPlainText(cam['ip'])
                if 'port' in cam:
                    self.ui.camera_port_textedit.setPlainText(cam['port'])
                if 'topic' in cam:
                    self.ui.camera_topic_textedit.setPlainText(cam['topic'])
            
            # GPS configuration
            if 'gps' in config:
                gps = config['gps']
                if 'network_type' in gps:
                    index = self.ui.gps_comboBox.findText(gps['network_type'])
                    if index >= 0:
                        self.ui.gps_comboBox.setCurrentIndex(index)
                if 'ip' in gps:
                    self.ui.gps_ip_textedit.setPlainText(gps['ip'])
                if 'port' in gps:
                    self.ui.gps_port_textedit.setPlainText(gps['port'])
                if 'topic' in gps:
                    self.ui.gps_topic_textedit.setPlainText(gps['topic'])
            
            # IMU configuration
            if 'imu' in config:
                imu = config['imu']
                if 'network_type' in imu:
                    index = self.ui.imu_comboBox.findText(imu['network_type'])
                    if index >= 0:
                        self.ui.imu_comboBox.setCurrentIndex(index)
                if 'ip' in imu:
                    self.ui.imu_ip_textedit.setPlainText(imu['ip'])
                if 'port' in imu:
                    self.ui.imu_port_textedit.setPlainText(imu['port'])
                if 'topic' in imu:
                    self.ui.imu_topic_textedit.setPlainText(imu['topic'])
            
            # LIDAR configuration
            if 'lidar' in config:
                lidar = config['lidar']
                if 'network_type' in lidar:
                    index = self.ui.lidar_comboBox.findText(lidar['network_type'])
                    if index >= 0:
                        self.ui.lidar_comboBox.setCurrentIndex(index)
                if 'ip' in lidar:
                    self.ui.lidar_ip_textedit.setPlainText(lidar['ip'])
                if 'port' in lidar:
                    self.ui.lidar_port_textedit.setPlainText(lidar['port'])
                if 'topic' in lidar:
                    self.ui.lidar_topic_textedit.setPlainText(lidar['topic'])
        except Exception as e:
            print(f'Error applying configuration: {e}')

    def closeEvent(self, event):
        if self.connected:
            try:
                self.timer.stop()
                if hasattr(self, 'cameraManager'):
                    self.cameraManager.disconnect()
                if hasattr(self, 'gpsManager'):
                    self.gpsManager.disconnect()
                if hasattr(self, 'imuManager'):
                    self.imuManager.disconnect()
                if hasattr(self, 'lidarManager'):
                    self.lidarManager.disconnect_sensor()
            except Exception as e:
                print(f'closeEvent cleanup error: {e}')
        super(QtWidgets.QDialog, self).closeEvent(event)

    def setSettingPanel(self,Type,
                            ipLabel, ipText,
                            portLabel, portText,
                            topicLabel, topicText,
                            typeLabel, typeText):
        if Type == 'UDP':
            ipLabel.setEnabled(True)
            ipText.setEnabled(True)
            portLabel.setEnabled(True)
            portText.setEnabled(True)
            topicLabel.setDisabled(True)
            topicText.setDisabled(True)
            typeLabel.setDisabled(True)
            typeText.setDisabled(True)
        else:
            ipLabel.setEnabled(True)
            ipText.setEnabled(True)
            portLabel.setDisabled(True)
            portText.setDisabled(True)
            topicLabel.setEnabled(True)
            topicText.setEnabled(True)
            typeLabel.setEnabled(True)
            typeText.setEnabled(True)

    def updateUi(self):
        self.getNetworkConfig()
        self.setSettingPanel(
            self.cameraNetworkType,
            self.ui.camera_ip_label, self.ui.camera_ip_textedit,
            self.ui.camera_port_label, self.ui.camera_port_textedit,
            self.ui.camera_topic_label, self.ui.camera_topic_textedit,
            self.ui.camera_type_label, self.ui.camera_type_textedit
        )
        self.setSettingPanel(
            self.gpsNetworkType,
            self.ui.gps_ip_label, self.ui.gps_ip_textedit,
            self.ui.gps_port_label, self.ui.gps_port_textedit,
            self.ui.gps_topic_label, self.ui.gps_topic_textedit,
            self.ui.gps_type_label, self.ui.gps_type_textedit
        )
        self.setSettingPanel(
            self.imuNetworkType,
            self.ui.imu_ip_label, self.ui.imu_ip_textedit,
            self.ui.imu_port_label, self.ui.imu_port_textedit,
            self.ui.imu_topic_label, self.ui.imu_topic_textedit,
            self.ui.imu_type_label, self.ui.imu_type_textedit
        )
        self.setSettingPanel(
            self.lidarNetworkType,
            self.ui.lidar_ip_label, self.ui.lidar_ip_textedit,
            self.ui.lidar_port_label, self.ui.lidar_port_textedit,
            self.ui.lidar_topic_label, self.ui.lidar_topic_textedit,
            self.ui.lidar_type_label, self.ui.lidar_type_textedit
        )

    def updateScene(self):
        self.mapScene.clear()

        if self.gpsManager.recvChk:
            vehiclePose = self.updateGps()

            if self.imuManager.recvChk:
                self.updateImu(vehiclePose)
            else:
                # no heading info
                self.mapScene.addEllipse(
                    vehiclePose[0]-5,
                    vehiclePose[1]-5,
                    10,
                    10,
                    self.mapEgoColor,
                    QBrush(self.mapEgoColor.color()))

        if self.cameraManager.recvChk:
            self.updateImg()

        # LIDAR updates are now handled directly via signals

    def updateGps(self):
        self.gpsLon, self.gpsLat = self.gpsManager.getPose()
        zoom_lvl = 16
        totImg = []
        centerPose, vehiclePose = getTileNum(self.gpsLat, self.gpsLon, zoom_lvl)
        if (self.buffCenterPose is None) or (self.buffCenterPose != centerPose):
            totImg = self.getMapBuff(centerPose, zoom_lvl)
            if totImg is None:
                return
            self.buffMapTile = totImg
            self.buffCenterPose = centerPose

        elif self.buffCenterPose == centerPose:
            totImg = self.buffMapTile

        height, width, channel = totImg.shape
        bytesPerLine = 3 * width
        qImg = QImage(totImg.data, width, height, bytesPerLine, QImage.Format_RGB888)
        self.mapScene.addPixmap(QPixmap.fromImage(qImg))
        return vehiclePose

    def getMapBuff(self, centerPose, zoom_level):
        rowTileNum = 3
        colTileNum = 3

        totImg = None
        imgCol = []
        for i in range(rowTileNum):
            for j in range(colTileNum):
                tmpImg = getTile(zoom_level, centerPose[0]+i-1, centerPose[1]+j-1)

                if(tmpImg is None or len(tmpImg.content) < 500):
                    return
                else:
                    imgCol.append(cv2.cvtColor(cv2.imdecode(np.frombuffer(tmpImg.content, dtype=np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB))

            if(totImg is not None):
                totImg = np.vstack([totImg, np.hstack(imgCol)])
            else:
                totImg = np.hstack(imgCol)
            imgCol.clear()

        #resizing
        totImg = cv2.resize(totImg[50:718,:,:], dsize=(400,400))
        return totImg

    def updateImu(self, vehiclePose):
        arrow = self.mapScene.addPixmap(self.odomArrow)
        arrow.setPos(vehiclePose[0]-8.5, vehiclePose[1]-10)
        arrow.setTransformOriginPoint(arrow.boundingRect().center())

        imu_data = self.imuManager.getIMU()
        ori_w = imu_data.orientation_w
        ori_x = imu_data.orientation_x
        ori_y = imu_data.orientation_y
        ori_z = imu_data.orientation_z
        _, _, heading = self.euler_from_quaternion(ori_x, ori_y, ori_z, ori_w)

        arrow.setRotation(-90-(heading*180/math.pi))

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def updateImg(self):
        try:
            camImg = self.cameraManager.getImg()
            qtImg = self.convert_cv_qt(camImg)
            self.ui.CamView.setPixmap(qtImg)
        except Exception as e:
            print(f'updateImg Exception : {e}')
            pass

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(400, 400, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

    def updateLidarDisplay(self):
        """Update the LIDAR 3D visualization with cached data using PyVista."""
        if not self.new_lidar_data:
            return
            
        if self.lidar_plotter is None or self.cached_lidar_data is None:
            return
            
        self.new_lidar_data = False
        
        try:
            x, y, z, intensity = self.cached_lidar_data
            if x is None or len(x) == 0:
                print("Warning: Empty LIDAR data received")
                return
            
            # Convert lists to numpy arrays if needed
            x = np.array(x) if not isinstance(x, np.ndarray) else x
            y = np.array(y) if not isinstance(y, np.ndarray) else y
            z = np.array(z) if not isinstance(z, np.ndarray) else z
            
            # Remove NaN values
            valid_mask = ~(np.isnan(x) | np.isnan(y) | np.isnan(z))
            x = x[valid_mask]
            y = y[valid_mask]
            z = z[valid_mask]
            
            # Downsample for performance (every 5th point)
            step = 5
            x = x[::step]
            y = y[::step]
            z = z[::step]
            
            # Create point cloud
            points = np.column_stack((x, y, -z))  # Note: negating z to match previous orientation
            point_cloud = pv.PolyData(points)
            
            # Add intensity as scalar data if available
            if intensity is not None:
                intensity_array = np.array(intensity) if not isinstance(intensity, np.ndarray) else intensity
                intensity_array = intensity_array[valid_mask][::step]
                if len(intensity_array) == len(points):
                    point_cloud['intensity'] = intensity_array
            
            # Update the existing mesh in-place using shallow_copy
            if self.lidar_mesh is not None:
                self.lidar_mesh.shallow_copy(point_cloud)
            
            self.lidar_plotter.render()
            
        except Exception as e:
            print(f'updateLidarDisplay Exception : {e}')

    def onLidarPointCloudReady(self, x, y, z, intensity):
        """Handle new point cloud data from LIDAR via Qt signal."""
        self.cached_lidar_data = (x, y, z, intensity)
        self.new_lidar_data = True
        self.updateLidarDisplay()

    def onLidarConnectionChanged(self, connected):
        """Handle LIDAR connection status changes."""
        self.lidar_connected = connected
        status = "Connected" if connected else "Disconnected"
        print(f"LIDAR {status}")

    def onLidarError(self, error_msg):
        """Handle LIDAR errors via Qt signal."""
        print(f"LIDAR Error: {error_msg}")

    def onLidarPointCountChanged(self, count):
        """Handle point count updates from LIDAR."""
        self.lidar_point_count = count

    def initLidarGraph(self):
        """Initialize the 3D LIDAR visualization with PyVista."""
        try:
            # Create PyVista plotter widget
            self.lidar_plotter = QtInteractor(self.ui.lidar_frame)
            
            # Add to the layout FIRST before any other configuration
            self.graph_container_layout.addWidget(self.lidar_plotter, 1)
            
            # Set size to fill the entire parent widget
            self.lidar_plotter.resize(400, 400)
            self.lidar_plotter.setMinimumSize(QtCore.QSize(400, 400))
            self.lidar_plotter.setMaximumSize(QtCore.QSize(400, 400))
            
            # Configure the plotter
            self.lidar_plotter.set_background('black')
            self.lidar_plotter.show_axes()
            
            # Set initial camera position
            self.lidar_plotter.camera_position = [
                (0, -150, 50),  # camera position
                (0, 0, 0),      # focal point
                (0, 0, 1)       # view up
            ]

            # Keep a mesh object to update
            self.lidar_mesh = pv.PolyData()
            self.lidar_actor = self.lidar_plotter.add_mesh(
                self.lidar_mesh,
                color='red',
                point_size=3.0,
                render_points_as_spheres=False,
                name='lidar_points',
                reset_camera=False
            )

        except Exception as e:
            print(f"ERROR: LIDAR 3D visualization failed to initialize: {e}")
            # Add placeholder on failure
            placeholder_label = QtWidgets.QLabel("LIDAR 3D View\n(Initialization failed)")
            placeholder_label.setAlignment(QtCore.Qt.AlignCenter)
            placeholder_label.setStyleSheet("background-color: #333; color: #ff6b6b; font-size: 12pt;")
            placeholder_label.setMaximumSize(QtCore.QSize(400, 400))
            self.graph_container_layout.addWidget(placeholder_label)


if __name__ == "__main__":
    freeze_support()
    # Enable OpenGL context sharing (required for QtDataVisualization)
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.ApplicationAttribute.AA_ShareOpenGLContexts)

    app = QtWidgets.QApplication(sys.argv)
    
    # Create and show main window
    main_window = MainWindow()
    main_window.show()
    
    # Run the event loop
    sys.exit(app.exec_())
