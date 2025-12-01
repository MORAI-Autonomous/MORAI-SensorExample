#!/usr/bin/env python3

import math
import os
import sys
from multiprocessing import freeze_support

import cv2
import numpy as np
import requests
from CAMprocess import CAMConnector
from form_ui import Ui_main_window
from GPSprocess import GPSConnector
from IMUprocess import IMUConnector
from LIDARprocess import LIDARConnector
from PySide2 import QtCore, QtGui, QtWidgets
from PySide2.QtCore import Qt
from PySide2.QtDataVisualization import QtDataVisualization
from PySide2.QtGui import QBrush, QColor, QImage, QPen, QPixmap, QVector3D
from PySide2.QtWidgets import QHBoxLayout, QVBoxLayout, QWidget


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

        # Widget 3: Lidar 3D Scatterplot
        self.lidar_graph = None
        self.lidar_container = None
        self.lidar_connected = False
        self.lidar_point_count = 0
        self.cached_lidar_data = None

        hLayout = QHBoxLayout(self.ui.pointcloudWidget)
        vLayout = QVBoxLayout()
        self.graph_container_layout = hLayout
        hLayout.addLayout(vLayout)
        
        # Initialize Lidar graph with delayed timer after window is shown and stable
        QtCore.QTimer.singleShot(500, self.initLidarGraph)

        self.ui.camera_comboBox.currentIndexChanged.connect(self.updateUi)
        self.ui.gps_comboBox.currentIndexChanged.connect(self.updateUi)
        self.ui.imu_comboBox.currentIndexChanged.connect(self.updateUi)
        self.ui.lidar_comboBox.currentIndexChanged.connect(self.updateUi)
        
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
                if self.cameraNetworkType == 'ROS' or self.gpsNetworkType == 'ROS' or self.imuNetworkType =='ROS' or self.lidarNetworkType == 'ROS':
                    try:
                        import rospy
                        rospy.init_node('morai_sensor_viewer',anonymous=True)
                    except ImportError:
                        QtWidgets.QMessageBox.critical(self, "Error", "ROS (rospy) module not found.")
                        return

                #Sensor Connect
                self.cameraManager = CAMConnector(self.cameraNetworkType)
                self.cameraManager.connect(self.cameraIp, self.cameraPort, self.cameraTopic)

                self.gpsManager = GPSConnector(self.gpsNetworkType)
                self.gpsManager.connect(self.gpsIp, self.gpsPort, self.gpsTopic)

                self.imuManager = IMUConnector(self.imuNetworkType)
                self.imuManager.connect(self.imuIp, self.imuPort, self.imuTopic)

                self.lidarManager = LIDARConnector()
                # Connect LIDAR signals
                self.lidarManager.pointCloudReady.connect(self.onLidarPointCloudReady)
                self.lidarManager.connectionStatusChanged.connect(self.onLidarConnectionChanged)
                self.lidarManager.connectionError.connect(self.onLidarError)
                self.lidarManager.pointCountChanged.connect(self.onLidarPointCountChanged)
                self.lidarManager.connect_sensor(self.lidarNetworkType,self.lidarIp,self.lidarPort,self.lidarTopic)

                #Verify that the connection is valid
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
                print('Disconnected')
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
                self.mapScene.addEllipse(vehiclePose[0]-5, vehiclePose[1]-5, 10,10,self.mapEgoColor, \
                    QBrush(self.mapEgoColor.color()))

        if self.cameraManager.recvChk:
            self.updateImg()

        # LIDAR updates are now handled by PySide signals here
        if self.cached_lidar_data is not None:
            self.updateLidarDisplay()

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
        """Update the LIDAR 3D visualization with cached data."""
        if self.lidar_graph is None or self.cached_lidar_data is None:
            return
        try:
            x, y, z, _ = self.cached_lidar_data
            if x is None or len(x) == 0:
                return
            
            # Clear existing data
            proxy = self.lidar_graph.seriesList()[0].dataProxy()
            item_count = proxy.itemCount()
            if item_count > 0:
                proxy.removeItems(0, item_count)

            step = 5
            # Debug: Print data stats
            # print(f"Displaying {len(x)//step} points. Range X: {np.min(x):.2f} to {np.max(x):.2f}")

            dataArray = []
            for i in range(0, len(x), step):
                if np.isnan(x[i]) or np.isnan(y[i]) or np.isnan(z[i]):
                    continue
                
                # Create item and set position explicitly
                itm = QtDataVisualization.QScatterDataItem()
                itm.setPosition(QVector3D(float(x[i]), float(y[i]), float(-z[i])))
                dataArray.append(itm)

            if dataArray:
                proxy.addItems(dataArray)
            else:
                print("Warning: No valid points to display")

        except Exception as e:
            print(f'updateLidarDisplay Exception : {e}')

    def onLidarPointCloudReady(self, x, y, z, intensity):
        """Handle new point cloud data from LIDAR via Qt signal."""
        self.cached_lidar_data = (x, y, z, intensity)
        # The display will be updated in the next timer tick

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
        """Initialize the 3D LIDAR visualization graph."""
        try:
            # Check if the window is actually visible and ready
            if not self.isVisible():
                QtCore.QTimer.singleShot(100, self.initLidarGraph)
                return
                
            self.lidar_graph = QtDataVisualization.Q3DScatter()
            
            # Force the graph to have a valid surface before continuing
            self.lidar_graph.show()
            QtWidgets.QApplication.processEvents()  # Process pending events
            
            self.series = QtDataVisualization.QScatter3DSeries()
            self.lidar_graph.addSeries(self.series)
            self.lidar_graph.setShadowQuality(QtDataVisualization.QAbstract3DGraph.ShadowQualityNone)
            self.lidar_graph.setOrthoProjection(False)
            self.lidar_graph.axisX().setRange(-100,100)
            self.lidar_graph.axisY().setRange(-100,100)
            self.lidar_graph.axisZ().setRange(-30,30)

            self.lidar_graph.scene().activeCamera().setZoomLevel(300)
            self.lidar_graph.scene().activeCamera().setYRotation(-20)
            self.lidar_graph.activeTheme().setGridEnabled(False)
            
            # Use MeshPoint for better performance with point clouds
            self.lidar_graph.seriesList()[0].setMesh(QtDataVisualization.QAbstract3DSeries.MeshPoint)
            self.lidar_graph.seriesList()[0].setItemSize(0.025)
            self.lidar_graph.seriesList()[0].setBaseColor(QColor(255, 0, 0))
            
            # Store container as instance variable to prevent garbage collection
            self.lidar_container = QWidget.createWindowContainer(self.lidar_graph, self)
            self.lidar_container.setMaximumSize(QtCore.QSize(400,400))
            
            # Add to the layout we saved earlier
            self.graph_container_layout.addWidget(self.lidar_container, 1)

        except Exception as e:
            print(f"ERROR: LIDAR 3D visualization failed to initialize: {e}")
            # Add placeholder on failure
            placeholder_label = QtWidgets.QLabel("LIDAR 3D View\n(Initialization failed)")
            placeholder_label.setAlignment(QtCore.Qt.AlignCenter)
            placeholder_label.setStyleSheet("background-color: #333; color: #ff6b6b; font-size: 12pt;")
            placeholder_label.setMaximumSize(QtCore.QSize(400,400))
            self.graph_container_layout.addWidget(placeholder_label, 1)


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
