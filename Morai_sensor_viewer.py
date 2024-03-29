#! /bin/env python
# This Python file uses the following encoding: utf-8
import sys, os
import requests
import math
import time

import numpy as np
import cv2
import PyQt5
import qimage2ndarray

from PyQt5 import QtWidgets, uic, QtGui, QtCore
from PyQt5.QtGui import QColor, QPen, QBrush, QPixmap, QVector3D
from PyQt5.QtCore import Qt, QSize, QThread
from PyQt5.QtWidgets import QHBoxLayout, QVBoxLayout, QWidget 
from PyQt5.QtDataVisualization import (Q3DCamera, Q3DScatter, Q3DTheme,
                                       QAbstract3DGraph, QAbstract3DSeries, QScatter3DSeries,
                                       QScatterDataItem, QScatterDataProxy )

from GPSprocess import GPSConnector
from IMUprocess import IMUConnector
from CAMprocess import CAMConnector
from Lidarprocess import LIDARConnector

from multiprocessing import  freeze_support

class NetworkError(Exception):
    pass

def getTileNum(lat, lon, zn):
    latr = lat/180*math.pi
    n = math.pow(2.0,zn)
    posX = (lon + 180)/360*n
    posY = (1.0 - math.asinh(math.tan(latr))/math.pi)/2*n
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
        result = requests.get(wmtsAddr)
        return result
    except Exception as e:
        return None

class main_window(QtWidgets.QDialog):
    def __init__(self):
        super(main_window, self).__init__()
        
        uic.loadUi('form.ui', self)
        resourcePath = os.path.join(os.path.abspath(os.getcwd()),'resource') + '/'
        
        self.mutex = QtCore.QMutex()
        self.setWindowIcon(QtGui.QIcon(resourcePath+'MORAIicon.png'))        

        #Gps,Imu GraphicsScene
        self.mapScene = QtWidgets.QGraphicsScene(self)
        self.MapView.setScene(self.mapScene)
        self.MapView.show()        

        #imu arrow
        self.odomArrow = QtGui.QPixmap(resourcePath+'pin.png')        
        self.odomArrow = self.odomArrow.scaled(17, 25, Qt.IgnoreAspectRatio) 
        
        #Cam Label View
        self.CamView.setPixmap(QtGui.QPixmap())
        self.CamView.resize(400,400)

        #Lidar 3D Scatter
        self.lidar_graph = Q3DScatter()        
        serises = QScatter3DSeries()                
        self.lidar_graph.addSeries(serises)
        self.lidar_graph.setShadowQuality(QAbstract3DGraph.ShadowQualityNone)        
        self.lidar_graph.axisX().setRange(-50,50)
        self.lidar_graph.axisY().setRange(-50,50)
        self.lidar_graph.axisZ().setRange(-30,30)

        self.lidar_graph.scene().activeCamera().setZoomLevel(300)        
        self.lidar_graph.scene().activeCamera().setYRotation(-20)
        self.lidar_graph.activeTheme().setGridEnabled(False)
        self.lidar_graph.seriesList()[0].setMesh(QAbstract3DSeries.MeshPoint)   
        self.lidar_graph.seriesList()[0].setItemSize(0.025)
        container = QWidget.createWindowContainer(self.lidar_graph)
        container.setMaximumSize(QtCore.QSize(400,400))

        hLayout = QHBoxLayout(self.pointcloudWidget)
        vLayout = QVBoxLayout()
        hLayout.addWidget(container, 1)
        hLayout.addLayout(vLayout)

        #tile var        
        self.buffMapTile = None
        self.buffCenterPose = None
        self.mapEgoColor = QPen(QColor(255,0,0))

        #KR_R_PG_K-City
        self.gpsLat = 37.229319
        self.gpsLon = 126.773287
        
        self.ui_timer = QtCore.QTimer(self)
        self.ui_timer.setInterval(1000)  
        self.ui_timer.timeout.connect(self.updateUi)
        self.ui_timer.start()

        self.Connected = False
        self.ConnectButton.clicked.connect(self.connect)     

    def getNetworkConfig(self):
        self.cameraNetworkType = self.camera_comboBox.currentText()
        self.cameraIp = self.camera_ip_textedit.toPlainText()        
        self.cameraPort = int(self.camera_port_textedit.toPlainText())
        self.cameraTopic = self.camera_topic_textedit.toPlainText()

        self.gpsNetworkType = self.gps_comboBox.currentText()
        self.gpsIp = self.gps_ip_textedit.toPlainText()
        self.gpsPort = int(self.gps_port_textedit.toPlainText())
        self.gpsTopic = self.gps_topic_textedit.toPlainText()

        self.imuNetworkType = self.imu_comboBox.currentText()
        self.imuIp = self.imu_ip_textedit.toPlainText()
        self.imuPort = int(self.imu_port_textedit.toPlainText())
        self.imuTopic = self.imu_topic_textedit.toPlainText()
        
        self.lidarNetworkType = self.lidar_comboBox.currentText()
        self.lidarIp = self.lidar_ip_textedit.toPlainText()
        self.lidarPort = int(self.lidar_port_textedit.toPlainText())
        self.lidarTopic = self.lidar_topic_textedit.toPlainText()
        

    def connect(self):
        try:
            if not self.Connected:                
                # self.getNetworkConfig()                

                if self.cameraNetworkType == 'ROS' or self.gpsNetworkType == 'ROS' or self.imuNetworkType =='ROS' or self.lidarNetworkType == 'ROS':
                    import rospy
                    rospy.init_node('morai_sensor_viewer',anonymous=True)

                #Sensor Connect             
                self.cameraManager = CAMConnector(self.cameraNetworkType)
                self.cameraManager.connect(self.cameraIp, self.cameraPort, self.cameraTopic)

                self.gpsManager = GPSConnector(self.gpsNetworkType)
                self.gpsManager.connect(self.gpsIp, self.gpsPort, self.gpsTopic)

                self.imuManager = IMUConnector(self.imuNetworkType)
                self.imuManager.connect(self.imuIp, self.imuPort, self.imuTopic)

                self.lidarManager = LIDARConnector()
                self.lidarManager.connect(self.lidarNetworkType,self.lidarIp,self.lidarPort,self.lidarTopic)
                
                
                # self.lidarManager.connect(self.lidarIp, self.lidarPort, self.lidarTopic)

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
                    
                    errorMsg += 'Need to check sensor setting'
                    QtWidgets.QMessageBox.about(self, 'Error', errorMsg)
                    print('connect fail')
                    raise NetworkError

                else:
                    
                    self.timer = QtCore.QTimer(self)
                    self.timer.setInterval(100)
                    self.timer.timeout.connect(self.updateScene) 
                    self.timer.start()

                    self.Connected = True
                    self.ConnectButton.setText('Disconnect')

            else:                  
                self.Connected = False      
                self.timer.stop()
                print('disconnect')
                self.ConnectButton.setText('Connect')
                raise NetworkError
                
        except NetworkError:
            self.cameraManager.disconnect()
            self.gpsManager.disconnect()
            self.imuManager.disconnect()
            self.lidarManager.disconnect()
            
            del (self.cameraManager)
            del (self.gpsManager)
            del (self.imuManager)
            del (self.lidarManager)

    def closeEvent(self, event):
        super(QtWidgets.QDialog, self).closeEvent(event)

    def setSettingPannel(self,Type,
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
        self.setSettingPannel(
            self.cameraNetworkType,
            self.camera_ip_label, self.camera_ip_textedit,
            self.camera_port_label, self.camera_port_textedit,
            self.camera_topic_label, self.camera_topic_textedit,
            self.camera_type_label, self.camera_type_textedit
        )
        self.setSettingPannel(
            self.gpsNetworkType,
            self.gps_ip_label, self.gps_ip_textedit,
            self.gps_port_label, self.gps_port_textedit,
            self.gps_topic_label, self.gps_topic_textedit,
            self.gps_type_label, self.gps_type_textedit
        )
        self.setSettingPannel(
            self.imuNetworkType,
            self.imu_ip_label, self.imu_ip_textedit,
            self.imu_port_label, self.imu_port_textedit,
            self.imu_topic_label, self.imu_topic_textedit,
            self.imu_type_label, self.imu_type_textedit
        )
        self.setSettingPannel(
            self.lidarNetworkType,
            self.lidar_ip_label, self.lidar_ip_textedit,
            self.lidar_port_label, self.lidar_port_textedit,
            self.lidar_topic_label, self.lidar_topic_textedit,
            self.lidar_type_label, self.lidar_type_textedit
        )
 
    def updateScene(self):
        # self.mutex.lock()
        if self.gpsManager.recvChk:
            vehiclePose = self.updateGps()
            
            if self.imuManager.recvChk:                
                self.updateImu(vehiclePose)               
            else:                
                self.mapScene.addEllipse(vehiclePose[0]-5, vehiclePose[1]-5, 10,10,self.mapEgoColor, \
                    QBrush(self.mapEgoColor.color()))
        
        if self.cameraManager.recvChk:        
            self.updateImg()        

        if self.lidarManager.recvChk:
            self.updateLidar()
        
        # self.mutex.unlock()

    def updateGps(self):
        self.gpsLon, self.gpsLat = self.gpsManager.getPose()
        zoomLvl = 16            
        totImg = []
        centerPose, vehiclePose = getTileNum(self.gpsLat, self.gpsLon, zoomLvl)
        if (self.buffCenterPose is None) or (self.buffCenterPose != centerPose):                
            totImg = self.getMapBuff(centerPose, zoomLvl)
            if totImg is None:
                return
            self.buffMapTile = totImg
            self.buffCenterPose = centerPose

        elif self.buffCenterPose == centerPose:
            totImg = self.buffMapTile

        self.mapScene.addPixmap(PyQt5.QtGui.QPixmap.fromImage(qimage2ndarray.array2qimage(totImg)))        
        return vehiclePose
        
    def getMapBuff(self,centerPose, zoomLvl):
        rowTileNum = 3
        colTileNum = 3
       
        totImg = None
        imgCol = []
        for i in range(rowTileNum):
            for j in range(colTileNum):
                tmpImg = getTile(zoomLvl, centerPose[0]+i-1, centerPose[1]+j-1)

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
            camImg = cv2.resize(camImg,dsize=(400,400))
            qtImg = self.convert_cv_qt(camImg)
            self.CamView.setPixmap(qtImg)
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

    def updateLidar(self):
        try:
            x,y,z,_ = self.lidarManager.getLidar()
            dataArray = []
            for i in range(0,len(x),5):            
                itm = QScatterDataItem(QVector3D(x[i],y[i],-z[i]))
                dataArray.append(itm)

            self.lidar_graph.seriesList()[0].dataProxy().resetArray(dataArray)

        except Exception as e:
            print(f'updateLidar Exception : {e}')


    
if __name__ == "__main__":

    freeze_support()    
    app = QtWidgets.QApplication(sys.argv)
    screen = main_window()
    screen.show()
    app.exec_()
    sys.exit()
    