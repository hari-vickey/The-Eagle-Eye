#!/usr/bin/env python3
"""
ROS Node - pyqt5.py - HMI - Aruco Detection
This Node will detect the aruco markers of the bot then publishes them 
to other ROS Nodes using ROS publisher.
This Node uses rospy_message_converter to convert dictionary to 
ROS Messages.
Also, this node also capable of 
ROS Publisher  - /bot_position
ROS Subscriber - /image_raw
"""
# Importing Required Modules
import os
import sys
import cv2
import time
import math
import json
import rospy
import rospkg
import datetime
import threading
import numpy as np
import pandas as pd
from PyQt5.QtGui import *
import cv2.aruco as aruco
from PyQt5.QtCore import *
from helper import function
from PyQt5.QtWidgets import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from PyQt5 import QtGui, QtWidgets
from PyQt5.QtCore import QTimer,QDateTime
from cv_bridge import CvBridge, CvBridgeError
from rospy_message_converter import message_converter

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):

        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1920, 1080)
        MainWindow.setTabShape(QTabWidget.Rounded)

        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        self.flag = False
        rp = rospkg.RosPack()
        self.str_pkg_path = rp.get_path('swarmrobot')
        # For adding background wallpaper for the main window 
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(0, 110, 1920, 1200))
        self.label.setPixmap(QPixmap("{}/images/mainwindow.png".format(self.str_pkg_path)))

        # For adding table widget for excel data
        self.table = QTableWidget(self.centralwidget)
        self.table.setObjectName(u"table")
        self.table.setGeometry(QRect(920, 450, 400, 521))

        self.table2 = QTableWidget(self.centralwidget)
        self.table2.setObjectName(u"table")
        self.table2.setGeometry(QRect(1440, 450, 400, 521))

        # For creating the Titlebar in the interface
        self.textBrowser = QTextBrowser(self.centralwidget)
        self.textBrowser.setGeometry(QRect(0, 0, 1920, 110))
        self.textBrowser.setObjectName("textBrowser")
        self.textBrowser.setStyleSheet("background-color: #EECEEC")

        # For creating the table
        self.font = QFont("Times", 12)
        self.font.setBold(True)
        self.font.setWeight(75)
        self.font.setKerning(True)
        self.Processing = QTabWidget(self.centralwidget)
        self.Processing.setObjectName(u"Processing")
        self.Processing.setEnabled(True)
        self.Processing.setGeometry(QRect(920, 150, 921, 250))
        # self.Processing.setMaximumSize(QSize(921, 16777215))
        self.Processing.setFont(self.font)
        self.Processing.setAutoFillBackground(True)
        self.Processing.setTabShape(QTabWidget.Triangular)

        # For the tab "Processing"
        self.tab = QWidget()
        self.tab.setObjectName("tab")
        self.tableWidget_2 = QTableWidget(self.tab)
        self.tableWidget_2.setGeometry(QRect(0, 0, 911, 221))
        self.tableWidget_2.setObjectName("tableWidget_2")
        
        self.tableWidget_2.setColumnCount(6)
        self.tableWidget_2.setRowCount(2)

        item = QTableWidgetItem()
        self.tableWidget_2.setVerticalHeaderItem(0, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setVerticalHeaderItem(1, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setHorizontalHeaderItem(0, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setHorizontalHeaderItem(1, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setHorizontalHeaderItem(2, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setHorizontalHeaderItem(3, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setHorizontalHeaderItem(4, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setHorizontalHeaderItem(5, item)
        self.tableWidget_2.verticalHeader().setDefaultSectionSize(90)
        self.tableWidget_2.verticalHeader().setHighlightSections(True)
        self.tableWidget_2.horizontalHeader().setDefaultSectionSize(149)
        self.tableWidget_2.horizontalHeader().setMinimumSectionSize(138)
        self.Processing.addTab(self.tab, "")

        # For the tab "Delivered"
        self.tab_5 = QWidget()
        self.tab_5.setObjectName("tab_5")
        self.tableWidget_3 = QTableWidget(self.tab_5)
        self.tableWidget_3.setGeometry(QRect(0, 0, 911, 221))
        self.tableWidget_3.setObjectName("tableWidget_3")
        
        self.tableWidget_3.setColumnCount(5)
        self.tableWidget_3.setRowCount(10)
        
        item = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(0, item)
        item = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(1, item)
        item = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(2, item)
        item = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(3, item)
        item = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(4, item)
        
        item = QTableWidgetItem()
        self.tableWidget_3.setHorizontalHeaderItem(0, item)
        item = QTableWidgetItem()
        self.tableWidget_3.setHorizontalHeaderItem(1, item)
        item = QTableWidgetItem()
        self.tableWidget_3.setHorizontalHeaderItem(2, item)
        item = QTableWidgetItem()
        self.tableWidget_3.setHorizontalHeaderItem(3, item)
        item = QTableWidgetItem()
        self.tableWidget_3.setHorizontalHeaderItem(4, item)
        item = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(5, item)
        item = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(6, item)
        item = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(7, item)
        item = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(8, item)
        item = QTableWidgetItem()
        self.tableWidget_3.setVerticalHeaderItem(9, item)
        
        self.tableWidget_3.horizontalHeader().setDefaultSectionSize(175)
        self.tableWidget_3.horizontalHeader().setMinimumSectionSize(120)
        self.Processing.addTab(self.tab_5,"")

        # For Creating frame for video streaming 
        self.frame = QLabel(self.centralwidget)
        self.frame.setGeometry(QRect(80, 160, 800, 600))
        pixmap = QPixmap("{}/images/eagle_eye.jpg".format(self.str_pkg_path))
        P = pixmap.scaled(800, 600, Qt.KeepAspectRatio)
        self.frame.setPixmap(P)
        self.frame.setAutoFillBackground(True)
        self.frame.setFrameShape(QFrame.Box)
        self.frame.setFrameShadow(QFrame.Raised)
        self.frame.setLineWidth(2)
        self.frame.setObjectName("frame")

        # For displaying "Run time " content
        self.textEdit = QTextEdit(self.centralwidget)
        self.textEdit.setObjectName(u"textEdit")
        self.textEdit.setGeometry(QRect(400, 870, 200, 87))

        # For showing the text "Run time" 
        self.label_2 = QLabel(self.centralwidget)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(250, 870, 200, 90))
   
        # For showing the name of the excel data
        self.data1 = QLabel(self.centralwidget)
        self.data1.setObjectName(u"data1")
        self.data1.setGeometry(QRect(1000, 380, 210, 90))

        self.data2 = QLabel(self.centralwidget)
        self.data2.setObjectName(u"data2")
        self.data2.setGeometry(QRect(1530, 380, 210, 90))
   
        # For adding flipkart logo in the titlebar
        self.flipkart_logo = QLabel(self.centralwidget)
        self.flipkart_logo.setObjectName(u"flipkart_logo")
        self.flipkart_logo.setGeometry(QRect(1680, -50, 220, 220))
        logo = QPixmap("{}/images/Flipkart_logo.png".format(self.str_pkg_path))
        flipkart_pixmap = logo.scaled(220, 220, Qt.KeepAspectRatio, Qt.FastTransformation)
        self.flipkart_logo.setPixmap(flipkart_pixmap)

        #For adding eagleeye logo in the titlebar
        self.eagleeye_logo = QLabel(self.centralwidget)
        self.eagleeye_logo.setObjectName(u"eagleeye_logo")
        self.eagleeye_logo.setGeometry(QRect(0, -55, 220, 220))
        eagle = QPixmap("{}/images/eagle_eye.jpg".format(self.str_pkg_path))
        eagleeye_pixmap = eagle.scaled(170, 180, Qt.KeepAspectRatio, Qt.FastTransformation)
        self.eagleeye_logo.setPixmap(eagleeye_pixmap)

        # For Creating Start PushButton 
        self.pushButton = QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QRect(310, 800, 141, 51))
        self.pushButton.setObjectName("pushButton")
        self.pushButton.setStyleSheet("border:2px; background-color: #00FF00; border-radius : 20px")

        # For Creating Stop PushButton 
        self.pushButton_2 = QPushButton(self.centralwidget)
        self.pushButton_2.setGeometry(QRect(510, 800,141, 51))
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_2.setStyleSheet("border:2px; background-color: red; border-radius : 20px")

        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        # self.Processing.setCurrentIndex(2)
        QMetaObject.connectSlotsByName(MainWindow)


    def retranslateUi(self, MainWindow):
        _translate = QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))

        self.textBrowser.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
        "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
        "p, li { white-space: pre-wrap; }\n"
        "</style></head><body style=\" font-family:\'MS Shell Dlg 2\'; font-size:7.8pt; font-weight:400; font-style:normal;\">\n"
        "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:18pt; color:black;\"><b>Flipkart Grid 3.0</b></span></p>\n"
        "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:14pt; color:black;\"><b>The Eagle Eye</b></span></p>\n"
        "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:14pt; color:black;\"><b>Central Monitoring System</span></b></p></body></html>"))

        # For the Tab "Processing" in the table
        item = self.tableWidget_2.verticalHeaderItem(0)
        item.setText(_translate("MainWindow", "1"))
        item = self.tableWidget_2.verticalHeaderItem(1)
        item.setText(_translate("MainWindow", "2"))
        
        item = self.tableWidget_2.horizontalHeaderItem(0)
        item.setText(_translate("MainWindow", "Package ID"))
        item = self.tableWidget_2.horizontalHeaderItem(1)
        item.setText(_translate("MainWindow", "Induct Station"))
        item = self.tableWidget_2.horizontalHeaderItem(2)
        item.setText(_translate("MainWindow", "City"))
        item = self.tableWidget_2.horizontalHeaderItem(3)
        item.setText(_translate("MainWindow", "Bot ID"))
        item = self.tableWidget_2.horizontalHeaderItem(4)
        item.setText(_translate("MainWindow", "Dispatch"))
        item = self.tableWidget_2.horizontalHeaderItem(5)
        item.setText(_translate("MainWindow", "Shipment"))
        
        __sortingEnabled = self.tableWidget_2.isSortingEnabled()
        self.tableWidget_2.setSortingEnabled(False)
        self.tableWidget_2.setSortingEnabled(__sortingEnabled)
        self.Processing.setTabText(self.Processing.indexOf(self.tab), _translate("MainWindow", "Processing"))


        # For the Tab "Delivered" in the table
        item = self.tableWidget_3.verticalHeaderItem(0)
        item.setText(_translate("MainWindow", "1"))
        item = self.tableWidget_3.verticalHeaderItem(1)
        item.setText(_translate("MainWindow", "2"))
        item = self.tableWidget_3.verticalHeaderItem(2)
        item.setText(_translate("MainWindow", "3"))
        item = self.tableWidget_3.verticalHeaderItem(3)
        item.setText(_translate("MainWindow", "4"))
        item = self.tableWidget_3.verticalHeaderItem(4)
        item.setText(_translate("MainWindow", "5"))
        item = self.tableWidget_3.verticalHeaderItem(5)
        item.setText(_translate("MainWindow", "6"))
        item = self.tableWidget_3.verticalHeaderItem(6)
        item.setText(_translate("MainWindow", "7"))
        item = self.tableWidget_3.verticalHeaderItem(7)
        item.setText(_translate("MainWindow", "8"))
        item = self.tableWidget_3.verticalHeaderItem(8)
        item.setText(_translate("MainWindow", "9"))
        item = self.tableWidget_3.verticalHeaderItem(9)
        item.setText(_translate("MainWindow", "10"))
        
        item = self.tableWidget_3.horizontalHeaderItem(0)
        item.setText(_translate("MainWindow", "Package ID"))
        item = self.tableWidget_3.horizontalHeaderItem(1)
        item.setText(_translate("MainWindow", "Induct Station"))
        item = self.tableWidget_3.horizontalHeaderItem(2)
        item.setText(_translate("MainWindow", "City"))
        item = self.tableWidget_3.horizontalHeaderItem(3)
        item.setText(_translate("MainWindow", "Bot ID"))
        item = self.tableWidget_3.horizontalHeaderItem(4)
        item.setText(_translate("MainWindow", "Time Taken(s)"))
        
        __sortingEnabled = self.tableWidget_3.isSortingEnabled()
        self.tableWidget_3.setSortingEnabled(False) 
        self.tableWidget_3.setSortingEnabled(__sortingEnabled)
        self.Processing.setTabText(self.Processing.indexOf(self.tab_5), _translate("MainWindow", "Delivered"))
        
        # For writing text in the start push button and enabling the video 
        # self.flag = True
        # self.Worker1 = Worker1()
        self.pushButton.setText(_translate("MainWindow", "START"))
        self.pushButton.setFont(QFont('Times', 15))
        self.pushButton.clicked.connect(self.StartFeed)

        # For loading excel in the table
        excel = "{}/sheet/Book1.xlsx".format(self.str_pkg_path)
        excel2 = "{}/sheet/Book2.xlsx".format(self.str_pkg_path)
        worksheet = 'Sheet1'
        self.pushButton.clicked.connect(lambda _, xlxs_path1=excel, xlxs_path2 = excel2, sheet_name=worksheet: self.loading_Excel_Data(xlxs_path1, xlxs_path2, sheet_name))

        # For writing text in the stop push button and enabling the video 
        self.pushButton_2.setText(_translate("MainWindow", "STOP"))
        self.pushButton_2.setFont(QFont('Times', 15))
        self.pushButton_2.clicked.connect(self.StopFeed)
        # self.toolBar.setWindowTitle(_translate("MainWindow", "toolBar"))

        # For writing "Run time" text in label_2 
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Run time", None))
        self.label_2.setFont(QFont('Times', 20))

        # For giving the excel sheet table widget name
        self.data1.setText(QCoreApplication.translate("MainWindow", u"Sample_Data1.xlxs", None))
        self.data1.setFont(QFont('Times', 15))

        self.data2.setText(QCoreApplication.translate("MainWindow", u"Sample_Data2.xlxs", None))
        self.data2.setFont(QFont('Times', 15))

        # for timer 
        self.timer1 = QTimer()
        self.start = time.time()
        self.s = 0
        self.label.setText("")

    def loading_Excel_Data(self, excel1, excel2, worksheet):
        df1 = pd.read_excel(excel1, worksheet)
        df2 = pd.read_excel(excel2, worksheet)
        if df1.size == 0:
            return

        df1.fillna('', inplace=True)
        self.table.setRowCount(df1.shape[0])
        self.table.setColumnCount(df1.shape[1])
        self.table.setHorizontalHeaderLabels(df1.columns)

        # returns pandas array object
        for row in df1.iterrows():
            values = row[1]
            for col_index, value in enumerate(values):
                if isinstance(value, (float, int)):
                    value = '{0:0,.0f}'.format(value)
                tableItem = QTableWidgetItem(str(value))
                self.table.setItem(row[0], col_index, tableItem)

        self.table.setColumnWidth(2, 300)

        if df2.size == 0:
            return

        df2.fillna('', inplace=True)
        self.table2.setRowCount(df2.shape[0])
        self.table2.setColumnCount(df2.shape[1])
        self.table2.setHorizontalHeaderLabels(df2.columns)

        # returns pandas array object
        for row in df2.iterrows():
            values = row[1]
            for col_index, value in enumerate(values):
                if isinstance(value, (float, int)):
                    value = '{0:0,.0f}'.format(value)
                tableItem = QTableWidgetItem(str(value))
                self.table2.setItem(row[0], col_index, tableItem)

        self.table2.setColumnWidth(2, 300)

  
    def ImageUpdateSlot(self, image):
        """ Function for updating the image in the frame """
        self.frame.setPixmap(QPixmap.fromImage(image))

    def StartFeed(self):
        """ Function use to start the video and timer """
        self.flag = True
        # os.system("rosrun swarmrobot client.py")
        self.timer1.timeout.connect(self.showTime)
        self.timer1.start(1000)

    def StopFeed(self):
        """ Function use to stop the video and timer """
        self.flag = False
        image = ("{}/images/black.jpg".format(self.str_pkg_path))
        Imagelogo = cv2.imread(image)
        ConvertToQtFormat = QImage(Imagelogo.data, Imagelogo.shape[1], Imagelogo.shape[0], QImage.Format_RGB888)
        Pic = ConvertToQtFormat.scaled(1200, 600, Qt.KeepAspectRatio)
        self.ImageUpdateSlot(Pic)
        rospy.sleep(1)
        exit()        
 
    def showTime(self):
        """ method called by timer """
        if self.flag == True: 
            sec = int(time.time()-self.start)
            fin = str(sec)
            fi_s = str(0)
            if sec%60 == 0:
                self.s = self.s + 1
                self.start = self.start + 60
            fi_s = str(self.s)
            # showing text
            self.textEdit.setText(QCoreApplication.translate("MainWindow", fi_s + ":" + fin, None))
            self.textEdit.setFont(QFont('Times', 20))

# Class Detect
class Detect():
    """
    This class will detect the aruco markers of the bot
    and publish them using ros publihsers to other nodes
    """

    # Constructor
    # Initializing the variables of this class
    def __init__(self):

        # Creating Objects for PyQt5 Application
        app = QApplication(sys.argv)
        MainWindow = QMainWindow()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(MainWindow)

        # Creating a object to CvBridge() class
        self.bridge = CvBridge()

        # Creating Variables to store necessary values
        self.location = ['Mumbai', 'Delhi', 'Kolkata', 
                         'Chennai', 'Bengaluru', 'Hyderabad', 
                         'Pune', 'Ahmedabad', 'Jaipur']

        # Reading Current System Time
        time = datetime.datetime.now()
        # Global Varibales for Aruco Marker Detections
        self.deg2, self.deg4 = 0, 0
        self.time_2, self.time_4 = time, time
        self.index, self.graphc, self.completed = 0, 0, 0
        self.aruco1, self.ind1, self.aruco2, self.ind2 = 0, 0, 0, 0
        self.temp1, self.temp2, self.temp3, self.temp4 = [], [], [], []
        self.dict, self.destination, self.inductzone = {}, {}, {}

        # Publishing Bot Positions
        self.publisher = rospy.Publisher("/bot_position", String, 
                                         queue_size=1)

        # Subscribing to the ROS Image topic
        rospy.Subscriber("/image_raw", Image, 
                                          self.callback, queue_size = 1)

        # Subscribing to Data Visualizer
        rospy.Subscriber("/data_visualize", String, self.viz_callback)

        # Creating msg variable of String Datatype
        self.msg = String()

        # UI Display
        MainWindow.show()
        sys.exit(app.exec_())

    # Function for ROS Camera Subscription Callback
    def callback(self, data):
        """
        This functions gets all the image published in the subscribed topic and
        sends the image to the next function to process them
        """
        try:
            # Convert img msg to an data readable by cv2 library,
            # to process them further
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv_image[0:720, 115:1120]

            # If Aruco markers of inductzone and destination detected
            # Also, if the graph is plotted then bot detection starts
            if self.completed == 1 and self.ui.flag == True:
                # Detect Bots using Aruco Markers
                bot = self.aruco_detect_bot(cv_image)
                img = self.mark_points(cv_image, bot)
                img = cv2.resize(img, (800, 600))
                thread = threading.Thread(name="worker", target=self.qt_convert, args=(img, ) )
                thread.start()
                # frame = self.qt_convert(img)
                self.ui.ImageUpdateSlot(self.frame)

            elif self.completed == 2:
                function.write_graph()
                self.destination.update(self.inductzone)
                function.write_location(self.destination)

                print('\033[93m' + "Induct Zone Aruco Markers Detected")
                print(self.inductzone)
                print(".....")
                print('\033[92m' + "Completed Graph Plotting")
                print(".....")
                print('\033[94m' + "Destination Markers Detected")
                print(self.destination)
                print("....." + '\033[0m')
                self.completed = 1

            # Execute the arena_config function only one time
            elif self.graphc == 0:
                self.arena_config(cv_image)

            # Execute the aruco_detection_destination untill it detects,
            # all the destination points
            elif self.aruco1 == 0:
                self.aruco_detect_inductpoint(cv_image)

            # Execute the aruco_detection_inductstation untill it detects,
            # all the inductstation points
            elif self.aruco2 == 0:
                self.aruco_detect_destination(cv_image)

        except Exception as e:
            print(e)
            # pass

    # Function to detect aruco markers Bot
    def aruco_detect_bot(self, frame):
        """
        This function will detect the aruco markers of the bot
        using dictionary of size 4x4 50
        """
        try:
            # Creating Detector Parameters
            parameters =  cv2.aruco.DetectorParameters_create()
            # Detect the markers in the image
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
            frame = aruco.drawDetectedMarkers(frame, markerCorners)
            converted = np.int_(markerCorners)
            bot_name, points, cpts = [], [], []
            l = 0

            for i in markerIds:
                name = 'bot' + str(int(i))
                bot_name.append(name)
                pts = [converted[l][0][2].tolist(), 
                       converted[l][0][0].tolist(),
                       converted[l][0][1].tolist(),
                       converted[l][0][3].tolist()]
                points.append(pts)
                l += 1

            bot = dict(zip(bot_name, points))

            for i in bot:
                try:
                    deg = self.bot_angle(bot[i][2], bot[i][0], bot[i][1], bot[i][3], i)
                except ZeroDivisionError:
                    deg = 0

                x1 = int((bot[i][1][0]+bot[i][0][0])/2)
                y1 = int((bot[i][1][1]+bot[i][0][1])/2)
                point = (x1, y1, int(deg))
                cpts.append(point)

            bot = dict(zip(bot_name, cpts))
            self.msg = json.dumps(bot)
            self.publisher.publish(self.msg)
            print(bot)

            return bot

        except Exception as e:
            print(e)
            # pass

    # Function to Angle of the Bot
    def bot_angle(self, pt2, pt0, pt1, pt3, i):
        """
        This function is to calculate the angle of the bot
        using the third and four point of the aruco marker
        """
        x = abs(pt3[0] - pt0[0])
        y = abs(pt3[1] - pt0[1])
        p = abs(pt0[0] - pt1[0])
        q = abs(pt0[1] - pt1[1])

        if i == "bot2":
            d = self.deg2
        else:
            d = self.deg4

        if pt0[1] < pt3[1] and pt0[0] > pt3[0] and pt0[0] > pt1[0]:
            rad = math.atan(x/y)
            d = rad *(180/(math.pi))
        elif pt0[1] > pt3[1] and pt0[0] > pt3[0] and pt0[1] > pt1[1] and pt0[0] > pt1[0]:
            rad = math.atan(q/p)
            d = rad *(180/(math.pi))
            d = d + 45
        elif pt0[1] > pt3[1] and pt0[0] > pt3[0] and pt0[1] > pt1[1] and pt0[0] < pt1[0] :
            rad = math.atan(p/q)
            d = rad *(180/(math.pi))
            d = d + 130    
            # Left Turn
        elif pt0[1] < pt3[1] and pt0[0] < pt3[0] and pt0[1] < pt1[1]:
            rad = math.atan(x/y)
            d = -(rad *(180/(math.pi)))
        elif pt0[0] < pt3[0] and pt0[0] < pt1[0] and pt0[1] > pt3[1] and pt0[1] < pt1[1]:
            rad = math.atan(p/q)
            d = rad *(180/(math.pi))
            d = -(d + 45)   
        elif pt0[0] < pt3[0] and pt0[0] < pt1[0] and pt0[1] > pt3[1] and pt0[1] > pt1[1]:
            rad = math.atan(q/p)
            d = rad *(180/(math.pi))
            d = -(d + 130)
        elif pt0[1] == pt3[1]:
            d = 90
        elif pt0[0] == pt3[0] and pt0[1] == pt1[1]:
            d = 180
        elif pt0[0] == pt1[0]:
            d = 90

        if i == "bot2":
            self.deg2 = d
        else:
            self.deg4 = d

        return d

    # Function to have spatial awareness of the arena
    def arena_config(self, image):
        """
        This function defines the bot radius and calls the next function to
        create the arena as graph and generates the points of the obstacle
        """
        radius, clearance = 0, 0
        height, width, _ = image.shape
        function.init_graph(height, width)
        self.graphc = 1

    # Function to detect aruco Markers of Induct Station Points
    def aruco_detect_inductpoint(self, frame):
        try:
            """
            This Function will detect inductstation aruco markers
            """
            self.ind1 = 0
            parameters =  cv2.aruco.DetectorParameters_create()
            # Detect the Induct Point markers in the image
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
            markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

            for i in markerIds:
                try:
                    self.temp1.index(i)
                    self.ind1 += 1

                except ValueError:
                    temp = int(i)
                    self.temp1.append(temp)
                    self.temp2.append(markerCorners[self.ind1].tolist())
                    self.ind1 += 1

            if len(self.temp1) == 2:
                self.inductzone = self.extract_induct_point(self.temp1, self.temp2)
                self.aruco1 = 1

        except Exception as e:
            print(e)
            # pass

    # Function to Extract Induct Points
    def extract_induct_point(self, ids, bbox):
        """
        Extract the induct points from the list and convert
        it to dictionary and returns it.
        """
        l_ind = 0
        temp_ls, dest_ls = [], []

        for i in ids:
            name = int(i)
            cen_x = int((bbox[l_ind][0][0][0] + bbox[l_ind][0][2][0])/2)
            cen_y = int((bbox[l_ind][0][0][1] + bbox[l_ind][0][2][1])/2)
            ctp = (cen_x, cen_y)
            temp_ls.append(ctp)
            dest_ls.append(name)
            l_ind += 1

        dest = dict(zip(dest_ls, temp_ls))

        return dest

    # Function to detect aruco Markers of Destination Points
    def aruco_detect_destination(self, frame):
        """
        This Function is responsible to detect all the destination markers
        to identify all possible location around the chute.
        """
        try:
            self.ind2 = 0
            parameters =  cv2.aruco.DetectorParameters_create()
            # Detect the markers in the image
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
            markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

            for i in markerIds:
                indices = [j for j, x in enumerate(self.temp1) if x == i]
                if len(indices) == 1:
                    temp = int(i)
                    self.temp3.append(temp)
                    self.temp4.append(markerCorners[self.ind2])
                elif len(indices) == 0:
                    temp = int(i)
                    self.temp3.append(temp)
                    self.temp4.append(markerCorners[self.ind2])
                self.ind2 += 1

            if len(self.temp3) == 18:
                self.destination = self.extract_goal_point(self.temp3, self.temp4)
                self.aruco2, self.completed = 1, 2

        except Exception as e:
            print(e)
            pass

    # Function to Extract Goal Points
    def extract_goal_point(self, ids, bbox):
        """
        This function will extract the co-ords of the goal point and
        returns a dictionary
        """
        ind = 0
        ids_ls, cor_ls = [], []
        temp_ls, sum_ls, pts_ls = [], [], []

        for i in ids:
            ids_ls.append(ids[ind])
            t = bbox[ind].tolist()
            cor_ls.append(t[0])
            ind += 1

        for k in range(1, 10):
            indices = [i for i, x in enumerate(ids_ls) if x == k]
            for l in indices:
                sum_ls = sum_ls + cor_ls[l]
            temp_ls.append(sum_ls)
            sum_ls = []

        for i in range(0,9):
            x1 = int((temp_ls[i][0][0] + temp_ls[i][2][0])/2)
            y1 = int((temp_ls[i][0][1] + temp_ls[i][2][1])/2)
            x2 = int((temp_ls[i][4][0] + temp_ls[i][6][0])/2)
            y2 = int((temp_ls[i][4][1] + temp_ls[i][6][1])/2)

            if (y1<y2):
                x1, x2 = x2, x1
                y1, y2 = y2, y1

            d1 = (x1, y1)
            d2 = (x2, y2)
            i1 = (x1, y2)
            i2 = (x2, y1)

            function.render_graph((x1+25, y1-25), (x2-25, y2+25))
            pts = [d1, d2, i1, i2]
            pts_ls.append(pts)

        dest = dict(zip(self.location, pts_ls))

        return dest

    # Function for Data Visualization Callback
    def viz_callback(self, data):
        """
        This Function gets all the published data visualization messages
        and process them to show the path
        """
        msg = message_converter.convert_ros_message_to_dictionary(data)
        temp = msg['data']
        bot = json.loads(temp)
        thread = threading.Thread(name="worker", target=self.UpdateTable, args=(bot, ))
        thread.start()

    # Function to Update Table
    def UpdateTable(self, bot):
        """
        This Function is update table simultaneously
        """
        if self.index == 10:
            self.index = 0
        for i in bot:
            if i == "bot2":
                self.dict[i] = bot[i]
                self.ui.tableWidget_2.setItem(0, 0, QTableWidgetItem(bot[i][5]))
                self.ui.tableWidget_2.setItem(0, 1, QTableWidgetItem(bot[i][3]))
                self.ui.tableWidget_2.setItem(0, 2, QTableWidgetItem(bot[i][4]))
                self.ui.tableWidget_2.setItem(0, 3, QTableWidgetItem("2"))
                self.ui.tableWidget_2.setItem(0, 4, QTableWidgetItem(bot[i][6]))
                self.ui.tableWidget_2.setItem(0, 5, QTableWidgetItem(bot[i][7]))
                if bot[i][7] == "No":
                    self.time_2 = datetime.datetime.now()
                elif bot[i][7] == "Yes":
                    t2 = datetime.datetime.now()
                    temp2 = t2 - self.time_2
                    time2 = str(temp2.seconds)
                    self.ui.tableWidget_3.setItem(self.index, 0, QTableWidgetItem(bot[i][5]))
                    self.ui.tableWidget_3.setItem(self.index, 1, QTableWidgetItem(bot[i][3]))
                    self.ui.tableWidget_3.setItem(self.index, 2, QTableWidgetItem(bot[i][4]))
                    self.ui.tableWidget_3.setItem(self.index, 3, QTableWidgetItem("2"))
                    self.ui.tableWidget_3.setItem(self.index, 4, QTableWidgetItem(time2))
                    self.index += 1

            if i == "bot4":
                self.dict[i] = bot[i]
                self.ui.tableWidget_2.setItem(1, 0, QTableWidgetItem(bot[i][5]))
                self.ui.tableWidget_2.setItem(1, 1, QTableWidgetItem(bot[i][3]))
                self.ui.tableWidget_2.setItem(1, 2, QTableWidgetItem(bot[i][4]))
                self.ui.tableWidget_2.setItem(1, 3, QTableWidgetItem("4"))
                self.ui.tableWidget_2.setItem(1, 4, QTableWidgetItem(bot[i][6]))
                self.ui.tableWidget_2.setItem(1, 5, QTableWidgetItem(bot[i][7]))
                if bot[i][7] == "No":
                    self.time_4 = datetime.datetime.now()
                elif bot[i][7] == "Yes":
                    t4 = datetime.datetime.now()
                    temp4 = t4 - self.time_4
                    time4 = str(temp4.seconds)
                    self.ui.tableWidget_3.setItem(self.index, 0, QTableWidgetItem(bot[i][5]))
                    self.ui.tableWidget_3.setItem(self.index, 1, QTableWidgetItem(bot[i][3]))
                    self.ui.tableWidget_3.setItem(self.index, 2, QTableWidgetItem(bot[i][4]))
                    self.ui.tableWidget_3.setItem(self.index, 3, QTableWidgetItem("4"))
                    self.ui.tableWidget_3.setItem(self.index, 4, QTableWidgetItem(time4))
                    self.index += 1

    # Function to Mark Points on the Image
    def mark_points(self, img, bot):
        """
        Marking Points in the input image
        Also draw the lines of the path estimated
        """
        for i in bot:
            img = cv2.circle(img, (bot[i][0], bot[i][1]), 2, (255, 255, 0), 8)

        # Marking the Start Point and Goal point
        for i in self.dict:
            # Marking the Start Point and Goal point
            img = cv2.circle(img, self.dict[i][0], 2, (255, 0, 0), 8)
            img = cv2.circle(img, self.dict[i][1], 2, (0, 0, 255), 8)
            ls = self.dict[i][2]
            ls.insert(0, self.dict[i][0])
            # Marking the Minimized set of goalpoints
            for point1, point2 in zip(ls, ls[1:]):
                cv2.line(img, point1, point2, [0, 255, 0], 2)

        return img

    # Function to Convert CV Images to Qt Frames
    def qt_convert(self, cv_image):
        """
        This Function will responsible for converting cv_images to Qt Frames
        """
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        frame = QImage(image.data, image.shape[1], image.shape[0], QImage.Format_RGB888)
        self.frame = frame.scaled(790, 590, Qt.KeepAspectRatio)
        # return self.frame

# Main Function
def main(args):
    # Initializing ROS Node
    rospy.init_node('node_pyqt5', anonymous=True)
    # Creating instance for the class
    det = Detect()
    try:
        # Spin the Node
        rospy.spin()

    except KeyboardInterrupt:
        # ShutDown the Node
        rospy.loginfo("Shutting down")
        rospy.on_shutdown(det.clean_shutdown)
        # Destroying CV2 Windows
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)
