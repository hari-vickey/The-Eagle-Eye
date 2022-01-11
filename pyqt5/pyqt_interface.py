# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'round1.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!
import cv2
import sys
import time
import threading
from PIL import Image
from numpy import asarray
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import QtGui, QtWidgets
from PyQt5.QtCore import QTimer,QDateTime

# sys.path.insert(0, "D:/git learning/The-Eagle-Eye/test_programs")

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):

        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1920, 1080)
        MainWindow.setTabShape(QTabWidget.Rounded)

        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

    # For adding background wallpaper for the main window 
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(0, 0, 1920, 1090))
        self.label.setPixmap(QPixmap("images/mainwindow.png"))

    # For creating the Titlebar in the interface
        self.textBrowser = QTextBrowser(self.centralwidget)
        self.textBrowser.setGeometry(QRect(0, 0, 1920, 111))
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
        self.Processing.setGeometry(QRect(920, 260, 921, 250))
        # self.Processing.setMaximumSize(QSize(921, 16777215))
        self.Processing.setFont(self.font)
        self.Processing.setAutoFillBackground(True)
        self.Processing.setTabShape(QTabWidget.Triangular)

        self.tab = QWidget()
        self.tab.setObjectName("tab")
        self.tableWidget_2 = QTableWidget(self.tab)
        self.tableWidget_2.setGeometry(QRect(0, 0, 911, 221))
        self.tableWidget_2.setObjectName("tableWidget_2")
        self.tableWidget_2.setColumnCount(7)
        self.tableWidget_2.setRowCount(5)
        item = QTableWidgetItem()
        self.tableWidget_2.setVerticalHeaderItem(0, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setVerticalHeaderItem(1, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setVerticalHeaderItem(2, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setVerticalHeaderItem(3, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setVerticalHeaderItem(4, item)
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
        item = QTableWidgetItem()
        self.tableWidget_2.setHorizontalHeaderItem(6, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setItem(0, 0, item)
        item = QTableWidgetItem()
        item.setFlags(Qt.ItemIsSelectable|Qt.ItemIsEditable|Qt.ItemIsDragEnabled|Qt.ItemIsDropEnabled|Qt.ItemIsUserCheckable|Qt.ItemIsEnabled|Qt.ItemIsTristate)
        self.tableWidget_2.setItem(0, 1, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setItem(0, 3, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setItem(1, 0, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setItem(1, 3, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setItem(3, 0, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setItem(3, 3, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setItem(4, 0, item)
        item = QTableWidgetItem()
        self.tableWidget_2.setItem(4, 3, item)
        self.tableWidget_2.horizontalHeader().setDefaultSectionSize(200)
        self.tableWidget_2.verticalHeader().setDefaultSectionSize(45)
        self.tableWidget_2.verticalHeader().setHighlightSections(True)
        self.tableWidget_2.horizontalHeader().setDefaultSectionSize(178)
        self.tableWidget_2.horizontalHeader().setMinimumSectionSize(120)
        self.Processing.addTab(self.tab, "")

        self.tab_2 = QWidget()
        self.tab_2.setObjectName("tab_2")
        self.tableWidget_4 = QTableWidget(self.tab_2)
        self.tableWidget_4.setGeometry(QRect(0, 0, 911, 221))
        self.tableWidget_4.setObjectName("tableWidget_4")
        self.tableWidget_4.setColumnCount(3)
        self.tableWidget_4.setRowCount(4)
        item = QTableWidgetItem()
        self.tableWidget_4.setVerticalHeaderItem(0, item)
        item = QTableWidgetItem()
        self.tableWidget_4.setVerticalHeaderItem(1, item)
        item = QTableWidgetItem()
        item.setBackground(QColor(255, 85, 127))
        self.tableWidget_4.setVerticalHeaderItem(2, item)
        item = QTableWidgetItem()
        self.tableWidget_4.setVerticalHeaderItem(3, item)
        item = QTableWidgetItem()
        self.tableWidget_4.setHorizontalHeaderItem(0, item)
        item = QTableWidgetItem()
        self.tableWidget_4.setHorizontalHeaderItem(1, item)
        item = QTableWidgetItem()
        self.tableWidget_4.setHorizontalHeaderItem(2, item)
        item = QTableWidgetItem()
        self.tableWidget_4.setItem(0, 0, item)
        item = QTableWidgetItem()
        brush = QBrush(QColor(255, 170, 0))
        brush.setStyle(Qt.NoBrush)
        item.setForeground(brush)
        self.tableWidget_4.setItem(1, 0, item)
        item = QTableWidgetItem()
        self.tableWidget_4.setItem(2, 0, item)
        item = QTableWidgetItem()
        self.tableWidget_4.setItem(3, 0, item)
        self.tableWidget_4.horizontalHeader().setDefaultSectionSize(295)
        self.tableWidget_4.verticalHeader().setDefaultSectionSize(45)
        self.tableWidget_4.verticalHeader().setHighlightSections(True)
        self.Processing.addTab(self.tab_2, "")

        self.tab_5 = QWidget()
        self.tab_5.setObjectName("tab_5")
        self.tableWidget_3 = QTableWidget(self.tab_5)
        self.tableWidget_3.setGeometry(QRect(0, 0, 911, 221))
        self.tableWidget_3.setObjectName("tableWidget_3")
        self.tableWidget_3.setColumnCount(5)
        self.tableWidget_3.setRowCount(5)
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
        self.tableWidget_3.setItem(0, 0, item)
        item = QTableWidgetItem()
        self.tableWidget_3.setItem(1, 0, item)
        item = QTableWidgetItem()
        self.tableWidget_3.setItem(2, 0, item)
        item = QTableWidgetItem()
        self.tableWidget_3.setItem(3, 0, item)
        item = QTableWidgetItem()
        self.tableWidget_3.setItem(4, 0, item)
        self.tableWidget_3.horizontalHeader().setDefaultSectionSize(178)
        self.tableWidget_3.horizontalHeader().setMinimumSectionSize(120)
        self.Processing.addTab(self.tab_5,"")

    # For Creating frame for video streaming 
        self.frame = QLabel(self.centralwidget)
        self.frame.setGeometry(QRect(80, 160, 800, 600))
        pixmap = QPixmap('images/eagle_eye.jpg')
        P = pixmap.scaled(1200, 600, Qt.KeepAspectRatio)
        self.frame.setPixmap(P)
        self.frame.setAutoFillBackground(True)
        self.frame.setFrameShape(QFrame.Box)
        self.frame.setFrameShadow(QFrame.Raised)
        self.frame.setLineWidth(5)
        self.frame.setObjectName("frame")
        
    # For displaying timer content
        self.textEdit = QTextEdit(self.centralwidget)
        self.textEdit.setObjectName(u"textEdit")
        self.textEdit.setGeometry(QRect(1500, 550, 200, 87))

    # For showing the text "Run time" 
        self.label_2 = QLabel(self.centralwidget)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(1350, 550, 200, 90))

    # For displaying "Run time " content
        self.textEdit = QTextEdit(self.centralwidget)
        self.textEdit.setObjectName(u"textEdit")
        self.textEdit.setGeometry(QRect(400, 900, 200, 87))

    # For showing the text "Run time" 
        self.label_2 = QLabel(self.centralwidget)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(250, 900, 200, 90))

    # For adding flipkart logo in the titlebar
        self.flipkart_logo = QLabel(self.centralwidget)
        self.flipkart_logo.setObjectName(u"flipkart_logo")
        self.flipkart_logo.setGeometry(QRect(1680, -50, 220, 220))
        logo = QPixmap("images/Flipkart_logo.png")
        flipkart_pixmap = logo.scaled(220, 220, Qt.KeepAspectRatio, Qt.FastTransformation)
        self.flipkart_logo.setPixmap(flipkart_pixmap)

    #For adding eagleeye logo in the titlebar
        self.eagleeye_logo = QLabel(self.centralwidget)
        self.eagleeye_logo.setObjectName(u"eagleeye_logo")
        self.eagleeye_logo.setGeometry(QRect(0, -55, 220, 220))
        eagle = QPixmap("images/eagle_eye.jpg")
        eagleeye_pixmap = eagle.scaled(170, 180, Qt.KeepAspectRatio, Qt.FastTransformation)
        self.eagleeye_logo.setPixmap(eagleeye_pixmap)


    # For Creating Start PushButton 
        self.pushButton = QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QRect(309, 800, 141, 51))
        self.pushButton.setObjectName("pushButton")
        self.pushButton.setStyleSheet("border:2px; background-color: #00FF00; border-radius : 20px")

    # For Creating Stop PushButton 
        self.pushButton_2 = QPushButton(self.centralwidget)
        self.pushButton_2.setGeometry(QRect(510, 800,141,51))
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_2.setStyleSheet("border:2px; background-color: red; border-radius : 20px")

        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.Processing.setCurrentIndex(2)
        QMetaObject.connectSlotsByName(MainWindow)

    # # For searchbar input
    #     self.lineEdit_2= QLineEdit(self.centralwidget)
    #     self.lineEdit_2.setObjectName(u"line_Edit_2")
    #     self.lineEdit_2.setGeometry(990, 190, 331, 51)

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
        item = self.tableWidget_2.verticalHeaderItem(1)
        item.setText(_translate("MainWindow", "1"))
        item = self.tableWidget_2.verticalHeaderItem(2)
        item.setText(_translate("MainWindow", "2"))
        item = self.tableWidget_2.verticalHeaderItem(3)
        item.setText(_translate("MainWindow", "3"))
        item = self.tableWidget_2.verticalHeaderItem(4)
        item.setText(_translate("MainWindow", "4"))
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
        item = self.tableWidget_2.horizontalHeaderItem(6)
        item.setText(_translate("MainWindow", "Location"))
        __sortingEnabled = self.tableWidget_2.isSortingEnabled()
        self.tableWidget_2.setSortingEnabled(False)
        self.tableWidget_2.setSortingEnabled(__sortingEnabled)
        self.Processing.setTabText(self.Processing.indexOf(self.tab), _translate("MainWindow", "Processing"))

    # For entering the values in the table
        for i in range(1,5):
                self.tableWidget_2.setItem(i,0, QTableWidgetItem("Package"+str(i))) 

    # For the Tab "Yet to Dispatch" in the table
        item = self.tableWidget_4.verticalHeaderItem(0)
        item.setText(_translate("MainWindow", "1"))
        item = self.tableWidget_4.verticalHeaderItem(1)
        item.setText(_translate("MainWindow", "2"))
        item = self.tableWidget_4.verticalHeaderItem(2)
        item.setText(_translate("MainWindow", "3"))
        item = self.tableWidget_4.verticalHeaderItem(3)
        item.setText(_translate("MainWindow", "4"))
        item = self.tableWidget_4.horizontalHeaderItem(0)
        item.setText(_translate("MainWindow", "Package ID"))
        item = self.tableWidget_4.horizontalHeaderItem(1)
        item.setText(_translate("MainWindow", "Induct Station"))
        item = self.tableWidget_4.horizontalHeaderItem(2)
        item.setText(_translate("MainWindow", "City"))
        __sortingEnabled = self.tableWidget_4.isSortingEnabled()
        self.tableWidget_4.setSortingEnabled(False)
        self.tableWidget_4.setSortingEnabled(__sortingEnabled)
        self.Processing.setTabText(self.Processing.indexOf(self.tab_2), _translate("MainWindow", "Yet to Dispatch"))
        # self.tab_2.setStyleSheet("background-color: pink")

    # For the Tab "Delivered" in the table 
        item = self.tableWidget_3.verticalHeaderItem(1)
        item.setText(_translate("MainWindow", "1"))
        item = self.tableWidget_3.verticalHeaderItem(2)
        item.setText(_translate("MainWindow", "2"))
        item = self.tableWidget_3.verticalHeaderItem(3)
        item.setText(_translate("MainWindow", "3"))
        item = self.tableWidget_3.verticalHeaderItem(4)
        item.setText(_translate("MainWindow", "4"))
        item = self.tableWidget_3.horizontalHeaderItem(0)
        item.setText(_translate("MainWindow", "Package ID"))
        item = self.tableWidget_3.horizontalHeaderItem(1)
        item.setText(_translate("MainWindow", "City"))
        item = self.tableWidget_3.horizontalHeaderItem(2)
        item.setText(_translate("MainWindow", "Induct Station"))
        item = self.tableWidget_3.horizontalHeaderItem(3)
        item.setText(_translate("MainWindow", "Bot ID"))
        item = self.tableWidget_3.horizontalHeaderItem(4)
        item.setText(_translate("MainWindow", "Time"))
        __sortingEnabled = self.tableWidget_3.isSortingEnabled()
        self.tableWidget_3.setSortingEnabled(False) 
        self.tableWidget_3.setSortingEnabled(__sortingEnabled)
        self.Processing.setTabText(self.Processing.indexOf(self.tab_5), _translate("MainWindow", "Delivered"))
        # self.tab_5.setStyleSheet("background-color: lightgreen")
        
    # For writing text in the start push button and enabling the video 
        self.flag = True
        self.Worker1 = Worker1()
        self.pushButton.setText(_translate("MainWindow", "START"))
        self.pushButton.setFont(QFont('Times', 15))
        self.pushButton.clicked.connect(self.StartFeed)

    # For writing text in the stop push button and enabling the video 
        self.pushButton_2.setText(_translate("MainWindow", "STOP"))
        self.pushButton_2.setFont(QFont('Times', 15))
        self.pushButton_2.clicked.connect(self.StopFeed)
        # self.toolBar.setWindowTitle(_translate("MainWindow", "toolBar"))

    # For writing "Run time" text in label_2 
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Run time", None))
        self.label_2.setFont(QFont('Times', 20))

        self.timer1 = QTimer()
        self.Worker1.ImageUpdate.connect(self.ImageUpdateSlot)
        self.start = time.time()
        self.s = 0

        self.label.setText("")
  
    def ImageUpdateSlot(self, Image1):
        """ Function for updating the image in the frame """
        self.frame.setPixmap(QPixmap.fromImage(Image1))
     
    def StartFeed(self):
        """ Function use to start the video and timer """
        self.flag == True
        self.Worker1.start()
        self.timer1.timeout.connect(self.showTime)
        self.timer1.start(1000)

    def StopFeed(self):
        """ Function use to stop the video and timer """
        self.flag = False
        self.Worker1.stop()
 
    def showTime(self):
        """ method called by timer """
        if self.flag == True: 
            text = int(time.time()-self.start)
            fin = str(text)
            fi_s = str(0)
            if text%60 == 0:
                self.s = self.s + 1
                self.start = self.start + 60
            fi_s = str(self.s)
        # showing text
            self.textEdit.setText(QCoreApplication.translate("MainWindow", fi_s + ":" + fin, None))
            self.textEdit.setFont(QFont('Times', 20))

class Worker1(QThread):
    """ Class for streaming the video """
    ImageUpdate = pyqtSignal(QImage)
  
    def run(self):
        """ Function for start streaming the video """
        self.ThreadActive = True
        Capture = cv2.VideoCapture(0)
        while self.ThreadActive:
            ret, frame = Capture.read()
            if ret:
                Image1 = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                FlippedImage = cv2.flip(Image1, 1)
                ConvertToQtFormat = QImage(FlippedImage.data, FlippedImage.shape[1], FlippedImage.shape[0], QImage.Format_RGB888)
                Pic = ConvertToQtFormat.scaled(800, 600, Qt.KeepAspectRatio)
                self.ImageUpdate.emit(Pic)

    def stop(self):
        """ Function for stop streaming the video """
        self.ThreadActive = False
        Imagelogo = Image.open("images/black.jpg")
        numpyarray = asarray(Imagelogo)
        Imagelogo = cv2.cvtColor(numpyarray, cv2.COLOR_BGR2RGB)
        ConvertToQtFormat = QImage(Imagelogo.data, Imagelogo.shape[1], Imagelogo.shape[0], QImage.Format_RGB888)
        Pic = ConvertToQtFormat.scaled(1200, 600, Qt.KeepAspectRatio)
        self.ImageUpdate.emit(Pic)
        self.quit()

if __name__ == "__main__":
    app = QApplication(sys.argv) 
    MainWindow = QMainWindow()
    # MainWindow.setStyleSheet("QMainWindow{background-image: url ('22.png')}")
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
