from PyQt5 import QtCore, QtGui, QtWidgets
import sys
import rospy
from std_msgs.msg import Float32, Int8
import time
import cv2
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

class Ui_MainWindow(object):
    def _init_(self, MainWindow):
        super(Ui_MainWindow, self)._init_()
        self.setupUi(MainWindow)

        # Initializing ROS node
        rospy.init_node('science_gui', anonymous = True)

        # Setting up sensor subscribers
        rospy.Subscriber('bmp180_temperature', Float32, self.bmp180_temperature_callback)
        rospy.Subscriber('bmp180_pressure', Float32, self.bmp180_pressure_callback)
        rospy.Subscriber('dht11_humidity', Float32, self.dht11_humidity_callback)
        rospy.Subscriber('mlx90614_temperature_l', Float32, self.mlx90614_temperature_l_callback)
        rospy.Subscriber('lm393_humidity_l', Float32, self.lm393_humidity_l_callback)
        rospy.Subscriber('mlx90614_temperature_c', Float32, self.mlx90614_temperature_c_callback)
        rospy.Subscriber('lm393_humidity_c', Float32, self.lm393_humidity_c_callback)
        rospy.Subscriber('mlx90614_temperature_r', Float32, self.mlx90614_temperature_r_callback)
        rospy.Subscriber('lm393_humidity_r', Float32, self.lm393_humidity_r_callback)

        
        # Setting up pump subscribers
        # rospy.Subscriber('pump_l_status', Int8, self.pump_l_status_callback)
        # rospy.Subscriber('pump_c_status', Int8, self.pump_c_status_callback)
        # rospy.Subscriber('pump_r_status', Int8, self.pump_r_status_callback)

        time.sleep(2)

        # Setting up pump publishers
        self.pump_l_pub = rospy.Publisher('pump_l_status', Int8, queue_size = 5)

        self.pump_c_pub = rospy.Publisher('pump_c_status', Int8, queue_size = 5)

        self.pump_r_pub = rospy.Publisher('pump_r_status', Int8, queue_size = 5)

        # Halogen lamp publishers
        self.halogen_pub = rospy.Publisher('halo_status', Int8, queue_size = 5)
        time.sleep(1)
        self.halogen_pub.publish(0)
        time.sleep(0.5)

        # Spectrometer publishers and subscribers
        self.spectro_status_pub = rospy.Publisher('spectro_status', Int8, queue_size = 5)
        time.sleep(1)
        self.spectro_status_pub.publish(0)
        time.sleep(0.5)

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(998, 720)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.label_11 = QtWidgets.QLabel(self.centralwidget)
        self.label_11.setGeometry(QtCore.QRect(290, 540, 71, 16))
        self.label_11.setObjectName("label_11")
        self.label_9 = QtWidgets.QLabel(self.centralwidget)
        self.label_9.setGeometry(QtCore.QRect(60, 570, 47, 14))
        self.label_9.setObjectName("label_9")
        self.laBALL = QtWidgets.QLabel(self.centralwidget)
        self.laBALL.setGeometry(QtCore.QRect(520, 510, 91, 16))
        self.laBALL.setObjectName("laBALL")
        self.label_10 = QtWidgets.QLabel(self.centralwidget)
        self.label_10.setGeometry(QtCore.QRect(500, 30, 481, 371))
        self.label_10.setObjectName("label_10")
        self.line_3 = QtWidgets.QFrame(self.centralwidget)
        self.line_3.setGeometry(QtCore.QRect(490, 30, 20, 371))
        self.line_3.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_3.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_3.setObjectName("line_3")
        self.line_11 = QtWidgets.QFrame(self.centralwidget)
        self.line_11.setGeometry(QtCore.QRect(30, 450, 20, 191))
        self.line_11.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_11.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_11.setObjectName("line_11")
        self.line_7 = QtWidgets.QFrame(self.centralwidget)
        self.line_7.setGeometry(QtCore.QRect(510, 390, 471, 16))
        self.line_7.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_7.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_7.setObjectName("line_7")
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setGeometry(QtCore.QRect(740, 450, 221, 31))
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.pump_l_button = QtWidgets.QPushButton(self.centralwidget)
        self.pump_l_button.setGeometry(QtCore.QRect(340, 590, 91, 23))
        self.pump_l_button.setObjectName("pump_l_button")
        self.lm393_humidity_l = QtWidgets.QLabel(self.centralwidget)
        self.lm393_humidity_l.setGeometry(QtCore.QRect(400, 540, 71, 16))
        self.lm393_humidity_l.setText("")
        self.lm393_humidity_l.setObjectName("lm393_humidity_l")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(280, 450, 231, 31))
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.line_9 = QtWidgets.QFrame(self.centralwidget)
        self.line_9.setGeometry(QtCore.QRect(40, 470, 921, 16))
        self.line_9.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_9.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_9.setObjectName("line_9")
        self.line_14 = QtWidgets.QFrame(self.centralwidget)
        self.line_14.setGeometry(QtCore.QRect(730, 450, 20, 191))
        self.line_14.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_14.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_14.setObjectName("line_14")
        self.line_13 = QtWidgets.QFrame(self.centralwidget)
        self.line_13.setGeometry(QtCore.QRect(500, 450, 20, 191))
        self.line_13.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_13.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_13.setObjectName("line_13")
        self.lm393_humidity_r = QtWidgets.QLabel(self.centralwidget)
        self.lm393_humidity_r.setGeometry(QtCore.QRect(860, 540, 71, 16))
        self.lm393_humidity_r.setText("")
        self.lm393_humidity_r.setObjectName("lm393_humidity_r")
        self.bmp180_temperature = QtWidgets.QLabel(self.centralwidget)
        self.bmp180_temperature.setGeometry(QtCore.QRect(160, 510, 81, 16))
        self.bmp180_temperature.setText("")
        self.bmp180_temperature.setObjectName("bmp180_temperature")
        self.dht11_humidity = QtWidgets.QLabel(self.centralwidget)
        self.dht11_humidity.setGeometry(QtCore.QRect(160, 570, 81, 16))
        self.dht11_humidity.setText("")
        self.dht11_humidity.setObjectName("dht11_humidity")
        self.line_10 = QtWidgets.QFrame(self.centralwidget)
        self.line_10.setGeometry(QtCore.QRect(40, 640, 921, 16))
        self.line_10.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_10.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_10.setObjectName("line_10")
        self.mlx90614_temperature_c = QtWidgets.QLabel(self.centralwidget)
        self.mlx90614_temperature_c.setGeometry(QtCore.QRect(630, 510, 71, 16))
        self.mlx90614_temperature_c.setText("")
        self.mlx90614_temperature_c.setObjectName("mlx90614_temperature_c")
        self.mlx90614_temperature_r = QtWidgets.QLabel(self.centralwidget)
        self.mlx90614_temperature_r.setGeometry(QtCore.QRect(860, 510, 61, 16))
        self.mlx90614_temperature_r.setText("")
        self.mlx90614_temperature_r.setObjectName("mlx90614_temperature_r")
        self.bmp180_pressure = QtWidgets.QLabel(self.centralwidget)
        self.bmp180_pressure.setGeometry(QtCore.QRect(160, 540, 81, 16))
        self.bmp180_pressure.setText("")
        self.bmp180_pressure.setObjectName("bmp180_pressure")
        self.line_2 = QtWidgets.QFrame(self.centralwidget)
        self.line_2.setGeometry(QtCore.QRect(30, 20, 481, 16))
        self.line_2.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.line_4 = QtWidgets.QFrame(self.centralwidget)
        self.line_4.setGeometry(QtCore.QRect(970, 30, 20, 371))
        self.line_4.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_4.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_4.setObjectName("line_4")
        self.line_5 = QtWidgets.QFrame(self.centralwidget)
        self.line_5.setGeometry(QtCore.QRect(510, 20, 471, 16))
        self.line_5.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_5.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_5.setObjectName("line_5")
        self.line_8 = QtWidgets.QFrame(self.centralwidget)
        self.line_8.setGeometry(QtCore.QRect(40, 440, 921, 16))
        self.line_8.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_8.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_8.setObjectName("line_8")
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(510, 450, 231, 31))
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(40, 450, 231, 31))
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setGeometry(QtCore.QRect(60, 510, 81, 16))
        self.label_5.setObjectName("label_5")
        self.line_15 = QtWidgets.QFrame(self.centralwidget)
        self.line_15.setGeometry(QtCore.QRect(950, 450, 20, 191))
        self.line_15.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_15.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_15.setObjectName("line_15")
        self.label_12 = QtWidgets.QLabel(self.centralwidget)
        self.label_12.setGeometry(QtCore.QRect(520, 540, 71, 16))
        self.label_12.setObjectName("label_12")
        self.pump_r_button = QtWidgets.QPushButton(self.centralwidget)
        self.pump_r_button.setGeometry(QtCore.QRect(810, 590, 91, 23))
        self.pump_r_button.setObjectName("pushButton_5")
        self.line_6 = QtWidgets.QFrame(self.centralwidget)
        self.line_6.setGeometry(QtCore.QRect(30, 390, 481, 16))
        self.line_6.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_6.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_6.setObjectName("line_6")
        self.line_12 = QtWidgets.QFrame(self.centralwidget)
        self.line_12.setGeometry(QtCore.QRect(270, 450, 20, 191))
        self.line_12.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_12.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_12.setObjectName("line_12")
        self.label_13 = QtWidgets.QLabel(self.centralwidget)
        self.label_13.setGeometry(QtCore.QRect(750, 510, 91, 16))
        self.label_13.setObjectName("label_13")
        self.label_7 = QtWidgets.QLabel(self.centralwidget)
        self.label_7.setGeometry(QtCore.QRect(60, 540, 61, 16))
        self.label_7.setObjectName("label_7")
        self.line = QtWidgets.QFrame(self.centralwidget)
        self.line.setGeometry(QtCore.QRect(20, 30, 20, 371))
        self.line.setFrameShape(QtWidgets.QFrame.VLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.color_test_feed = QtWidgets.QLabel(self.centralwidget)
        self.color_test_feed.setGeometry(QtCore.QRect(30, 30, 481, 371))
        self.color_test_feed.setStyleSheet("border-color: rgb(0, 85, 255);")
        self.color_test_feed.setObjectName("color_test_feed")
        self.label_14 = QtWidgets.QLabel(self.centralwidget)
        self.label_14.setGeometry(QtCore.QRect(750, 540, 71, 16))
        self.label_14.setObjectName("label_14")
        self.lm393_humidity_c = QtWidgets.QLabel(self.centralwidget)
        self.lm393_humidity_c.setGeometry(QtCore.QRect(630, 540, 71, 16))
        self.lm393_humidity_c.setText("")
        self.lm393_humidity_c.setObjectName("lm393_humidity_c")
        self.mlx90614_temperature_l = QtWidgets.QLabel(self.centralwidget)
        self.mlx90614_temperature_l.setGeometry(QtCore.QRect(400, 510, 71, 16))
        self.mlx90614_temperature_l.setText("")
        self.mlx90614_temperature_l.setObjectName("mlx90614_temperature_l")
        self.pump_c_button = QtWidgets.QPushButton(self.centralwidget)
        self.pump_c_button.setGeometry(QtCore.QRect(570, 590, 91, 23))
        self.pump_c_button.setObjectName("pump_c_button")
        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_6.setGeometry(QtCore.QRect(290, 510, 91, 16))
        self.label_6.setObjectName("label_6")
        self.spectro_l_button = QtWidgets.QPushButton(self.centralwidget)
        self.spectro_l_button.setGeometry(QtCore.QRect(340, 620, 91, 23))
        self.spectro_l_button.setObjectName("spectro_l_button")
        self.spectro_c_button = QtWidgets.QPushButton(self.centralwidget)
        self.spectro_c_button.setGeometry(QtCore.QRect(570, 620, 91, 23))
        self.spectro_c_button.setObjectName("spectro_c_button")
        self.spectro_r_button = QtWidgets.QPushButton(self.centralwidget)
        self.spectro_r_button.setGeometry(QtCore.QRect(810, 620, 91, 23))
        self.spectro_r_button.setObjectName("spectro_r_button")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 998, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        # Connecting all buttons
        self.pump_l_button.clicked.connect(self.pump_l_button_clicked)
        self.pump_c_button.clicked.connect(self.pump_c_button_clicked)
        self.pump_r_button.clicked.connect(self.pump_r_button_clicked)

        self.spectro_l_button.clicked.connect(self.spectro_l_button_clicked)
        self.spectro_c_button.clicked.connect(self.spectro_c_button_clicked)
        self.spectro_r_button.clicked.connect(self.spectro_r_button_clicked)

        #self.spectro_result_button.clicked.connect(self.spectro_result_button_clicked)
        #self.color_result_button.clicked.connect(self.color_result_button_clicked)

        self.feed_start_button.clicked.connect(self.StartFeed)
        self.feed_stop_button.clicked.connect(self.StopFeed)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)


    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_11.setText(_translate("MainWindow", "Soil Moisture"))
        self.label_9.setText(_translate("MainWindow", "Humidity"))
        self.laBALL.setText(_translate("MainWindow", "Soil Temperature"))
        self.label_10.setText(_translate("MainWindow", "Spectro Feed"))
        self.label_4.setText(_translate("MainWindow", "Right Box"))
        self.pump_l_button.setText(_translate("MainWindow", "ACTUATE PUMP"))
        self.label_2.setText(_translate("MainWindow", "Left Box"))
        self.label_3.setText(_translate("MainWindow", "Center Box"))
        self.label.setText(_translate("MainWindow", "Atmospheric Conditions"))
        self.label_5.setText(_translate("MainWindow", "Air Temperature  "))
        self.label_12.setText(_translate("MainWindow", "Soil Moisture"))
        self.pump_r_button.setText(_translate("MainWindow", "ACTUATE PUMP"))
        self.label_13.setText(_translate("MainWindow", "Soil Temperature"))
        self.label_7.setText(_translate("MainWindow", "Pressure"))
        self.color_test_feed.setText(_translate("MainWindow", "Color Test Feed"))
        self.label_14.setText(_translate("MainWindow", "Soil Moisture"))
        self.pump_c_button.setText(_translate("MainWindow", "ACTUATE PUMP"))
        self.label_6.setText(_translate("MainWindow", "Soil Temperature"))
        self.spectro_l_button.setText(_translate("MainWindow", "SPECTRO"))
        self.spectro_c_button.setText(_translate("MainWindow", "SPECTRO"))
        self.spectro_r_button.setText(_translate("MainWindow", "SPECTRO"))


    # Subscriber callback functions
    def bmp180_temperature_callback(self, data):
        self.bmp180_temperature.setText(str(data.data))

    def bmp180_pressure_callback(self, data):
        self.bmp180_pressure.setText(str(data.data))
    
    def dht11_humidity_callback(self, data):
        self.dht11_humidity.setText(str(data.data))

    def mlx90614_temperature_l_callback(self, data):
        self.mlx90614_temperature_l.setText(str(data.data))    
    
    def lm393_humidity_l_callback(self, data):
        self.lm393_humidity_l.setText(str(data.data))

    def mlx90614_temperature_c_callback(self, data):
        self.mlx90614_temperature_c.setText(str(data.data))    

    def mlx90614_temperature_r_callback(self, data):
        self.mlx90614_temperature_r.setText(str(data.data))    
    
    def lm393_humidity_c_callback(self, data):
        self.lm393_humidity_c.setText(str(data.data))
    
    def lm393_humidity_r_callback(self, data):
        self.lm393_humidity_r.setText(str(data.data))
    
    def pump_l_button_clicked(self):
        self.pump_l_pub.publish(1)
        self.pump_l_button.setEnabled(False)
        time.sleep(0.5)
    
    def pump_c_button_clicked(self):
        self.pump_c_pub.publish(1)
        self.pump_c_button.setEnabled(False)
        time.sleep(0.5)

    def pump_r_button_clicked(self):
        self.pump_r_pub.publish(1)
        self.pump_r_button.setEnabled(False)
        time.sleep(0.5)

    def spectro_l_button_clicked(self):
        self.spectro_status_pub.publish(1)
        time.sleep(0.5)
        self.halogen_pub.publish(0)
        time.sleep(0.5)
    
    def spectro_c_button_clicked(self):
        self.spectro_status_pub.publish(2)
        time.sleep(0.5)
        self.halogen_pub.publish(0)
        time.sleep(0.5)
    
    def spectro_r_button_clicked(self):
        self.spectro_status_pub.publish(3)
        time.sleep(0.5)
    
    def spectro_result_button_clicked(self):
        self.halogen_pub.publish(1)
        time.sleep(0.5)
        self.halogen_pub.publish(0)
        time.sleep(0.5)
    
    def ImageUpdateSlot(self, Image):
        self.color_test_feed.setPixmap(QPixmap.fromImage(Image))

    def CancelFeed(self):
        self.Worker1.stop()

    def StartFeed(self):
        self.Worker1 = Worker1()
        self.Worker1.start()
        self.Worker1.ImageUpdate.connect(self.ImageUpdateSlot)

class Worker1(QThread):
    ImageUpdate = pyqtSignal(QImage)
    def run(self):
        self.ThreadActive = True
        Capture = cv2.VideoCapture(0)   # Change this accordingly
        while self.ThreadActive:
            ret, frame = Capture.read()
            if ret:
                Image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                FlippedImage = cv2.flip(Image, 1)
                ConvertToQtFormat = QImage(FlippedImage.data, FlippedImage.shape[1], FlippedImage.shape[0], QImage.Format_RGB888)
                Pic = ConvertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
                self.ImageUpdate.emit(Pic)
        Capture.release()

    def stop(self):
        self.ThreadActive = False

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
    rospy.spin()
