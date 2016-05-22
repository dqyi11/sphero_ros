#!/usr/bin/python

import sys, rospy, math
from PyQt4 import QtGui, QtCore
#from geometry_msgs.msg import Twist
#from std_msgs.msg import ColorRGBA, Float32, Bool

from sphero_swarm_node.msg import SpheroBackLed, SpheroColor, SpheroTwist, SpheroTurn, SpheroHeading, SpheroDisableStabilization

class SpheroListItem(QtGui.QListWidgetItem):
    
    def __init__(self, name, addr):
        super(QtGui.QListWidgetItem, self).__init__()
        self.name = name
        self.addr = addr
        self.setText(str(self))
 
    def __repr__(self):
        return str(self.name) + "   " + str(self.addr)

class LEDWidget(QtGui.QLabel):
   
    def __init__(self, size, rgb):
        super(QtGui.QLabel, self).__init__()
        pixmap = QtGui.QPixmap(size[0], size[1])
        self.setPixmap(pixmap)
        self.rgb = rgb
        self.setAutoFillBackground(True)
        self.setBackgroundRole(QtGui.QPalette.Base)

    def setRGB(self, r, g, b):
        self.rgb = [r,g,b]
        p = QtGui.QPalette()
        p.setColor(self.backgroundRole(), QtGui.QColor(self.rgb[0], self.rgb[1], self.rgb[2]))
        self.setPalette(p)
        self.update()
  
    def paintEvent(self, e):
        super(QtGui.QLabel, self).paintEvent(e)
        #qp = QtGui.QPainter()
        #qp.begin(self)
        #qp.end()             

class DashboardWidget(QtGui.QWidget):

    def __init__(self, parent):
        super(QtGui.QWidget, self).__init__()
        self.parentWindow = parent
        self.initUI()

    def initUI(self):
    
        self.bk_label = QtGui.QLabel("back LED")
        self.bk_sl = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.bk_sl.setMinimum(0)
        self.bk_sl.setMaximum(255)
        self.bk_sl.setValue(self.parentWindow.backLEDVal)
        self.bk_sl.setTickPosition(QtGui.QSlider.TicksBelow)
        self.bk_sl.setTickInterval(1)

        self.r_label = QtGui.QLabel("R")
        self.r_sl = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.r_sl.setMinimum(0)
        self.r_sl.setMaximum(255)
        self.r_sl.setValue(self.parentWindow.ledRVal)
        self.r_sl.setTickPosition(QtGui.QSlider.TicksBelow)
        self.r_sl.setTickInterval(1)
        self.r_sl.valueChanged.connect(self.valuechange)

        self.g_label = QtGui.QLabel("G")
        self.g_sl = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.g_sl.setMinimum(0)
        self.g_sl.setMaximum(255)
        self.g_sl.setValue(self.parentWindow.ledGVal)
        self.g_sl.setTickPosition(QtGui.QSlider.TicksBelow)
        self.g_sl.setTickInterval(1)
        self.g_sl.valueChanged.connect(self.valuechange)

        self.b_label = QtGui.QLabel("B")
        self.b_sl = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.b_sl.setMinimum(0)
        self.b_sl.setMaximum(255)
        self.b_sl.setValue(self.parentWindow.ledBVal)
        self.b_sl.setTickPosition(QtGui.QSlider.TicksBelow)
        self.b_sl.setTickInterval(1)
        self.b_sl.valueChanged.connect(self.valuechange)

        self.led = LEDWidget([20,20],
                             [self.parentWindow.ledRVal,
                              self.parentWindow.ledGVal,
                              self.parentWindow.ledBVal])
        
        self.btnUpdate = QtGui.QPushButton("Update")
        self.btnUpdate.clicked.connect(self.updateColor) 
    
        self.disableStabilizationRadioButton = QtGui.QRadioButton("Disable Stabilization")
        self.disableStabilizationRadioButton.setChecked(False)
        self.disableStabilizationRadioButton.toggled.connect(self.handleDisableStabilizationCheck)
 
        self.degLabel = QtGui.QLabel("Degree:")
        self.degTextbox = QtGui.QLineEdit()
        self.degTextbox.setText("15")
        self.degTextbox.setFixedWidth(40)
        self.leftBtn = QtGui.QPushButton("Counter Clockwise")
        self.leftBtn.clicked.connect(self.leftRotate)
        self.rightBtn = QtGui.QPushButton("Clockwise")
        self.rightBtn.clicked.connect(self.rightRotate)

        self.spheroListWidget = QtGui.QListWidget()
        self.refreshBtn = QtGui.QPushButton("Refresh")
        self.refreshBtn.clicked.connect(self.parentWindow.refreshDevices)
        btnGridLayout = QtGui.QGridLayout()
        btnGridLayout.addWidget(self.refreshBtn, 0, 4)

        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.led)
        r_layout = QtGui.QHBoxLayout()
        r_layout.addWidget(self.r_label)
        r_layout.addWidget(self.r_sl)
        layout.addLayout(r_layout)
        g_layout = QtGui.QHBoxLayout()
        g_layout.addWidget(self.g_label)
        g_layout.addWidget(self.g_sl)
        layout.addLayout(g_layout)
        b_layout = QtGui.QHBoxLayout()
        b_layout.addWidget(self.b_label)
        b_layout.addWidget(self.b_sl)
        layout.addLayout(b_layout)
        bk_layout = QtGui.QHBoxLayout()
        bk_layout.addWidget(self.bk_label)
        bk_layout.addWidget(self.bk_sl)
        layout.addLayout(bk_layout)
        btn_layout = QtGui.QHBoxLayout()
        btn_layout.addWidget(self.btnUpdate)
        layout.addLayout(btn_layout)   

        layout.addWidget(self.disableStabilizationRadioButton)
        ctrlLayout = QtGui.QHBoxLayout()
        ctrlLayout.addWidget(self.degLabel)
        ctrlLayout.addWidget(self.degTextbox)
        ctrlLayout.addWidget(self.leftBtn)
        ctrlLayout.addWidget(self.rightBtn)
        layout.addLayout(ctrlLayout)
        layout.addWidget(self.spheroListWidget)
        layout.addLayout(btnGridLayout)
        self.setLayout(layout)
        self.show()

    def updateColor(self):
        
        self.parentWindow.ledRVal = self.r_sl.value()
        self.parentWindow.ledGVal = self.g_sl.value()
        self.parentWindow.ledBVal = self.b_sl.value()        
        self.setLEDColor(self.parentWindow.ledRVal,
                         self.parentWindow.ledGVal,
                         self.parentWindow.ledBVal)
        self.led.setRGB(self.parentWindow.ledRVal,
                        self.parentWindow.ledGVal,
                        self.parentWindow.ledBVal)

        self.backLEDVal = self.bk_sl.value()
        self.setBackLED(self.parentWindow.backLEDVal)
        self.update()

    def valuechange(self):
        self.led.setRGB(self.r_sl.value(),
                        self.g_sl.value(),
                        self.b_sl.value())
        self.led.update()

    def updateHeading(self):
        val = self.headingSlider.value()
        self.parentWindow.setHeading(val)
        #self.headingSlider.setValue(180)
        self.update()

    def leftRotate(self):
        deg = int(self.degTextbox.displayText())
        self.parentWindow.setHeading(360-deg)

    def rightRotate(self):
        deg = int(self.degTextbox.displayText())
        self.parentWindow.setHeading(15)

    def headingChange(self, int):
        delta_val = self.headingSlider.value() - self.currentHeadingSliderValue 
        #self.parentWindow.setHeading(delta_val)
        self.currentHeadingSliderValue = self.headingSlider.value()

    def handleDisableStabilizationCheck(self):
        if self.disableStabilizationRadioButton.isChecked() == True:
            self.updateDisableStablization(True)
        else:
            self.updateDisableStablization(False)

    def setLEDColor(self, r, g, b):
        selected_items = self.spheroListWidget.selectedItems()
        if len(selected_items) > 0:
            for item in selected_items:
                self.parentWindow.setLEDColorByName(item.name, r, g, b)

    def setBackLED(self, val):
        selected_items = self.spheroListWidget.selectedItems()
        if len(selected_items) > 0:
            for item in selected_items:
                self.parentWindow.setBackLEDByName(item.name, val)

    def setHeading(self, val):
        selected_items = self.spheroListWidget.selectedItems()
        if len(selected_items) > 0:
            for item in selected_items:
                self.parentWindow.setHeadingByName(item.name, val)

    def setDisableStabilization(self, on):
        selected_items = self.spheroListWidget.selectedItems()
        if len(selected_items) > 0:
            for item in selected_items:
                self.parentWindow.setDisableStabilizationByName(item.name, on)


class SpheroDashboardForm(QtGui.QMainWindow):
    
    def __init__(self):
        super(QtGui.QMainWindow, self).__init__()
        self.resize(600, 300) 
        
        self.sphero_dict = {}
        rospy.init_node('sphero_swarm_dashboard', anonymous=True)
        self.cmdTurnPub = rospy.Publisher('cmd_turn', SpheroTurn, queue_size=1)     
 
        self.ledPub = rospy.Publisher('set_color', SpheroColor, queue_size=1)
        self.backLedPub = rospy.Publisher('set_back_led', SpheroBackLed, queue_size=1)   
        self.headingPub = rospy.Publisher('set_heading', SpheroHeading, queue_size=1)
        self.disableStabilizationPub = rospy.Publisher('disable_stabilization', SpheroDisableStabilization, queue_size=1)

        self.ledRVal = 0
        self.ledGVal = 0
        self.ledBVal = 0
        self.backLEDVal = 0

        #self.setLEDColor(self.ledRVal, self.ledGVal, self.ledBVal)
        #self.setBackLED(self.backLEDVal)

        self.initUI()

    def initUI(self):       
        self.dashboard = DashboardWidget(self)
        self.setCentralWidget(self.dashboard)
        self.setWindowTitle("Sphero Swarm dashboard")
        self.show()   

    def refreshDevices(self):

        self.dashboard.spheroListWidget.clear()
        self.sphero_dict = rospy.get_param('/sphero_swarm/connected')

        print(self.sphero_dict)

        for name in self.sphero_dict:
            bt_addr = self.sphero_dict[name]
            self.dashboard.spheroListWidget.addItem(SpheroListItem( name, bt_addr ))
        self.update()

    def setLEDColorByName(self, name, r, g, b):
        color = SpheroColor(name, float(r)/255.0, float(g)/255.0, float(b)/255.0, 1.0)
        self.ledPub.publish(color)

    def setBackLEDByName(self, name, val):
        light = SpheroBackLed(name, val)
        self.backLedPub.publish(light)

    def setHeadingByName(self, name, val):
        turning = SpheroTurn(name, val)
        self.cmdTurnPub.publish(turning)

        heading = SpheroHeading(name, 0.0)
        self.headingPub.publish(heading)

    def setDisableStabilizationByName(self, name, on):
        stab_data = SpheroDisableStabilization(name, on)
        self.disableStabilizationPub.publish(stab_data)             

if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    w = SpheroDashboardForm()
    w.show()
    sys.exit(app.exec_())
  
        
