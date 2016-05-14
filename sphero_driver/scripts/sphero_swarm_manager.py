#!/usr/bin/python

import sys, rospy, math, bluetooth, time
from PyQt4 import QtGui, QtCore
from sphero_driver import sphero_driver

class BluetoothListItem(QtGui.QListWidgetItem):
    
    def __init__(self, name, addr):
        super(QtGui.QListWidgetItem, self).__init__()
        self.name = name
        self.addr = addr
        self.setText(str(self))
 
    def __repr__(self):
        return str(self.name) + "   " + str(self.addr)

class BluetoothConfig(QtGui.QWidget):

    def __init__(self, parentWindow):
        super(QtGui.QWidget, self).__init__()
        self.parentWindow = parentWindow
        self.initUI()

    def initUI(self):
        self.bluetoothLabel = QtGui.QLabel("Bluetooth Devices")
        self.bluetoothDeviceList = QtGui.QListWidget()
        self.scanBtn = QtGui.QPushButton("Scan")
        self.scanBtn.clicked.connect(self.scanBluetoothDevice)
        self.connectBtn = QtGui.QPushButton("Connet")
        self.connectBtn.clicked.connect(self.connectBluetoothDevice)
        self.connectAllBtn = QtGui.QPushButton("Connect All")
        self.connectAllBtn.clicked.connect(self.connectAllBluetoothDevice)
       
        layout = QtGui.QVBoxLayout() 
        layout.addWidget(self.bluetoothLabel)
        layout.addWidget(self.bluetoothDeviceList)
        btnLayout = QtGui.QHBoxLayout()
        btnLayout.addWidget(self.scanBtn)
        btnLayout.addWidget(self.connectBtn)
        btnLayout.addWidget(self.connectAllBtn)        

        layout.addLayout(btnLayout)
        self.setLayout(layout)

    def scanBluetoothDevice(self):
        self.bluetoothDeviceList.clear()
        self.bluetoothDeviceList.update()
        self.devices = []
        print "scanning..."
        nearby_devices = bluetooth.discover_devices(lookup_names=True)
        for bdaddr, bdname in nearby_devices:
            list_item = BluetoothListItem(bdname, bdaddr)
            self.bluetoothDeviceList.addItem(list_item)
            print list_item
        self.bluetoothDeviceList.update()
        print "finished"


    def connectBluetoothDevice(self):
        selected_items = self.bluetoothDeviceList.selectedItems()
        if len(selected_items) > 0:
            for item in selected_items:
                print "connect " + str(item.name)
                self.parentWindow.connectSphero(item.name, item.addr)
            self.scanBluetoothDevice()

    def connectAllBluetoothDevice(self):
        for i in range(self.bluetoothDeviceList.count()):
            item = self.bluetoothDeviceList.item(i)
            print "connect " + str(item.name)
            self.parentWindow.connectSphero(item.name, item.addr)
        self.scanBluetoothDevice()

class SpheroSwarmManagerWidget(QtGui.QWidget):
   
    def __init__(self, parentWindow):
        super(QtGui.QWidget, self).__init__()
        self.parentWindow = parentWindow
        self.spheroLabel = QtGui.QLabel("Sphero List")
        self.spheroListWidget = QtGui.QListWidget()

        self.disconnectBtn = QtGui.QPushButton("Disconnect")        
        self.disconnectBtn.clicked.connect(self.disconnectBluetoothDevice)
        self.disconnectAllBtn = QtGui.QPushButton("Disconnect All")
        self.disconnectAllBtn.clicked.connect(self.disconnectAllBluetoothDevice)
        self.testBtn = QtGui.QPushButton("Test")        
        self.testBtn.clicked.connect(self.testSphero)

        layout = QtGui.QVBoxLayout() 
        layout.addWidget(self.spheroLabel)
        layout.addWidget(self.spheroListWidget)
        btnLayout = QtGui.QHBoxLayout()
        btnLayout.addWidget(self.testBtn)
        btnLayout.addWidget(self.disconnectBtn)
        
        layout.addLayout(btnLayout)
        self.setLayout(layout)

    def updateList(self):
        self.spheroListWidget.clear()
        for s in self.parentWindow.sphero_list:
            print "add " + str(s.target_name)
            self.spheroListWidget.addItem(BluetoothListItem(s.target_name, s.target_address))
        self.spheroListWidget.update()

    def disconnectBluetoothDevice(self):
        selected_items = self.spheroListWidget.selectedItems()
        if len(selected_items) > 0:
            for item in selected_items:
                print "disconnect " + str(item.name)
                self.parentWindow.disconnectSphero(item.name, item.addr)
                #self.spheroListWidget.takeItem( self.spheroListWidget.row(item) )

    def disconnectAllBluetoothDevice(self):
        for i in range(self.spheroListWidget.count()):
            item = self.spheroListWidget.item(i)
            print "disconnect " + str(item.name)
            self.parentWindow.disconnectSphero(item.name, item.addr)
        #self.scanBluetoothDevice()

    def testSphero(self):
        #print "test Sphero"
        selected_items = self.spheroListWidget.selectedItems()
        if len(selected_items) > 0:
            for item in selected_items:
                sphero = self.parentWindow.findSphero(item.addr)
                self.parentWindow.testSphero(sphero)


class SpheroSwarmManagerForm(QtGui.QMainWindow):

    def __init__(self):
        super(QtGui.QMainWindow, self).__init__()
        self.resize(600, 400)
        self.bluetoothConfig = BluetoothConfig(self) 
        self.bluetoothConfig.hide()
        self.sphero_list = []
        self.initUI()      

    def initUI(self):
        self.bluetoothConfigAction = QtGui.QAction('Bluetooth', self)
        self.bluetoothConfigAction.triggered.connect(self.configBluetooth)

        menubar = self.menuBar()
        configMenu = menubar.addMenu('&Config')
        configMenu.addAction(self.bluetoothConfigAction)

        self.spheroMgr = SpheroSwarmManagerWidget(self)
        self.setCentralWidget(self.spheroMgr)

        self.setWindowTitle('Sphero Swarm')
        self.show()         
    
    def configBluetooth(self):
        self.bluetoothConfig.show()      

    def connectSphero(self, name, address):
        print "connect " + str(name) + "   " + str(address)

        if self.hasActiveSphero(address) == False:
            sphero = sphero_driver.Sphero(name, address)
            sphero.connect()
            sphero.set_raw_data_strm(40, 1, 0, False)
            sphero.start()
            self.sphero_list.append(sphero)

        self.spheroMgr.updateList()

    def disconnectSphero(self, name, address):
        print "disconnect " + str(name) + "   " + str(address)
        for s in self.sphero_list:
            if s.target_address == address:
                 s.disconnect()
                 self.sphero_list.remove(s)
        
        self.spheroMgr.updateList()

    def testSphero(self, sphero):
        
        print "TESTING " + str(sphero.target_name)
	time.sleep(2)
	print "disable stabilization"
	sphero.set_stablization(0, False)
	time.sleep(3)
	print "set color to RED"
	sphero.set_rgb_led(255,0,0,0,False)
	time.sleep(1)
	print "set color to GREEN"
	sphero.set_rgb_led(0,255,0,0,False)
	time.sleep(1)
	print "set color to BLUE"
	sphero.set_rgb_led(0,0,255,0,False)
	time.sleep(3)
	print "set back led"
	sphero.set_rgb_led(255,255,255,0,False)
	sphero.set_back_led(255,False)
	time.sleep(3)
	print "enable stablization"
	sphero.set_stablization(1, False)
	time.sleep(5)
	print "set aiming"
	sphero.roll(0, 90, 0, False)
	sphero.set_heading(90, False)
	time.sleep(3)

    def findSphero(self, addr):
        for s in self.sphero_list:
            if s.target_address == addr:
                return s  
        return None    

    def hasSphero(self, addr):
        for s in self.sphero_list:
            if s.target_address == addr:
                return True  
        return False
         
    def hasActiveSphero(self, addr):
        for s in self.sphero_list:
            if s.target_address == addr:
                return True
        return False

if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    w = SpheroSwarmManagerForm()
    w.show()
    sys.exit(app.exec_())
