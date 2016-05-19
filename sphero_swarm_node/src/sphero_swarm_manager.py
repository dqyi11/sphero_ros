#!/usr/bin/python

import sys, rospy, math, bluetooth, time, threading
from PyQt4 import QtGui, QtCore

class SpheroListItem(QtGui.QListWidgetItem):
    
    def __init__(self, name, addr):
        super(QtGui.QListWidgetItem, self).__init__()
        self.name = name
        self.addr = addr
        self.setText(str(self))
 
    def __repr__(self):
        return str(self.name) + "   " + str(self.addr)

class SpheroSwarmManagerWidget(QtGui.QWidget):
   
    def __init__(self, parentWindow):
        super(QtGui.QWidget, self).__init__()
        self.parentWindow = parentWindow
        
        self.nameLabel = QtGui.QLabel("Name")
        self.nameLineEdit = QtGui.QLineEdit()
        self.btaddrLabel = QtGui.QLabel("BT ADDR")
        self.btaddrLineEdit = QtGui.QLineEdit()
        self.connectBtn = QtGui.QPushButton("Connect")
        self.connectBtn.clicked.connect(self.connectSphero)
 
        self.spheroLabel = QtGui.QLabel("Sphero List")
        self.refreshBtn = QtGui.QPushButton("Refresh")
        self.refreshBtn.clicked.connect(self.updateList)
        self.spheroListWidget = QtGui.QListWidget()

        self.disconnectBtn = QtGui.QPushButton("Disconnect")        
        self.disconnectBtn.clicked.connect(self.disconnectSphero)
        self.disconnectAllBtn = QtGui.QPushButton("Disconnect All")
        self.disconnectAllBtn.clicked.connect(self.disconnectAllSpheros)
        self.testBtn = QtGui.QPushButton("Test")        
        self.testBtn.clicked.connect(self.testSphero)
        self.testAllBtn = QtGui.QPushButton("Test All")
        self.testAllBtn.clicked.connect(self.testAllSpheros)

        layout = QtGui.QVBoxLayout() 
        addLayout = QtGui.QHBoxLayout()
        addLayout.addWidget(self.nameLabel)
        addLayout.addWidget(self.nameLineEdit)
        addLayout.addWidget(self.btaddrLabel)
        addLayout.addWidget(self.btaddrLineEdit)
        addLayout.addWidget(self.connectBtn)
        layout.addLayout(addLayout)
        spheroLayout = QtGui.QGridLayout()
        spheroLayout.addWidget(self.spheroLabel, 0, 0)
        spheroLayout.addWidget(self.refreshBtn, 0, 6)
        layout.addLayout(spheroLayout)
        layout.addWidget(self.spheroListWidget)
        btnLayout = QtGui.QHBoxLayout()
        btnLayout.addWidget(self.testBtn)
        btnLayout.addWidget(self.testAllBtn)
        btnLayout.addWidget(self.disconnectBtn)
        btnLayout.addWidget(self.disconnectAllBtn)
        layout.addLayout(btnLayout)
        self.setLayout(layout)

    def updateList(self):
        self.spheroListWidget.clear()
        self.parentWindow.updateSpheroSwarm()

        print(self.parentWindow.sphero_dict)

        for name in self.parentWindow.sphero_dict:
            bt_addr = self.parentWindow.sphero_dict[name]
            self.spheroListWidget.addItem(SpheroListItem( name, bt_addr ))
        self.spheroListWidget.update()

    def connectSphero(self):
        if self.nameLineEdit.text() == "":
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Name is null")
            return
        if self.btaddrLineEdit.text() == "":
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Name is null")
            return
        self.parentWindow.connectSphero(self.nameLineEdit.text(), self.btaddrLineEdit.text() )

    def disconnectSphero(self):
        selected_items = self.spheroListWidget.selectedItems()
        if len(selected_items) > 0:
            for item in selected_items:
                print "disconnect " + str(item.name)
                self.parentWindow.disconnectSphero(item.name, item.addr, True)
 

    def disconnectAllSpheros(self):
        for i in range(self.spheroListWidget.count()):
            item = self.spheroListWidget.item(i)
            print "disconnect " + str(item.name)
            self.parentWindow.disconnectSphero(item.name, item.addr)

        self.updateList()


    def testSphero(self):
        #print "test Sphero"
        selected_items = self.spheroListWidget.selectedItems()
        if len(selected_items) > 0:
            for item in selected_items:
                sphero = self.parentWindow.findSphero(item.addr)
                print "TESTING " + str(sphero.target_name)
         	print "disable stabilization"
	        sphero.set_stablization(0, False)
	        print "set color to RED"
	        sphero.set_rgb_led(255,0,0,0,False)
	        print "set color to GREEN"
	        sphero.set_rgb_led(0,255,0,0,False)
	        print "set color to BLUE"
	        sphero.set_rgb_led(0,0,255,0,False)
	        print "set back led"
	        sphero.set_rgb_led(255,255,255,0,False)
	        sphero.set_back_led(255,False)
	        print "enable stablization"
	        sphero.set_stablization(1, False)
	        print "set aiming"
	        sphero.roll(0, 90, 0, False) 
	        sphero.set_heading(90, False)

    def testAllSpheros(self):
        print "TESTING ALL"
	print "disable stabilization"
        for i in range(self.spheroListWidget.count()):
            item = self.spheroListWidget.item(i)
            sphero = self.parentWindow.findSphero(item.addr)
	    sphero.set_stablization(0, False)
	print "set color to RED"
        for i in range(self.spheroListWidget.count()):
            item = self.spheroListWidget.item(i)
            sphero = self.parentWindow.findSphero(item.addr)
	    sphero.set_rgb_led(255,0,0,0,False)
	print "set color to GREEN"
        for i in range(self.spheroListWidget.count()):
            item = self.spheroListWidget.item(i)
            sphero = self.parentWindow.findSphero(item.addr)
	    sphero.set_rgb_led(0,255,0,0,False)
	print "set color to BLUE"
	for i in range(self.spheroListWidget.count()):
            item = self.spheroListWidget.item(i)
            sphero = self.parentWindow.findSphero(item.addr)
            sphero.set_rgb_led(0,0,255,0,False)
	print "set back led"
	for i in range(self.spheroListWidget.count()):
            item = self.spheroListWidget.item(i)
            sphero = self.parentWindow.findSphero(item.addr)
            sphero.set_rgb_led(255,255,255,0,False)
            sphero.set_back_led(255,False)
	print "enable stablization"
        for i in range(self.spheroListWidget.count()):
            item = self.spheroListWidget.item(i)
            sphero = self.parentWindow.findSphero(item.addr)	
            sphero.set_stablization(1, False)
	print "set aiming"
        for i in range(self.spheroListWidget.count()):
            item = self.spheroListWidget.item(i)
            sphero = self.parentWindow.findSphero(item.addr)
	    sphero.roll(0, 90, 0, False)
            sphero.set_heading(90, False)

class SpheroSwarmManagerForm(QtGui.QMainWindow):

    def __init__(self):
        super(QtGui.QMainWindow, self).__init__()
        self.resize(600, 400)
        self.sphero_dict = {}
        self.initUI()      

    def initUI(self):

        self.spheroMgr = SpheroSwarmManagerWidget(self)
        self.setCentralWidget(self.spheroMgr)

        self.setWindowTitle('Sphero Swarm')
        self.show() 
   
    def updateSpheroSwarm(self):

        self.sphero_dict = rospy.get_param('/sphero_swarm/team')

    def connectSphero(self, name, btaddr):
        rospy.wait_for_service('add_sphero')
        try:
            add_sphero = rospy.ServiceProxy('add_sphero', SpheroInfo)
            resp1 = add_sphero(name, btaddr)
            if resp1.success > 0:
                 print "%s (%s) has been added"% (name, btaddr)
                 return True
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return False

    def disconnectSphero(self, name, btaddr):
        rospy.wait_for_service('remove_sphero')
        try:
            remove_sphero = rospy.ServiceProxy('remove_sphero', SpheroInfo)
            resp1 = remove_sphero(name, btaddr)
            if resp1.success > 0:
                 print "%s (%s) has been removed"% (name, btaddr)
                 return True
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return False                
    
  

if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    w = SpheroSwarmManagerForm()
    w.show()
    sys.exit(app.exec_())
