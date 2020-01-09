import os
import rospy
import rospkg
import threading

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QGraphicsView, QPushButton, QRadioButton, QMessageBox, QHBoxLayout, QLabel, QButtonGroup
from python_qt_binding.QtCore import QCoreApplication
from std_msgs.msg import String
from imu_zero.srv import ImuZeroAngle

class Dashboard(Plugin):

    msg_data = "default"
    def __init__(self, context):
        super(Dashboard, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Dashboard')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_dashboard'), 'resource', 'Dashboard.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('DashboardUi')


        # Set up signal-slot connections
        self._widget.set_imu_angle_button.clicked.connect(self.setImuAngle)
        self._widget.imu_angle.valueChanged.connect(self.imuAngleChanged)

        
        # Add buttons for auto modes
        v_layout = self._widget.auto_mode_v_layout #vertical layout storing the buttons
        self.auto_mode_button_group = QButtonGroup(self._widget) # needs to be a member variable so the publisher can access it to see which auto mode was selected
        # Search for auto_mode config items
        for i in range(1,100): # loop will exit when can't find the next auto mode, so really only a while loop needed, but exiting at 100 will prevent infinite looping
            if rospy.has_param("/frcrobot_jetson/auto_mode_" + str(i)):
                auto_sequence = rospy.get_param("/frcrobot_jetson/auto_mode_" + str(i))
               
                new_auto_mode = QWidget()
                new_h_layout = QHBoxLayout()

                new_button = QRadioButton("Mode " + str(i))
                new_button.setStyleSheet("font-weight: bold") 
                self.auto_mode_button_group.addButton(new_button, i) #second arg is the button's id
                new_h_layout.addWidget( new_button )
                
                new_h_layout.addWidget( QLabel(", ".join(auto_sequence)) )
                
                new_auto_mode.setLayout( new_h_layout )
                v_layout.addWidget(new_auto_mode)
            else:
                print(str(i-1) + " auto modes found.")
                # if no auto modes found, inform the user with a label
                if (i-1) == 0:
                    v_layout.addWidget( QLabel("No auto modes found") )
                break #break out of for loop searching for auto modes
            
        self.autoState = 0    
        
        #Subscriber
        self.sub = rospy.Subscriber("/fakeAutoState", String, self.autoStateCallback)
        load_thread = threading.Thread(target=self.publish_thread) #args=(self,))
        load_thread.start()

      	#Display Auto States 
        if self.autoState == 1:
            self._widget.lineEdit.setText("Auto Node NOT Started")
            self._widget.lineEdit.setStyleSheet("background-color:#5eff00;")
        elif self.autoState == 2: 
            self._widget.lineEdit.setText("Waiting for Auto Period to Start")
            self._widget.lineEdit.setStyleSheet("background-color:#ffa500;") 
        elif self.autoState == 3: 
            self._widget.lineEdit.setText("Executing Auto Node")
            self._widget.lineEdit.setStyleSheet("background-color:#ffff00")       
        elif self.autoState == 4: 
            self._widget.lineEdit.setText("Auto Node Finished Running")
            self._widget.lineEdit.setStyleSheet("background-color:#00ff00;")
	elif self.autoState == 0:
	    self._widget.lineEdit.setText("error")
	


        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
    


    def setImuAngle(self):
        angle = self._widget.imu_angle.value() # imu_angle is the text field (doublespinbox) that the user can edit to change the navx angle, defaulting to zero

        # call the service
        try:
            rospy.wait_for_service("/navx_jetson/set_imu_zero", 1) # timeout in sec, TODO maybe put in config file?
            caller = rospy.ServiceProxy("/navx_jetson/set_imu_zero", ImuZeroAngle)
            caller(angle)
            # change button to green color to indicate that the service call went through
            self._widget.set_imu_angle_button.setStyleSheet("background-color:#5eff00;")

        except (rospy.ServiceException, rospy.ROSException) as e: # the second exception happens if the wait for service times out
            self.errorPopup("Imu Set Angle Error", e)



    def errorPopup(self, title, e):
            msg_box = QMessageBox()
            msg_box.setWindowTitle(title)
            msg_box.setIcon(QMessageBox.Warning)
            msg_box.setText("%s"%e)
            msg_box.exec_()


    def imuAngleChanged(self):
        # change button to red color if someone fiddled with the angle input, to indicate that input wasn't set yet
        self._widget.set_imu_angle_button.setStyleSheet("background-color:#ff0000;")

    #Publisher -> fake Auto States
    def publish_thread(self):
        pub = rospy.Publisher('/fakeAutoState', String, queue_size=10)
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            pub.publish("2")
            r.sleep()



    def setImuAngle(self):
        angle = self._widget.imu_angle.value() # imu_angle is the text field (doublespinbox) that the user can edit to change the navx angle, defaulting to zero

        # call the service
        try:
            rospy.wait_for_service("/navx_jetson/set_imu_zero", 1) # timeout in sec, TODO maybe put in config file?
            caller = rospy.ServiceProxy("/navx_jetson/set_imu_zero", ImuZeroAngle)
            caller(angle)
            # change button to green color to indicate that the service call went through
            self._widget.set_imu_angle_button.setStyleSheet("background-color:#5eff00;")

        except (rospy.ServiceException, rospy.ROSException) as e: # the second exception happens if the wait for service times out
            self.errorPopup("Imu Set Angle Error", e)



    def errorPopup(self, title, e):
            msg_box = QMessageBox()
            msg_box.setWindowTitle(title)
            msg_box.setIcon(QMessageBox.Warning)
            msg_box.setText("%s"%e)
            msg_box.exec_()


    def imuAngleChanged(self):
        # change button to red color if someone fiddled with the angle input, to indicate that input wasn't set yet
        self._widget.set_imu_angle_button.setStyleSheet("background-color:#ff0000;")

    #Publisher -> fake Auto States
    def publish_thread(self):
        pub = rospy.Publisher('/fakeAutoState', String, queue_size=10)
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            pub.publish("2")
            r.sleep()


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
    
    def autoStateCallback(self, msg):
	self.autoState = int(msg.data)

    def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
        pass
