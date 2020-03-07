import os
import rospy
import rospkg
import threading

import sys

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QGraphicsView, QPushButton, QRadioButton, QMessageBox, QHBoxLayout, QLabel, QButtonGroup
from python_qt_binding.QtCore import QCoreApplication
from python_qt_binding.QtGui import QPixmap 
import resource_rc

from behavior_actions.msg import AutoState, AutoMode
from imu_zero.srv import ImuZeroAngle
import std_msgs.msg
import roslibpy



class Dashboard(Plugin):

    msg_data = "default"
    def __init__(self, context):
        #Start client -rosbridge
        client = roslibpy.Ros(host='localhost', port=5803)
        client.run()
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
            if rospy.has_param("/auto/auto_mode_" + str(i)):
                auto_sequence = rospy.get_param("/auto/auto_mode_" + str(i))
               
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
            
        # auto state stuff
        self.autoState = 0
        self.displayAutoState() #display initial auto state
        listener1 = roslibpy.Topic(client, '/auto/auto_state', 'behavior_actions/AutoState')
        self.auto_state_sub = listener1.subscribe(self.autoStateCallback)

        # publish thread
        publish_thread = threading.Thread(target=self.publish_thread) #args=(self,))
        publish_thread.start()

        # number balls display
        self.zero_balls = QPixmap(":/images/0_balls.png")
        self.one_ball = QPixmap(":/images/1_ball.png")
        self.two_balls = QPixmap(":/images/2_balls.png")
        self.three_balls = QPixmap(":/images/3_balls.png")
        self.four_balls = QPixmap(":/images/4_balls.png")
        self.five_balls = QPixmap(":/images/5_balls.png")
        self.more_than_five_balls = QPixmap(":/images/more_than_5_balls.png")
        listener2 = roslibpy.Topic(client,'/num_powercells','std_msgs/UInt8')
        self.n_balls_sub = listener2.subscribe(self.nBallsCallback)
        self.n_balls = -1 #don't know n balls at first 

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)



    def autoStateCallback(self, msg):
        if(self.autoState != msg.id):
	    self.autoState = msg.id
            self.displayAutoState()
    
    def displayAutoState(self):
        if self.autoState == 0:
            self._widget.auto_state_display.setText("Not ready")
            self._widget.auto_state_display.setStyleSheet("background-color:#ff5555;")
        elif self.autoState == 1: 
            self._widget.auto_state_display.setText("Ready, waiting for auto period")
            self._widget.auto_state_display.setStyleSheet("background-color:#ffffff;") 
        elif self.autoState == 2: 
            self._widget.auto_state_display.setText("Running")
            self._widget.auto_state_display.setStyleSheet("background-color:#ffff00")       
        elif self.autoState == 3: 
            self._widget.auto_state_display.setText("Finished")
            self._widget.auto_state_display.setStyleSheet("background-color:#00ff00;")
	elif self.autoState == 4:
	    self._widget.auto_state_display.setText("Error")
            self._widget.auto_state_display.setStyleSheet("background-color:#ff5555;")


    def nBallsCallback(self, msg):
        if(self.n_balls != msg.data):
            self.n_balls = msg.data
            display = self._widget.n_balls_display
            
            if msg.data == 0:
                display.setPixmap(self.zero_balls)
            elif msg.data == 1:
                display.setPixmap(self.one_ball)
            elif msg.data == 2:
                display.setPixmap(self.two_balls)
            elif msg.data == 3:
                display.setPixmap(self.three_balls)
            elif msg.data == 4:
                display.setPixmap(self.four_balls)
            elif msg.data == 5:
                display.setPixmap(self.five_balls)
            elif msg.data > 5:
                display.setPixmap(self.more_than_five_balls)
            else:
                display.setText("Couldn't read # balls")


    def setImuAngle(self):
        client = roslibpy.Ros(host='localhost',port = 5803)
        client.run()
        angle = self._widget.imu_angle.value() # imu_angle is the text field (doublespinbox) that the user can edit to change the navx angle, defaulting to zero
        # call the service
        try:
            service = roslibpy.Service(client,'/imu/set_imu_zero',ImuZeroAngle)
           # rospy.wait_for_service("/imu/set_imu_zero", 1) # timeout in sec, TODO maybe put in config file?
            #TODO remove print
            #Service Request-rosbridge
            request = roslibpy.ServiceRequest()
            result = service.call(request)
            #result(angle)
            # change button to green color to indicate that the service call went through
            self._widget.set_imu_angle_button.setStyleSheet("background-color:#5eff00;")

        except (rospy.ServiceException, rospy.ROSException) as e: # the second exception happens if the wait for service times out
            self.errorPopup("Imu Set Angle Error", e) 
        client.close()

    def imuAngleChanged(self):
        # change button to red color if someone fiddled with the angle input, to indicate that input wasn't set yet
        self._widget.set_imu_angle_button.setStyleSheet("background-color:#ff0000;")


    def errorPopup(self, title, e):
            msg_box = QMessageBox()
            msg_box.setWindowTitle(title)
            msg_box.setIcon(QMessageBox.Warning)
            msg_box.setText("%s"%e)
            msg_box.exec_()

    #Publisher -> fake Auto States
    def publish_thread(self):
        client1 = roslibpy.Ros(host='localhost', port=5803)
        client1.run()
        
        pub = roslibpy.Topic(client1, '/auto/auto_mode', 'behavior_actions/AutoMode',queue_length=10)
        r = rospy.Rate(10) # 10hz
        pub.advertise()
        while client1.is_connected:
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            pub.publish(roslibpy.Message({'header':h,'auto_mode':self.auto_mode_button_group.checkedId()}))
            r.sleep()
        pub.unadvertise()
        client1.terminate()

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
    

    def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
        pass

