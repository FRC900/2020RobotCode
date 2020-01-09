import os
import rospkg
import rospy
import threading

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QGraphicsView, QPushButton
from std_msgs.msg import String


#custom service
from controllers_2019_msgs.srv import *

class BasicSub(Plugin):

    msg_data = "default"
    counter = 0

    msg_data = "default"
    counter = 0

    def __init__(self, context):
        #PLUGIN CODE
        super(BasicSub, self).__init__(context)

        # Give QObjects reasonable names
        self.setObjectName('BasicSub')
        rp = rospkg.RosPack()

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
        ui_file = os.path.join(rp.get_path('rqt_basic_sub'), 'resource', 'BasicSub.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget, {'QPushButton': QPushButton})

        # Give QObjects reasonable names
        self._widget.setObjectName('BasicSubUi')

        # Add widget to the user interface
        context.add_widget(self._widget)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self._widget.testButton.clicked.connect(self.printHi)
        #MY CODE
        #self._widget.lineControlMode.setReadOnly(True)
        #self._widget.lineControlMode.setText("init")



        #temp = String("test string")
        #self._widget.lineControlMode.setText(temp)

        #Subscriber
        self.sub = rospy.Subscriber("/hello", String,  self.callback)

        #Publisher

        def publish_thread(self):
            pub = rospy.Publisher('/hello', String, queue_size=10)
            r = rospy.Rate(10) # 10hz
            pub_counter = 0
            while not rospy.is_shutdown():
                pub.publish("hello " + str(pub_counter) + " ")
                pub_counter -= 1
                r.sleep()
        
        load_thread = threading.Thread(target=publish_thread, args=(self,))
        load_thread.start()
    

    myBool = True

    #Client
    def controller_client(self):
        rospy.wait_for_service('/frcrobot_jetson/panel_intake_controller/panel_command')
        caller = rospy.ServiceProxy("/frcrobot_jetson/panel_intake_controller/panel_command", PanelIntakeSrv)
        caller(self.myBool,self.myBool)
        self.myBool = not(self.myBool)

    def printHi(self, evt):
        print("hi!")
        self.controller_client()

    def shutdown_plugin(self):
        self.sub.unregister()

    def callback(self, msg):
        print(msg.data + str(self.counter))
        self.counter += 1
    


