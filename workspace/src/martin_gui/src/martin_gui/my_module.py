import os
import rospy
import rospkg
from bebop_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

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
        ui_file = os.path.join(rospkg.RosPack().get_path('martin_gui'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        self.imu_msg_t0 = 0
        self.imu_msg_tn = 0
        self.imu_times = []
        
        self.cam_msg_t0 = 0
        self.cam_msg_tn = 0
        self.cam_times = []
        
        self.battery_sub = rospy.Subscriber("/bebop/states/common/CommonState/BatteryStateChanged", CommonCommonStateBatteryStateChanged, self.battery_callback)
        self.sensor_sub = rospy.Subscriber("imu", Imu, self.sensor_callback)
        self.cam_sub = rospy.Subscriber("bebop/image_mono", Image, self.camera_callback)
        self.takeoff_pub = rospy.Publisher("bebop/takeoff", Empty, queue_size=1)
        self.land_pub = rospy.Publisher("bebop/land", Empty, queue_size=1)
        self.startstop_pub = rospy.Publisher("/start_stop_recording", Empty, queue_size=1)
        self.toggle_pub = rospy.Publisher("/mode_toggle", Empty, queue_size=1)
	self.startover_pub = rospy.Publisher("/startover", Empty, queue_size=1)
        self._widget.recording_button.clicked.connect(self.startstop_button_press)
        self._widget.toggle_button.clicked.connect(self.toggle_button_press)
        self._widget.land_button.clicked.connect(self.land_button_press)
        self._widget.takeoff_button.clicked.connect(self.takeoff_button_press)
        self._widget.startover_button.clicked.connect(self.startover_button_press)
        self.cam_counter = 0
        self.sensor_counter = 0
        return
    

    def startover_button_press(self):
        msg = Empty()
        self.startover_pub.publish(msg)
        return

    def land_button_press(self):
        msg = Empty()
        self.land_pub.publish(msg)
        return
        
    def takeoff_button_press(self):
        msg = Empty()
        self.takeoff_pub.publish(msg)
        return
        
    def startstop_button_press(self):
        msg = Empty()
        self.startstop_pub.publish(msg)
        return
        
    def toggle_button_press(self):
        msg = Empty()
        self.toggle_pub.publish(msg)
        return

    def print_hz(self):
        if not self.imu_times:
            return
        mean = sum(self.imu_times) / len(self.imu_times)
        rate = 1./mean if mean > 0. else 0
        self._widget.imu_hz_text.setText("IMU: %.3f Hz"%(rate))
        return

    def print_cam_hz(self):
        if not self.cam_times:
            return
        mean = sum(self.cam_times) / len(self.cam_times)
        rate = 1./mean if mean > 0. else 0
        self._widget.cam_hz_text.setText("CAM: %.3f Hz"%(rate))
        return

    def sensor_callback(self, imu):
        curr_rostime = rospy.get_rostime()
        if curr_rostime.is_zero():
            self.imu_times = []
            return
        curr = curr_rostime.to_sec()
        if self.imu_msg_t0 < 0 or self.imu_msg_t0 > curr or self.imu_msg_t0 == 0 or curr - 	self.imu_msg_tn > 1:
            self.imu_msg_t0 = curr
            self.imu_msg_tn = curr
            self.imu_times = []
        else:
            self.imu_times.append(curr - self.imu_msg_tn)
            self.imu_msg_tn = curr

        if len(self.imu_times) > 500:
            self.imu_times.pop(0)
        self.sensor_counter+=1
        if self.sensor_counter % 100 == 0:
            self.print_hz()
        return

    def camera_callback(self, img):
        curr_rostime = rospy.get_rostime()
        if curr_rostime.is_zero():
            self.cam_times = []
            return
        curr = curr_rostime.to_sec()
        if self.cam_msg_t0 < 0 or self.cam_msg_t0 > curr or self.cam_msg_t0 == 0 or curr - self.cam_msg_tn > 1:
            self.cam_msg_t0 = curr
            self.cam_msg_tn = curr
            self.cam_times = []
        else:
            self.cam_times.append(curr - self.cam_msg_tn)
            self.cam_msg_tn = curr

        if len(self.cam_times) > 150:
            self.cam_times.pop(0)
        self.cam_counter+=1
        if self.cam_counter % 30 == 0:
            self.print_cam_hz()
        return

    def battery_callback(self, state):
        self._widget.battery_text.setText("Drone battery: %d%%"%state.percent)
        return

    def shutdown_plugin(self):
        self.sensor_sub.unregister()
        self.battery_sub.unregister()
        # TODO unregister all publishers here
        return

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
