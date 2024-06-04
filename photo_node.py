"""
Copyright (c) 2024 git-ToxiouS

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import gi
import time

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy


class PhotoNode(Node):
    def __init__(self):
        super().__init__('photo_node')
        self.counter_ = 0
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.input, 1)
        self.photo_button = 9

    def input(self, joy_cmd):
        # Run the pipeline when a button is pressed
        if joy_cmd.buttons[self.photo_button]:
            self.gst()

            # Sleep to let the pipeline shut down and print a message with the count
            time.sleep(0.5)
            self.get_logger().info("Photo nr "+str(self.counter_))

            # Increase counter
            self.counter_ = self.counter_ + 1

    def on_message(self, bus, message, loop):
        if message.type == Gst.MessageType.EOS:
            self.get_logger().info("End-Of-Stream reached")
            loop.quit()
        elif message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.get_logger().info("Error received from element %s: %s" % (message.src.get_name(), err))
            self.get_logger().info("Debugging information: %s" % debug)
            loop.quit()

    def gst(self):
        Gst.init(None)

        # Create the main loop
        loop = GLib.MainLoop()
    
        # Make a photo and keep refreshing it at 30 fps
        pipeline = Gst.parse_launch("qtiqmmfsrc ! video/x-raw,width=1920,height=1080,framerate=30/1 ! jpegenc ! multifilesink location=/root/"+str(self.counter_)+".jpg max-files=1")

        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_message, loop)

        pipeline.set_state(Gst.State.PLAYING)

        # Sleep for a brief moment to let the pipeline start up and capture the frame
        time.sleep(0.5)

        # Send End-of-Stream to stop the pipeline
        pipeline.send_event(Gst.Event.new_eos())

        # Run the main loop
        try:
            loop.run()
        except:
            pass

        # Clean up
        pipeline.set_state(Gst.State.NULL)


### MAIN FUNCTION ###
def main(args=None):
    print("Photo node started")
    # Initialize rclpy library
    rclpy.init(args=args)
    # Spin the node
    photo = PhotoNode()
    rclpy.spin(photo)

    photo.destroy_node()

    rclpy.shutdown()

### IF NAME ###
if __name__ == '__main__':
    main()
