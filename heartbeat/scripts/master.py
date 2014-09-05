#!/usr/bin/env python
import roslib
roslib.load_manifest('heartbeat')
import rospy
from heartbeat.msg import Heartbeat, HeartbeatResponse

class Master():
    def __init__(self):
        rospy.init_node('heartbeat_master')
        self.pub_request = rospy.Publisher("heartbeat/request", Heartbeat)
        rospy.Subscriber("heartbeat/response", HeartbeatResponse, self.callback)
        self.rate = 1.0

        while not rospy.is_shutdown():
            msg = Heartbeat()
            msg.rate = self.rate
            msg.header.stamp = rospy.Time.now()
            self.pub_request.publish(msg)
            rospy.sleep(1.0/self.rate)

    def callback(self, data):
        now = rospy.Time.now()
        upload_time = data.header.stamp - data.heartbeat.header.stamp
        download_time = now - data.header.stamp
        rospy.loginfo("upload_time: %d.%09d" % (upload_time.secs, upload_time.nsecs))
        rospy.loginfo("download_time: %d.%09d" % (download_time.secs, download_time.nsecs))
        #rospy.loginfo("header, %s" % data.heartbeat.header.stamp)
        #rospy.loginfo("%09d" % upload_time.to_nsec())

if __name__ == '__main__':
    Master()
