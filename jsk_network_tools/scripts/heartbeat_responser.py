#!/usr/bin/env python
import rospy
from jsk_network_tools.msg import Heartbeat, HeartbeatResponse
from std_msgs.msg import Float32

class Responser():
    def __init__(self):
        rospy.init_node('heartbeat_responser')
        self.pub_upload_time = rospy.Publisher("heartbeat/upload_time", Float32)
        self.pub_response = rospy.Publisher("heartbeat/response", HeartbeatResponse)
        rospy.Subscriber("heartbeat/request", Heartbeat, self.callback)
        self.timer_update = False
        self.delay = rospy.Time(0)
        rospy.Timer(rospy.Duration(1), self.timer_callback)
        rospy.spin()

    def timer_callback(self, event):
        if self.timer_update:
            self.delay = self.delay + rospy.Duration(1)
            upload_time_msg = Float32(self.delay.to_sec())
            self.pub_upload_time.publish(upload_time_msg)
        self.timer_update = True
        
    def callback(self, heartbeat):
        self.timer_update = False
        res = HeartbeatResponse()
        res.header.stamp = rospy.Time.now()
        res.heartbeat = heartbeat
        upload_time = rospy.Time.now() - heartbeat.header.stamp
        self.delay = upload_time
        upload_time_msg = Float32(upload_time.to_sec())
        self.pub_upload_time.publish(upload_time_msg)

        self.pub_response.publish(res)
        rospy.loginfo("respond to msg published %s.%s" % (heartbeat.header.stamp.secs, heartbeat.header.stamp.nsecs))


if __name__ == '__main__':
    Responser()
