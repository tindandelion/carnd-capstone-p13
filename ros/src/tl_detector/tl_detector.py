#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

from waypoints import Waypoints

STATE_COUNT_THRESHOLD = 3
LIGHT_NOT_FOUND = (-1, TrafficLight.UNKNOWN)


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber(
            '/current_pose', PoseStamped, self.update_current_pose)
        sub2 = rospy.Subscriber('/base_waypoints', Lane,
                                self.update_base_waypoints)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights',
                                TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher(
            '/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def update_current_pose(self, msg):
        self.pose = msg
        self.image_cb(None)

    def update_base_waypoints(self, lane):
        self.waypoints = Waypoints(lane.waypoints)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.publish_upcoming_light(light_wp)
        else:
            self.publish_upcoming_light(self.last_wp)

        self.state_count += 1

    def publish_upcoming_light(self, light_wp):
        rospy.logwarn("Upcoming light WP: %d", light_wp)
        self.upcoming_red_light_pub.publish(Int32(light_wp))

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        return TrafficLight.RED
        # if(not self.has_image):
        #     self.prev_light_loc = None
        #     return False

        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # # Get classification
        # return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not (self.pose and self.waypoints):
            return LIGHT_NOT_FOUND

        nearest_light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        min_dist = len(self.waypoints.waypoints)

        car_wp = self.waypoints.find_closest(self.pose.pose.position.x,
                                             self.pose.pose.position.y)
        for i, light in enumerate(self.lights):
            line_x, line_y = stop_line_positions[i]
            light_wp = self.waypoints.find_closest(line_x, line_y)
            dist = light_wp - car_wp
            if (dist >= 0) and (dist < min_dist):
                min_dist = dist
                nearest_light = (light, light_wp)

        if nearest_light:
            light, light_wp = nearest_light
            state = self.get_light_state(light)
            return light_wp, state
        else:
            return LIGHT_NOT_FOUND


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
