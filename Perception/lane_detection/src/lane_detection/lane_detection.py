import math
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
import cv2

from numpy import array, linspace
from sklearn.neighbors import KernelDensity
from matplotlib.pyplot import plot
from scipy.signal import argrelextrema
import matplotlib.pyplot as plt

class LaneDetector(object):

    def __init__(self, role_name):
            self.bridge = CvBridge()
            self.imgwidth = 400
            self.imgheight = 300
            self._semantic_img = np.zeros((self.imgheight, self.imgwidth, 3), np.uint8)
            self._roadmark_img = np.zeros((self.imgheight, self.imgwidth, 3), np.uint8)
            self._semanticseg_subscriber = rospy.Subscriber("/carla/{}/camera/semantic_segmentation/front/image_segmentation".format(role_name), Image, self.semanticseg_updated)
            self.roadmark_publisher = rospy.Publisher("/psaf/{}/roadmark".format(role_name), Image, queue_size=1)   
            self.stopline_publisher = rospy.Publisher("/psaf/{}/stopline_distance".format(role_name), Float64, queue_size=1) 
            self.cluster_bandwidth = 0.1
            self.cluster_resolution = 10000
        
    def run_step(self):
        """
        Run HoughTransformation, cluster lines, compute offset.
        """
        ## old code
        # height, width, channels = self._semantic_img.shape
        # for h in range(height):
        #     for w in range(width):
        #         if (self._semantic_img[h, w, :] == [50, 234, 157]).all():
        #             self._roadmark_img[h,w,:] = [255,255,255]
        #         else:
        #             self._roadmark_img[h,w,:] = [0,0,0]
        # minLineLength = 3
        # maxLineGap = 10
        # gray = cv2.cvtColor(self._roadmark_img, cv2.COLOR_BGR2GRAY)
        # gray = cv2.blur(gray,(2,2))
        # lines = cv2.HoughLinesP(gray, 3, np.pi / 180, 150, minLineLength, maxLineGap)
        # plt.imshow(self._roadmark_img)
        # plt.show()

        # cut mask from image
        height, width, channels = self._semantic_img.shape
        pts = np.array([[0,300],[200,150],[400,300]])
        mask = np.zeros((height, width, 3), np.uint8)
        cv2.drawContours(mask, [pts], -1, (255,255,255), -1, cv2.LINE_AA)
        for h in range(height):
            for w in range(width):
                if (self._semantic_img[h, w, :] == [50, 234, 157]).all() and (mask[h,w,:]==[255,255,255]).all():
                    self._roadmark_img[h,w,:] = [255,255,255]
                else:
                    self._roadmark_img[h,w,:] = [0,0,0]
        minLineLength = 3
        maxLineGap = 10
        
        # Hough Transformation
        gray = cv2.cvtColor(self._roadmark_img, cv2.COLOR_BGR2GRAY)
        gray = cv2.blur(gray,(2,2))
        lines = cv2.HoughLinesP(gray, 3, np.pi / 180, 80, minLineLength, maxLineGap)

        angles = self.calc_angles(lines)

        # if more than one line detected, make clusters
        if len(angles) > 1:
            # make angles positive
            pos_angles = [np.pi + v if v < 0 else v for v in angles]
            clusters = self.make_clusters(pos_angles)
             
            if clusters is not None:
                cluster_bounds = self.convert_clusters(clusters)
                lines_cluster = self.group_lines(cluster_bounds, lines)
                fangles, ypos = self.merge_cluster(lines_cluster)

                stop_line = False
                distance_to_stop = 0
                stopline_threshold = 0.05 * np.pi
                for idx, a in enumerate(fangles):
                    if abs(a) < stopline_threshold:
                        stop_line = True
                        distance_to_stop = (self.imgheight - ypos[idx]) / self.imgheight
                        break
                
                if stop_line:
                    self.stopline_publisher.publish(distance_to_stop)
                else:
                    self.stopline_publisher.publish(np.inf)


                #delta = self.compute_offset(fangles)
                #if delta:
                #    print(str(delta))

        try:
            im = self.bridge.cv2_to_imgmsg(self._roadmark_img)
            self.roadmark_publisher.publish(im)
        except CvBridgeError as e:
            print(e)

    def group_lines(self, cluster_bounds, lines):
        """
        Group lines by angles and clusters
        """
        lines_cluster = [[] for _ in range(len(cluster_bounds))]

        # colors for different clusters (only for debugging)
        colors = [(0, 145, 255), (239, 255, 0), (230, 0, 255), (0, 255, 17), (0, 255, 180), (70, 200, 100)] 

        for line in lines:
            x1, y1, x2, y2 = line[0][0], line[0][1], line[0][2], line[0][3]
            angle = np.arctan2(y2-y1,x2-x1)
            if angle < 0:
                angle += np.pi

            color = (0, 0, 255) # fallback color
            for idx, v in enumerate(cluster_bounds):
                if angle >= v[0] and angle <= v[1]:
                    # add line to cluster
                    lines_cluster[idx].append(line)
                    if idx > len(colors) -1:
                        color = (0, 255, 255)
                    else:
                        color = colors[idx]
                    break  

        return lines_cluster

    def merge_cluster(self, line_cluster):
        """
        Merge each cluster to one line
        """
        # colors for different clusters (only for debugging)
        colors = [(0, 145, 255), (239, 255, 0), (230, 0, 255), (0, 255, 17), (0, 255, 180), (70, 200, 100)] 

        fangles = []
        ypos = []
        for idx, line in enumerate(line_cluster):
            x1, y1, x2, y2 = line[0][0][0], line[0][0][1], line[0][0][2], line[0][0][3]
            if idx > len(colors) -1:
                color = (0, 255, 255)
            else:
                color = colors[idx]
            cv2.line(self._roadmark_img,(x1,y1),(x2,y2),color,3)
            angle = np.arctan2(y2-y1,x2-x1)
            if angle < 0:
                angle += np.pi
            fangles.append(angle)
            ystar = (y1 + y2) / 2
            ypos.append(ystar)
        return fangles, ypos

    def compute_offset(self, fangles):
        """
        Only if two lines detected, return value
        """
        if len(fangles) == 2:
            # compute offset
            delta = np.pi - fangles[0] -fangles[1]
            return delta
        else:
            return None

    def calc_angles(self, lines):
        """
        Calculate angles for lines.
        """
        angles = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0][0], line[0][1], line[0][2], line[0][3]
                angles.append(np.arctan2(y2-y1,x2-x1))
        return angles

    def draw_detected_lines(self, lines):
        """
        Draw lines into image.
        """
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0][0], line[0][1], line[0][2], line[0][3]
                cv2.line(self._roadmark_img,(x1,y1),(x2,y2),(0,255,0),1)
                cv2.circle(self._roadmark_img, (x1,y1), 1, (0,0,255), -1)
                cv2.circle(self._roadmark_img, (x2,y2), 1, (255,0,0), -1)

    def convert_clusters(self, clusters):
        """
        Convert each cluster to the form [min max].
        """
        cluster_bounds = []
        for c in clusters:
            c_max = np.max(c)
            c_min = np.min(c)
            c_bounds = [c_min, c_max]
            cluster_bounds.append(c_bounds)
        return cluster_bounds

    def make_clusters(self, angles):
        """
        Cluster angles using KernelDensityEstimation with bandwidth and resolution params.
        """
        if angles is None or len(angles) == 0:
            return 

        # calc density function
        a = array(angles).reshape(-1, 1)
        kde = KernelDensity(kernel='gaussian', bandwidth=self.cluster_bandwidth).fit(a)
        s = linspace(0, 2 * np.pi, self.cluster_resolution)
        e = kde.score_samples(s.reshape(-1,1))
        mi = argrelextrema(e, np.less)

        # only one cluster
        if mi[0].size == 0:
            return [angles]

        # go from minimum to minimum and aggregate clusters
        clusters = []
        clusters.append(a[a < s[mi][0]])
        for i in range(0, len(mi[0]) - 1):
            clusters.append(a[(a >= s[mi][i]) * (a <= s[mi][i+1])])
        clusters.append(a[a >= s[mi][len(mi[0]) - 1]])

        return clusters

    def semanticseg_updated(self, data):
        try:
            self._semantic_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def run(self):
        r = rospy.Rate(250)
        while not rospy.is_shutdown():
            self.run_step()
            try:
                r.sleep()
            except rospy.ROSInterruptException:
                pass

def main():
    rospy.init_node('lane_detection', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    lanedetection = LaneDetector(role_name)
    try:
        lanedetection.run()
    finally:
        del lanedetection

if __name__ == "__main__":
    main()

