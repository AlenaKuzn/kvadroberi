import math
from copy import copy

import tf_transformations
from simple_pid import PID
import tf2_geometry_msgs
import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Imu, LaserScan
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State, Altitude
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Vector3, Vector3Stamped, Quaternion, TransformStamped
from std_msgs.msg import Header
import statistics

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        bool1 = False
        self.RoomIdLog = [0]
        self.TargetRoom = "-1"
        self.FlightState = 2
        self.image = None
        self.cvbridge = CvBridge()
        self.qrDet = cv2.QRCodeDetector()
        self.imuData = None
        self.Altitude = None
        self.Scan = None
        self.Pose = PoseStamped()
        self.Speed = 1.0
        self.LastTime = self.get_clock().now().nanoseconds
        self.LastScan = 0
        self.StartPos = None
        self.VelVector = [self.Speed, 0.0, 0.0, 0.0]
        self.VectorStoryLen = 5#30
        self.VectorStory = []
        self.VectorYStoryLen = 15
        self.VectorYStory = []
        self.LastHeight = 1.5
        self.Entering = self.get_clock().now().nanoseconds
        self.LastRingSize = 0

        self.ZRPID = PID(3.0, 0.05, 1.0, setpoint=0)
        self.XPID = PID(0.00005, 0.0005, 0.005, setpoint=640)
        self.YPID = PID(1, 0.1, 0.05, setpoint=360)



        self.ZRPID.sample_time = 0.01
        self.XPID.sample_time = 0.01
        self.YPID.sample_time = 0.01

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )



        self.CamSub = self.create_subscription(Image, "/uav1/camera", self.getImage, 10)
        self.CamSubDown = self.create_subscription(Image, "/uav1/camera_down", self.getImageDown, 10)
        self.imu = self.create_subscription(Imu, "/uav1/mavros/imu/data", self.getImu, qos_profile)
        self.AltitudeSub = self.create_subscription(Altitude, "/uav1/mavros/altitude", self.getAltitude, qos_profile)
        self.LaserSub = self.create_subscription(LaserScan, "/uav1/scan", self.getScan, 10)
        self.local_pos_sub = self.create_subscription(PoseStamped, "/uav1/mavros/local_position/pose", self.getLocalPod, qos_profile)

        # Создаем клиента для установки режима
        self.set_mode_client = self.create_client(SetMode, '/uav1/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/uav1/mavros/cmd/arming')

        # Публишер для установки целевой позиции дрона (для взлета)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/uav1/mavros/setpoint_position/local', 10)
        self.local_velocity = self.create_publisher(TwistStamped, '/uav1/mavros/setpoint_velocity/cmd_vel', 10)

        # Подписываемся на топик состояния дрона (для проверки состояния)
        self.state_sub = self.create_subscription(State, '/uav1/mavros/state', self.state_cb, 10)

        # Текущее состояние дрона
        self.current_state = None
        self.RingCount = 0
        self.ArmAndTakeOff()

        pose1 = PoseStamped()
        pose1.header = Header()
        pose1.header.stamp = self.get_clock().now().to_msg()
        # Ус1танавливаем высоту взлета (например, 10 метров)
        pose1.pose.orientation.x = self.Pose.pose.orientation.x
        pose1.pose.orientation.y = self.Pose.pose.orientation.y
        pose1.pose.orientation.z = self.Pose.pose.orientation.z
        pose1.pose.orientation.w = self.Pose.pose.orientation.w
        pose1.pose.position.x = 0.0  # self.Pose.pose.position.x
        pose1.pose.position.y = 0.0  # self.Pose.pose.position.y
        pose1.pose.position.z = 2.0

        pose2 = PoseStamped()
        pose2.header = Header()
        pose2.header.stamp = self.get_clock().now().to_msg()
        # Ус2танавливаем высоту взлета (например, 10 метров)
        pose2.pose.orientation.x = self.Pose.pose.orientation.x
        pose2.pose.orientation.y = self.Pose.pose.orientation.y
        pose2.pose.orientation.z = self.Pose.pose.orientation.z
        pose2.pose.orientation.w = self.Pose.pose.orientation.w
        pose2.pose.position.x = 0.0  # self.Pose.pose.position.x
        pose2.pose.position.y = 0.0  # self.Pose.pose.position.y
        pose2.pose.position.z = 0.0

        rclpy.spin_once(self)
        while True:



            self.XLineControl()
            self.YTargetControl()
            self.DetectAndFlyAround(verbose=True)

            while self.CheckPose(pose1, delta=1) and self.RingCount > 0:
                print("Detected home")
                self.local_pos_pub.publish(pose2)
                #while self.CheckPose(pose1, delta=1.0) and self.RingCount > 0:
                #    print("Detected home2")
                #    self.setVelLocalRotate(0.0,0.0,-1.5, 0.0)
#

            #tm = np.array(self.image)
            #self.GUILidar(tm, self.Scan)
            #cv2.imshow("LO", tm)

            #self.VelVector[2] = self.AltitudeControl(self.Altitude.relative, 2)
            self.setVelLocalRotate(self.VelVector[0],self.VelVector[1], self.VelVector[2], self.VelVector[3])
            #print(self.VelVector)
            #LidarIm = np.array(self.image)
            #self.GUILidar(LidarIm, self.Scan)

            #cv2.imshow("1", LidarIm)
            cv2.waitKey(1)
            rclpy.spin_once(self)

    def CompensateLineForX(self, vector):
        temp = tf_transformations.euler_from_quaternion(
            [self.Pose.pose.orientation.x, self.Pose.pose.orientation.y, self.Pose.pose.orientation.z,
             self.Pose.pose.orientation.w])

        #temp1 = tf_transformations.quaternion_from_euler(temp[0], temp[1], temp[2] - math.pi / 2)

        vector[2] = vector[2] + temp[1] * math.sqrt(1280**2 + 720**2) / 1.74
        return vector

    def YTargetControl(self):
        imageFront = np.array(self.image)
        imageFront = cv2.cvtColor(imageFront, cv2.COLOR_BGR2RGB)
        cv2.imshow("Front2", imageFront)
        high = (65, 200 , 65)
        low = (0, 70, 0)

        mask = cv2.inRange(imageFront, low, high)

        edges = cv2.Canny(mask, 1, 255)
        counturs, hih = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        tempCont = []
        for contour in counturs:
            if cv2.contourArea(contour) > cv2.arcLength(contour, True):
                tempCont.append(contour)

        counturs = tempCont

        counturs = self.FilterConturs(counturs)
        if len(counturs) != 0:
            self.LastRingSize = cv2.contourArea(max(counturs, key=lambda x: len(x)))
            #print(self.LastRingSize)

        vector = self.FindDirVector(counturs)
        vector[0] = 0
        vector[1] = 0
        edges = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        edges = cv2.drawContours(edges, self.Vector2Contur(vector=vector), -1, (255, 0, 255),
                                 3)
        edges = cv2.drawContours(edges, counturs, -1, (0, 255, 0), 3)

        if vector[3] != 0 and self.LastRingSize > 3000:
            self.VectorYStory.append(vector)

            while len(self.VectorYStory) >= self.VectorYStoryLen:
                self.VectorYStory.pop(0)

            # vector = [sum(self.VectorStory, key=lambda ) / len(self.VectorStory)]

            x, y, z, w = 0, 0, 0, 0

            for i in self.VectorYStory:
                x += i[0]
                y += i[1]
                z += i[2]
                w += i[3]
            vector = [x / len(self.VectorYStory), y / len(self.VectorYStory), z / len(self.VectorYStory),
                      w / len(self.VectorYStory)]



            self.VelVector[2] = self.AltitudeControl(statistics.median([x[3] for x in self.VectorYStory]), 360, sensitivity=-0.01)
            print("Detected ring: " + str(self.LastRingSize))
            self.LastHeight = self.Altitude.relative
            if self.LastHeight < 1:
                self.LastHeight = 1.0
            self.Entering = self.get_clock().now().nanoseconds
        else:
            self.VelVector[2] = self.AltitudeControl(self.Altitude.relative, self.LastHeight)
            while self.Entering + 2700000000 > self.get_clock().now().nanoseconds and self.LastRingSize > 170000:
                self.setVelLocalRotate(self.VelVector[0],self.VelVector[1], 0, 0)
                #self.DetectAndFlyAround(verbose=True)
                self.RingCount += 1
                print("Ring count: " + str(self.RingCount))
            self.LastRingSize = 0
            #self.Altitude.relative, self.LastHeight)
            #self.LastHeight += (2 - self.LastHeight) * 0.5

        if min(self.Scan.ranges[135:225]) < 4:
            self.VelVector[1] = (self.VelVector[1] + (-(640 - vector[2])) / 500) / 2#(-(640 - vector[2])) / 600#  # -self.XPID(vector[2][0])
            if self.VelVector[1] > 2.0:
                self.VelVector[1] = 2.0
            elif self.VelVector[1] < -2.0:
                self.VelVector[1] = -2.0

        #print(self.LastHeight)
        cv2.imshow("FrontMask", edges)


    def DetectAndFlyAround(self, detDist=2.5, delta=1, verbose=False):

        #imageFront = np.array(self.image)
#
        #higher = (255, 255, 255)
        #lower = (70, 70, 70)
        #imageFront = cv2.inRange(imageFront, lower, higher)
        ##t = cv2.Canny(imageFront, 200, 255)
#
        #cv2.imshow("FrontStolb", imageFront)
        start = 0
        delta = 0.3
        distToRing = 0
        #for i in range(1, len(self.Scan.ranges)):
        #    if abs(self.Scan.ranges[i] - self.Scan.ranges[i-1]) > delta:
        #        if start == 0:
        #            start = i
        #        else:

        if self.LastRingSize < 50000:
            if sum(1 for i in self.Scan.ranges[90:180] if detDist <= i <= detDist + delta * 2) == 2:
                self.LastTime = self.get_clock().now().nanoseconds
                print("Detected ring via Lidar")
            if not (self.LastTime + 4000000000> self.get_clock().now().nanoseconds)and sum(1 for i in self.Scan.ranges[120:300] if i<=detDist+delta) > 10:
                vector = self.GetLineVector()

                if self.Scan.ranges.index(min(self.Scan.ranges)) > 180:
                    self.VelVector[1] = (-(640 - vector[2] - 350)) / 500
                else:
                    self.VelVector[1] = (-(640 - vector[2] + 350)) / 500

                #self.VelVector[1] = (-(640 - vector[2] - 350)) / 500  # -self.XPID(vector[2][0])
                if self.VelVector[1] > 1.5:
                    self.VelVector[1] = 1.5
                elif self.VelVector[1] < -1.5:
                    self.VelVector[1] = -1.5    #-self.XPID(vector[2] - 240)# * detDist / min(self.Scan.ranges))
                self.VelVector[2] = self.AltitudeControl(self.Altitude.relative, 1.5)
                self.LastHeight = 1.5

                #if True or vector:
                #    if (-(640 - 240 * detDist / min(self.Scan.ranges) - vector[2][0])) / 500 > 1.5: #self.MinusPower((-(400 - vector[2][0])) / 250, 2) > 3:
                #        self.VelVector[1] = 1.5
                #    elif (-(640 - 240 * detDist / min(self.Scan.ranges) - vector[2][0])) / 500 < -1.5: #self.MinusPower((-(400 - vector[2][0])) / 250, 2) < -3:
                #        self.VelVector[1] = -1.5
                #    else:
                #        self.VelVector[1] = (-((640 - 240 * detDist / min(self.Scan.ranges)) - vector[2][0])) / 500#self.MinusPower((-(400 - vector[2][0])) / 250, 2)
                if verbose:
                    print("Detected obstacle: " + str(self.VelVector[1]) + " / " + str(detDist / min(self.Scan.ranges)))
                return True
            else:
                return False
        else:
            return False


    def XLineControl(self, show=True):
        vector = self.GetLineVector()
        if True or vector:
            #edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)
            #edges = cv2.drawContours(edges, self.Vector2Contur(vector=self.FindDirVector(counturs)), -1, (255, 255, 0),
            #                         3)
            #edges = cv2.drawContours(edges, counturs, -1, (255, 0, 255), 3)

            #cv2.imshow("Down", edges)
            if  vector[1] != 0:
                angle = math.atan(vector[0] / vector[1])
            else:
                angle = 0

            #self.VelVector[3] = self.ZRPID(angle)
            self.VelVector[3] = -self.ZRPID(angle)
            if self.VelVector[3] > 1.5: #self.MinusPower(angle, 2) * 5 > 1.5:
                self.VelVector[3] = 1.5
            elif self.VelVector[3] < -1.5: #self.MinusPower(angle, 2) * 5 < -1.5:
                self.VelVector[3] = -1.5


                # * se  lf.VelVector[0] #(angle ** (2-1)) * abs(angle) * 10
        # print(self.VelVector[3])
            self.VelVector[1] = -(640 - vector[2]) / 600 #(-(640 - vector[2]))/500 #-self.XPID(vector[2][0])
            if  self.VelVector[1] > 1.5:
                self.VelVector[1] = 1.5
            elif self.VelVector[1] < -1.5:
                self.VelVector[1] = -1.5
            #print(self.VelVector[1])
            #self.setVelLocalRotate(self.VelVector[0], self.VelVector[1], self.AltitudeControl(self.Altitude.relative, 2),
            #                       self.VelVector[3])
        """imageDown = copy(self.imageDown)

        imageDown = cv2.cvtColor(imageDown, cv2.COLOR_BGR2GRAY)
        vel, mask = cv2.threshold(imageDown, 0.5, 7, cv2.THRESH_BINARY)

        mask[mask == 0] = 255

        edges = cv2.Canny(mask, 1, 255)
        counturs, hih = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(counturs) != 0:
            vector = self.FindDirVector(counturs)

            edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)
            edges = cv2.drawContours(edges, self.Vector2Contur(vector=self.FindDirVector(counturs)), -1, (255, 255, 0),
                                     3)
            edges = cv2.drawContours(edges, counturs, -1, (255, 0, 255), 3)
            if show:
                cv2.imshow("Down", edges)
            
            angle = math.atan(vector[0] / vector[1])

            if self.MinusPower(angle, 2) * 10 > 1.5:
                self.VelVector[1] = 1.5
            elif self.MinusPower(angle, 2) * 10 < -1.5:
                self.VelVector[1] = -1.5
            else:
                self.VelVector[1] = self.MinusPower(angle, 2) * 10
                # * se  lf.VelVector[0] #(angle ** (2-1)) * abs(angle) * 10
            
            if self.MinusPower((-(640 - vector[2][0])) / 250, 2) > 3:
                self.VelVector[3] = 3
            elif self.MinusPower((-(640 - vector[2][0])) / 250, 2) < -3:
                self.VelVector[3] = -3
            else:
                self.VelVector[1] = self.MinusPower((-(640 - vector[2][0])) / 250, 2) #* self.VelVector[0] / 500
        else:
            return False"""

    def GetLineVector(self, show=True):
        imageDown = copy(self.imageDown)

        imageDown = cv2.cvtColor(imageDown, cv2.COLOR_BGR2GRAY)
        vel, mask = cv2.threshold(imageDown, 0.5, 7, cv2.THRESH_BINARY)

        mask[mask == 0] = 255

        edges = cv2.Canny(mask, 1, 255)
        counturs, hih = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        counturs = self.FilterConturs(counturs)

        vector = self.CompensateLineForX(self.FindDirVector(counturs))
        #vector = [vector[0][0], vector[1][0], vector[2][0], vector[3][0]]
        if vector[3] !=0:
            self.VectorStory.append(vector)

        while len(self.VectorStory) >= self.VectorStoryLen:
            self.VectorStory.pop(0)

        #vector = [sum(self.VectorStory, key=lambda ) / len(self.VectorStory)]

        x, y, z, w = 0,0,0,0

        for i in self.VectorStory:
            x += i[0]
            y += i[1]
            z += i[2]
            w += i[3]
        vector = [x / len(self.VectorStory), y / len(self.VectorStory), z / len(self.VectorStory), w / len(self.VectorStory)]

        if show:
            edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)
            edges = cv2.drawContours(edges, self.Vector2Contur(vector=vector), -1, (255, 255, 0),
                                     3)
            edges = cv2.drawContours(edges, counturs, -1, (255, 0, 255), 3)
            cv2.imshow("LineDetection", edges)
        return vector



    def FilterConturs(self, Conturs, minCont=100):
        ans = []
        for contur in Conturs:
            if len(contur) >= minCont:
                ans.append(contur)
        return ans

    def MinusPower(self, Float, Power):
        if Float < 0 and Float ** Power > 0:
            return -(Float ** Power)
        else:
            return Float ** Power

    def Vector2Contur(self, vector):

        return np.array([[[int(vector[2] - vector[0] * 200), int(vector[3] - vector[1] * 200)],[int(vector[2] + vector[0] * 200), int(vector[3] + vector[1] * 200)]]])

    def FindDirVector(self, Conturs):
        if len(Conturs):
            maxCont = max(Conturs, key=lambda x: len(x))
            vector = cv2.fitLine(maxCont, cv2.DIST_L2, 0, 0.01, 0.01)
            vector = [vector[0][0], vector[1][0], vector[2][0], vector[3][0]]
            return vector
        else:
            return [0,0,0,0]

    def LenTodot(self, dot1, dot2):
        temp = [0, 0]
        temp[0] = dot2[0] - dot1[0]
        temp[1] = dot2[1] - dot1[1]
        return math.sqrt(temp[0] * temp[0] + temp[1] * temp[1])

    def IntersectVectorAndSurface(self, Surface, Vector, Platform=(0,0,0)):
        A, B, C, D = Surface
        m, n, p = Vector
        Origin = self.Pose.pose.position.x, self.Pose.pose.position.y, self.Pose.pose.position.z
        x1, y1, z1 = Origin

        try:
            t = -(A * x1 + B * y1 + C * z1 + D)/(A*m + B*n + C*p)
        except:
            t = 0

        x = x1 + m*t
        y = y1 + n*t
        z = z1 + p*t

        #xCorrection = (10/math.sin(x))*0.0 #0.00005#( Platform[0]**2)* 7
        #yCorrection = (10/math.sin(y))*0.005

        xCorrection = ((x**2)*7+10)*0.005 #0.00005#( Platform[0]**2)* 7
        yCorrection = ((y**2)*7+10)*0.005

        #xCal = ((x**2)*7+10)*xCorrection + x
        #yCal = ((y**2)*7+10)*yCorrection + y

        xCal = xCorrection + x
        yCal = yCorrection + y

        return [xCal, yCal, z]

    def SearchDegree(self, x, y):
        FoV = 1.74 / 2
        try:
            turnZangle = math.atan(abs(y - 360) / abs(x - 640))
        except:
            turnZangle = 0
        if (y - 360 >= 0):
            turnZangle += 90 * math.pi / 180
        if (x - 640 < 0):
            turnZangle *= -1
        diagonalDeg = FoV / math.sqrt(640 ** 2 + 360 ** 2)
        turnYangle = -math.sqrt((x - 640) ** 2 + (y - 360) ** 2) * diagonalDeg
        #print(str(turnZangle) + " " + str(turnYangle))
        ans = [0, 0, 1]

        # Z:
        # |cos θ   −sin θ   0| |x|   |x cos θ − y sin θ|   |x'|
        # |sin θ    cos θ   0| |y| = |x sin θ + y cos θ| = |y'|
        # |  0       0      1| |z|   |        z        |   |z'|

        #Y:
        # | cos θ    0   sin θ| |x|   | x cos θ + z sin θ|   |x'|
        # |   0      1       0| |y| = |         y        | = |y'|
        # |−sin θ    0   cos θ| |z|   |−x sin θ + z cos θ|   |z'|

        #X:
        # |1     0           0| |x|   |        x        |   |x'|
        # |0   cos θ    −sin θ| |y| = |y cos θ − z sin θ| = |y'|
        # |0   sin θ     cos θ| |z|   |y sin θ + z cos θ|   |z'|

        ans[0] = ans[0] * math.cos(turnYangle) + ans[2] * math.sin(turnYangle)
        ans[2] = -ans[0] * math.sin(turnYangle) + ans[2] * math.cos(turnYangle)

        ans[0] = ans[0] * math.cos(turnZangle) - ans[1] * math.sin(turnZangle)
        ans[1] = ans[0] * math.sin(turnZangle) + ans[1] * math.cos(turnZangle)

        return ans

    def PixelToVector(self, x, y, maxX=1280, maxY=720):
        ans = [0, 0, 1]
        ans2 = [0, 0, 1]

        #temp=x
        #x=y
        #y=temp
        # |cos θ   −sin θ   0| |x|   |x cos θ − y sin θ|   |x'|
        # Z:#|sin θ    cos θ   0| |y| = |x sin θ + y cos θ| = |y'|
        # |  0       0      1| |z|   |        z        |   |z'|

        # ```| cos θ    0   sin θ| |x|   | x cos θ + z sin θ|   |x'|
        # Y:#|   0      1       0| |y| = |         y        | = |y'|
        # ```|−sin θ    0   cos θ| |z|   |−x sin θ + z cos θ|   |z'|

        # |1     0           0| |x|   |        x        |   |x'|
        # X:#|0   cos θ    −sin θ| |y| = |y cos θ − z sin θ| = |y'|
        # |0   sin θ     cos θ| |z|   |y sin θ + z cos θ|   |z'|

        turnYangle = (y - maxY / 2) * 1.74
        turnZangle = (x - maxX / 2) * 1.74

        pixelperrad = 1.74 / math.sqrt(maxX * maxX + maxY * maxY)

        turnYangle = ((x - maxX/2)) * pixelperrad
        #print(f"tYangle {turnYangle*180/math.pi}")
        turnXangle = -((y - maxY/2)) * pixelperrad
        #print(f"tXangle {turnXangle*180/math.pi}")
        alpha = turnXangle
        beta = turnYangle

        # y trans
        ans[0] = ans[0] * math.cos(turnYangle) + ans[2] * math.sin(turnYangle)
        ans[2] = -ans[0] * math.sin(turnYangle) + ans[2] * math.cos(turnYangle)
        # x trans
        ans[1] = ans[1] * math.cos(turnXangle) - ans[2] * math.sin(turnXangle)
        ans[2] = ans[1] * math.sin(turnXangle) + ans[2] * math.cos(turnXangle)
        # z trans
        # ans[0] = ans[0] * math.cos(turnZangle) - ans[1] * math.sin(turnZangle)
        # ans[1] = ans[0] * math.sin(turnZangle) + ans[1] * math.cos(turnZangle)

        #x = math.cos(alpha) * math.cos(beta)
        #z = math.sin(alpha) * math.cos(beta)
        #y = math.sin(beta)

        #xCorrection = (10 / math.sin(ans[0])) * 0.0  # 0.00005#( Platform[0]**2)* 7
        #yCorrection = (10 / math.sin(ans[1])) * 0.005

        return ans#[x, y, z]  #, ans2

    def TryXnY(self, x, y):
        ans = self.PixelToVector(x, y)
        xPos = self.IntersectVectorAndSurface([0, 0, 1, 0], ans, Origin=[self.Pose.pose.position.x, self.Pose.pose.position.y, self.Pose.pose.position.z])
        yPos = self.IntersectVectorAndSurface([0, 0, 1, 0], ans, Origin=[self.Pose.pose.position.x, self.Pose.pose.position.y,
                                                                   self.Pose.pose.position.z])
        return [xPos, yPos]

    def FilterCounturs(self, counturs):
        temp = []
        for item in counturs:
            if len(item) > 10:
                temp.append(item)
        return temp



    def FindSpots(self, image):
        original_image = image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(image, 180, 200)
        #cv2.imshow("Gray", gray)
        cv2.imshow("Edges", edges)


        contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=False)

        ans = []
        count = 0
        sumx = 0
        sumy = 0

        sorted_contours = self.FilterCounturs(sorted_contours)

        maxCont = [[(0, 0)]]

        for item in sorted_contours:
            if len(maxCont) < len(item):
                maxCont = item

            for cont in item:
                x, y = cont[0]
                sumx += x
                sumy += y
                count += 1
            ans.append((int(sumx/count), int(sumy/count)))
            count = 0
            sumx = 0
            sumy = 0
        #print(ans)
        count = 0
        sumx = 0
        sumy = 0

        for cont in maxCont:
            x, y = cont[0]
            sumx += x
            sumy += y
            count += 1
        return ans, (int(sumx / count), int(sumy / count))
        #print(sorted_contours)

        # largest item
        """M = cv2.moments(max(sorted_contours))
        x, y, w, h = cv2.boundingRect(max(sorted_contours))

        xcoordinate1 = x
        xcoordinate2 = x + w
        #xcoordinate_center = int(M['m10'] / M['m00'])
        print("Larger Box")
        print("x coordinate 1: ", str(xcoordinate1))
        print("x coordinate 2: ", str(xcoordinate2))
        #print("x center coordinate ", str(xcoordinate_center))
        print("")

        ycoordinate1 = y
        ycoordinate2 = y + h
        #ycoordinate_center = int(M['m01'] / M['m00'])

        print("y coordinate 1: ", str(ycoordinate1))
        print("y coordinate 2: ", str(ycoordinate2))
        #print("y center coordinate ", str(ycoordinate_center))

        print("")# largest
        M2= cv2.moments(min(sorted_contours))
        x2, y2, w2, h2 = cv2.boundingRect(min(sorted_contours))

        x2coordinate1 = x2
        x2coordinate2 = x2 + w2
        #x2coordinate_center = int(M2['m10'] / M2['m00'])
        print("Smaller Box")
        print("x coordinate 1: ", str(x2coordinate1))
        print("x coordinate 2: ", str(x2coordinate2))
        #print("x center coordinate ", str(x2coordinate_center))
        print("")

        y2coordinate1 = y2
        y2coordinate2 = y2 + h2
        #y2coordinate_center = int(M2['m01'] / M2['m00'])

        print("y coordinate 1: ", str(y2coordinate1))
        print("y coordinate 2: ", str(y2coordinate2))
        #print("y center coordinate ", str(y2coordinate_center))"""
        #plt.imshow(image)
        #plt.show()


    def DefineActions(self):
        #Takeoff?1, Roaming2, aligning3, entering4
        cv2.imshow("CameraDown", self.imageDown)
        cv2.imshow("Camera", self.image)
        #image2 = copy(self.imageDown)
        #self.GUILidar(image2, self.Scan)
        #cv2.imshow("Lidar", image2)

        if cv2.waitKey(1) == ord('q'):
            pass
        #print(self.Scan.ranges[(int)(self.FindForwardAngle())])
        if self.Scan.ranges[(self.FindForwardAngle())] < 2: #and self.Scan.ranges[(self.FindForwardAngle())] != self.LastScan:
            #print(self.Scan.ranges[(int)(self.FindForwardAngle())]
            self.setRotation()
        elif self.TargetRoom != "-1" and abs(self.Altitude.relative - 2) > 0.3:
            self.setVelLocal(0.0, 0.0, self.AltitudeControl(self.Altitude.relative, 2.3))
        else:
            #print(self.Scan.ranges[(int)(self.FindForwardAngle())])
            if self.TargetRoom == "-1": #search for TargetID
                data, bbox, _ = self.qrDet.detectAndDecode(self.imageDown)
                if len(data) > 0 and data != self.RoomIdLog[-1]:
                    self.TargetRoom = data
                    self.RoomIdLog.append(data)
                    print("Found Target: " + data)
                else:
                    #search snd roam for targetID
                    self.setVelLocal(self.Speed, 0, self.AltitudeControl(self.Altitude.relative, 1.5))
                    pass
            else:
                #search and roam for door
                data, bbox, _ = self.qrDet.detectAndDecode(self.image)
                # enter room
                if len(data) > 0:
                    print(f"Found Room: {data}. Flightstase: {self.FlightState}, TargetRoom: {self.TargetRoom}")
                    if data == self.TargetRoom and self.FlightState != 4:
                        if self.FlightState != 3:
                            self.FlightState = 3
                        velvector = self.GetCenterQRVector(bbox)
                        print("Aligning...") #"Found Room: " + data + ". Aligning..."
                        if self.CheckCenterQR(bbox):
                            self.FlightState = 4
                            self.TargetRoom = "-1"
                            velvector.linear.x = 0.3
                            print("entering...")

                        self.setVelLocalRotate(velvector.linear.x, velvector.linear.y,
                                         self.AltitudeControl(self.Altitude.relative, 2.3), velvector.angular.z)
                        #+ self.AltitudeControl(self.Altitude.relative, 2))
                    elif data != self.TargetRoom != "-1":
                        #wrong door, turn
                        pass
                        self.setRotation()
                else:
                    self.FlightState = 2
                    self.setVelLocal(self.Speed, 0.0,
                                     self.AltitudeControl(self.Altitude.relative, 2.3))

    def setVel(self, x, y, z):
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        temptwist = Twist()
        temptwist.linear.x = x
        temptwist.linear.y = y
        temptwist.linear.z = z
        temptwist.angular.z = 0.0
        msg.twist = temptwist
        msg.header.frame_id = 'FRAME_BODY_NED'
        self.local_velocity.publish(msg)

    def setRotation(self):
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        print("Turning...")

        if self.LastTime.nanoseconds + 4000 >= self.get_clock().now().nanoseconds or self.Scan.ranges[(self.FindForwardAngle())] == self.LastScan:
            return
        print(self.Scan.ranges[(self.FindForwardAngle())])
        # Устанавливаем высоту взлета (например, 10 метров)
        pose.pose.position = self.Pose.pose.position
        #pose.pose.orientation.z -= math.pi/2
        temp = tf_transformations.euler_from_quaternion([self.Pose.pose.orientation.x, self.Pose.pose.orientation.y, self.Pose.pose.orientation.z, self.Pose.pose.orientation.w])

        temp1 = tf_transformations.quaternion_from_euler(temp[0], temp[1], temp[2] - math.pi/2)
        pose.pose.orientation.x = temp1[0]
        pose.pose.orientation.y = temp1[1]
        pose.pose.orientation.z = temp1[2]
        pose.pose.orientation.w = temp1[3]
        # Публикуем целевую позицию несколько раз (необходимо для стабильной работы OFFBOARD режима)
        #for _ in range(0,10):
        self.local_pos_pub.publish(pose)

        self.LastScan = self.Scan.ranges[(self.FindForwardAngle())]
        self.LastTime = self.get_clock().now()
            #print(temp)
            #rclpy.spin_once(self)

    def setVelLocal(self, x, y, z):
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        temptwist = Twist()

        vector_local = np.array([x, y, z])  # Example vector in local coordinates
        quaternion = np.array(np.array([self.imuData.orientation.x, self.imuData.orientation.y, self.imuData.orientation.z, self.imuData.orientation.w]))

        ans = self.transform_vector_local_to_global(vector_local, quaternion)

        temptwist.linear.x = ans[0]#*-1
        temptwist.linear.y = ans[1]#*-1
        temptwist.linear.z = ans[2]#*-1
        #temptwist.angular.z = 7.0
        #msg.twist = self.TransformToLocal(temptwist, self.imuData.orientation)
        msg.twist = temptwist
        msg.header.frame_id = 'FRAME_BODY_NED'
        #print(f"Got x: {x} y: {y} z:{z}")
        #print(f"returned x: {msg.twist.linear.x} y: {msg.twist.linear.y} z:{msg.twist.linear.z}")
        self.local_velocity.publish(msg)

    def setVelLocalRotate(self, x, y, z, zrotate):
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        temptwist = Twist()

        vector_local = np.array([x, y, z])  # Example vector in local coordinates
        quaternion = np.array(np.array([self.imuData.orientation.x, self.imuData.orientation.y, self.imuData.orientation.z, self.imuData.orientation.w]))

        ans = self.transform_vector_local_to_global(vector_local, quaternion)

        temptwist.linear.x = ans[0]#*-1
        temptwist.linear.y = ans[1]#*-1
        temptwist.linear.z = ans[2]#*-1
        temptwist.angular.z = float(zrotate)
        #msg.twist = self.TransformToLocal(temptwist, self.imuData.orientation)
        msg.twist = temptwist
        msg.header.frame_id = 'FRAME_BODY_NED'
        #print(f"Got x: {x} y: {y} z:{z}")
        #print(f"returned x: {msg.twist.linear.x} y: {msg.twist.linear.y} z:{msg.twist.linear.z}")
        self.local_velocity.publish(msg)

    def getScan(self, msg):
        self.Scan = msg

    def getLocalPod(self, msg):
        self.Pose = msg

    def getImage(self, msg):
        self.image = self.cvbridge.imgmsg_to_cv2(msg)

    def getAltitude(self, msg):
        self.Altitude = msg

    def getImageDown(self, msg):
        self.imageDown = self.cvbridge.imgmsg_to_cv2(msg)

    def getImu(self, msg):
        self.imuData = msg

    def AltitudeControl(self, altitude, target, sensitivity=-1.5):
        #print("altitude: " + str(altitude) + ", applaying: " + str((altitude - target)*-sensitivity) + ". Next room: " + self.TargetRoom)
        return (altitude - target)*-sensitivity

    def FindForwardAngle(self):
        temp = tf_transformations.euler_from_quaternion([self.Pose.pose.orientation.x, self.Pose.pose.orientation.y, self.Pose.pose.orientation.z, self.Pose.pose.orientation.w])
        #print(temp*math.pi) #+ " - " + str(self.Scan.ranges[int(temp[2]/math.pi*180 + 180)]))
        return 180#int(temp[2]*180/math.pi + 180)

    def TransformToLocal(self, Twist, Quat):
        print(f"HUI: {Quat.x}, {Quat.y} ,{Quat.z} ,{Quat.w}")

        Quattemp = (Quat.x, Quat.y, Quat.z, Quat.w)
        #temp = tf.quaternion_inverse(Quattemp)
        #transtemp = TransformStamped()
        #transtemp.transform.rotation.x = temp[0]
        #transtemp.transform.rotation.y = temp[1]
        #transtemp.transform.rotation.z = temp[2]
        #transtemp.transform.rotation.w = temp[3]

        transtemp = TransformStamped()
        transtemp.transform.rotation.x = Quat.x
        transtemp.transform.rotation.y = Quat.y
        transtemp.transform.rotation.z = Quat.z
        transtemp.transform.rotation.w = Quat.w

        v = Vector3Stamped()
        v.vector.x = Twist.linear.x
        v.vector.y = Twist.linear.y
        v.vector.z = Twist.linear.z
        tf2_geometry_msgs.do_transform_vector3()
        v1 = tf2_geometry_msgs.do_transform_vector3(v, transtemp)
        Twist.linear.x = v1.vector.x
        Twist.linear.y = v1.vector.y
        Twist.linear.z = v1.vector.z
        return Twist

    def transform_vector_global_to_local(self, vector, quaternion):
        """Transforms a vector from global to local coordinates using a quaternion."""

        def quaternion_conjugate(q):
            return np.array([q[0], -q[1], -q[2], -q[3]])

        def quaternion_multiply(q1, q2):
            w1, x1, y1, z1 = q1
            w2, x2, y2, z2 = q2
            return np.array([
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
            ])

        # Convert vector to a quaternion with a 0 scalar part
        vector_q = np.array([0] + list(vector))
        # Rotate vector using quaternion and its conjugate
        rotated_q = quaternion_multiply(quaternion_multiply(quaternion_conjugate(quaternion), vector_q), quaternion)
        # Return only the vector part of the quaternion
        return rotated_q[1:]

    def transform_vector_local_to_global(self, vector, quaternion):
        """Transforms a vector from local to global coordinates using a quaternion."""
        vector = np.array(list(vector)[::-1])
        def quaternion_multiply(q1, q2):
            w1, x1, y1, z1 = q1
            w2, x2, y2, z2 = q2
            return np.array([
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
            ])

        def quaternion_conjugate(q):
            return np.array([q[0], -q[1], -q[2], -q[3]])

        # Convert vector to a quaternion with a 0 scalar part
        vector_q = np.array([0] + list(vector))
        # Rotate vector by applying quaternion and its conjugate
        rotated_q = quaternion_multiply(quaternion_multiply(quaternion, vector_q), quaternion_conjugate(quaternion))
        # Return only the vector part of the quaternion
        return (rotated_q[1:])[::-1]

    def checkQR(self, image, centered=False):
        data, bbox, rectifiedImage = self.qrDet.detectAndDecode(image)
        if centered and len(data) > 0:
            print(bbox[0][0])
            if 700 > bbox[0][0][0] > 500 and 400 > bbox[0][0][1] > 300:
                return True
            else:
                return False
        if len(data) > 0:
            return True
        else:
            return False

    def QRBoxed(self, image, box):
        n = len(box)
        for j in range(n):
            cv2.line(image, tuple(box[j][0]), tuple(box[ (j+1) % n][0]), (255.0,0), 3)

    def GUILidar(self, image, msg, zoom=100):
        count = 0
        noinf = [msg.ranges[i] if (msg.ranges[i] * zoom * math.cos(i * 0.01749303564429283) + msg.ranges[i] * zoom * math.sin(i * 0.01749303564429283)) != math.inf else 50 for i in range(0, 360)]
        for i in range(0, 360):
            if noinf[i] < 10:
                count+=1
            if i == self.FindForwardAngle():
                try:
                    cv2.line(image, (640, 360), ((int)(640 + noinf[i] * zoom * math.cos(i * 0.01749303564429283)),
                                                 (int)(360 + noinf[i] * zoom * math.sin(i * 0.01749303564429283))),
                             (0, 125, 125), 3)  # rad*sin phi, rad*cos phi
                except:
                    pass
            else:
                try:
                    cv2.line(image, (640, 360), ((int)(640 + noinf[i] * zoom * math.cos(i * 0.01749303564429283)),
                                                 (int)(360 + noinf[i] * zoom * math.sin(i * 0.01749303564429283))),
                             (i / 2, 0, 0), 3)  # rad*sin phi, rad*cos phi
                except:
                    pass
        #print(count)

    def GetCenterQRVector(self, bbox, targetx=650, targety=150):
        x = bbox[0][2][0] - bbox[0][0][0]
        x = bbox[0][0][0] + x/2

        y = bbox[0][2][1] - bbox[0][0][1]
        y = bbox[0][0][1] + y/2

        temptwist = Twist()
        temptwist.linear.x = 0.0
        temptwist.linear.y = -(float)((targetx - x)/1000)
        temptwist.linear.z = (float)((targety - y)/500)

        print("Adjasting turn: " + str((float)(bbox[0][0][1] - bbox[0][1][1])/70))
        temptwist.angular.z = -(float)(bbox[0][0][1] - bbox[0][1][1])/70

        #if temptwist.angular.z > 1:
        #   temptwist.angular.z = 0.8

        return temptwist

    def CheckCenterQR(self, bbox, targetx=650, targety=150, delta=50):
        x = bbox[0][2][0] - bbox[0][0][0]
        x = bbox[0][0][0] + x / 2

        y = bbox[0][2][1] - bbox[0][0][1]
        y = bbox[0][0][1] + y / 2

        #if abs(targety - y) <= 50 and abs(targetx - x) <= 50:
        if abs(targetx - x) <= delta and abs((float)(bbox[0][0][1] - bbox[0][1][1])) < 10:
            return True
        else:
            return False

    def ArmAndTakeOff(self):
        if self.arming_client.wait_for_service(timeout_sec=5.0):
            arm_req = CommandBool.Request()
            arm_req.value = True
            future = self.arming_client.call_async(arm_req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() and future.result().success:
                self.get_logger().info('Drone armed')
            else:
                #self.ArmAndTakeOff()
                self.get_logger().error('Failed to arm drone')
        else:
            #self.ArmAndTakeOff()
            self.get_logger().error('Arming service not available')

        if self.set_mode_client.wait_for_service(timeout_sec=5.0):
            set_mode_req = SetMode.Request()
            set_mode_req.custom_mode = 'OFFBOARD'
            future = self.set_mode_client.call_async(set_mode_req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() and future.result().mode_sent:
                self.get_logger().info('OFFBOARD mode set')
            else:
                #self.ArmAndTakeOff()
                self.get_logger().error('Failed to set OFFBOARD mode')
        else:
            #self.ArmAndTakeOff()
            self.get_logger().error('Set mode service not available')

        self.get_logger().info('Taking off...')

        # Создаем сообщение для взлета (целевое положение)
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()

        # Устанавливаем высоту взлета (например, 10 метров)
        pose.pose.orientation.x = self.Pose.pose.orientation.x
        pose.pose.orientation.y = self.Pose.pose.orientation.y
        pose.pose.orientation.z = self.Pose.pose.orientation.z
        pose.pose.orientation.w = self.Pose.pose.orientation.w
        pose.pose.position.x = 0.0 #self.Pose.pose.position.x
        pose.pose.position.y = -1.0 #self.Pose.pose.position.y
        pose.pose.position.z += 3.0
        #rclpy.spin_once(self)
        #rclpy.spin_once(self)
        #rclpy.spin_once(self)
        while not self.CheckPose(pose, delta=0.3):
            print(self.Pose)
            self.local_pos_pub.publish(pose)
            rclpy.spin_once(self)

        # Публикуем целевую позицию несколько раз (необходимо для стабильной работы OFFBOARD режима)
        self.get_logger().info('Drone at height')

    def CheckPose(self, PoseStamped, delta=0.1):
        if not (PoseStamped.pose.position.x + delta > self.Pose.pose.position.x > PoseStamped.pose.position.x - delta):
            return False
        if not (PoseStamped.pose.position.y + delta > self.Pose.pose.position.y > PoseStamped.pose.position.y - delta):
            return False
        if not (PoseStamped.pose.position.z + delta > self.Pose.pose.position.z > PoseStamped.pose.position.z - delta):
            return False
        return True

    def state_cb(self, msg):
        self.current_state = msg


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()