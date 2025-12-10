#! /usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import math

from PyQt5.QtCore import Qt, QTimer, QRectF, QPointF, QPoint, QSize, QLineF, QRect
from PyQt5.QtWidgets import QApplication, QDialog, QGraphicsScene
from PyQt5.QtGui import QPen, QBrush, QColor, QPolygonF, QPixmap

from path_visualization_window import Ui_Form

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Controller_GraphicsScene(QGraphicsScene):

    pressed = False

    def __init__(self, parent=None):
        W, H = 150, 150
        self.Cx, self.Cy = int(W/2), int(H/2)
        QGraphicsScene.__init__(self, 0, 0, W, H, parent = None) 
        self.opt = ""

    def setOption(self, opt):
        self.opt = opt

    def mouseReleaseEvent(self,event):
        self.pressed = False
        window.repaint_controller(self.Cx, self.Cy)
        window.cmd_vel.linear.x = 0.0
        window.cmd_vel.angular.z = 0.0

    def mousePressEvent(self,event):
        self.pressed = True

    def mouseMoveEvent(self,event):
        if(self.pressed == True):
            x = event.scenePos().x()
            y = event.scenePos().y()
            if(((x - self.Cx)**2 + (y - self.Cy)**2) < 60**2):
                
                linear_speed = (self.Cy - y) / 60.0
                angular_speed = (self.Cx - x) / 60.0

                window.cmd_vel.linear.x = linear_speed
                window.cmd_vel.angular.z = angular_speed
                
                window.repaint_controller(x, y)

class Map_GraphicsScene(QGraphicsScene):
    def __init__(self, parent=None):
        QGraphicsScene.__init__(self, parent)
        self.center_x = 250
        self.center_y = 250
        self.prev_x = 250
        self.prev_y = 250
        self.scale = 0.2 #m/pix

    def mouseMoveEvent(self,event):
        pass

    def mousePressEvent(self, event):
        pass

class Path_Visualization_Simulator(QDialog):
    def __init__(self,parent=None):
        super(Path_Visualization_Simulator, self).__init__(parent)
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        self.controller_scene = Controller_GraphicsScene(self.ui.controller_graphicsView)
        self.ui.controller_graphicsView.setScene(self.controller_scene)

        self.map_scene = Map_GraphicsScene(self.ui.map_graphicsView)
        self.ui.map_graphicsView.setScene(self.map_scene)

        # Initialize ROS2
        rclpy.init(args=None)
        # Create a single node for both publishing and subscribing
        self.node = Node('path_visualization')
        
        # Create subscriber
        self.sub = self.node.create_subscription(
            Odometry, 
            '/odom', 
            self.listener_callback, 
            10
        )
        
        # Create Twist message and publisher
        self.cmd_vel = Twist()
        self.pub_cmd_vel = self.node.create_publisher(Twist, '/cmd_vel', 10)

        # Setup timer for ROS2 spinning and publishing
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.ros_update)
        self.timer.start(50)  # 20Hz update rate
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.pixmap = QPixmap(500, 500)
        self.vehicle_pen = QPen(Qt.red)
        self.center_line=QPen(Qt.black)
        self.center_line.setStyle(Qt.DashLine)
        self.map_scene.addLine(QLineF(250, 0, 250, 500), self.center_line) 
        self.map_scene.addLine(QLineF(0, 250, 500, 250), self.center_line)
        self.diameter = 2 #pix
        self.C = 10 #length
        self.first_time = True

        self.repaint_controller(75, 75)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return yaw_z # in radians

    def listener_callback(self, data):
        pose = data.pose.pose
        orientation = pose.orientation
        self.robot_x = pose.position.x
        self.robot_y = pose.position.y
        self.robot_theta = self.euler_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        self.update_map()

    def update_map(self):
        x = self.robot_x / self.map_scene.scale + self.map_scene.center_x
        y = -self.robot_y / self.map_scene.scale + self.map_scene.center_y
        angle = -math.pi/2 + self.robot_theta

        if not self.first_time:
            self.map_scene.clear()
            self.map_scene.addPixmap(self.pixmap)
            self.map_scene.addLine(QLineF(250, 0, 250, 500), self.center_line) 
            self.map_scene.addLine(QLineF(0, 250, 500, 250), self.center_line)
        else:
            self.first_time = False

        self.map_scene.addEllipse(x - int(self.diameter/2), y - int(self.diameter/2), 
                              self.diameter, self.diameter,
                              self.vehicle_pen)
        self.pixmap = self.ui.map_graphicsView.grab(QRect(QPoint(0,0),QSize(500, 500)))


        points = [[x - self.C*math.sin(angle), y - self.C*math.cos(angle)], 
                  [x - math.cos(angle)*self.C/2 + math.sqrt(3)*math.sin(angle)*self.C/2,
                   y + math.sin(angle)*self.C/2 + math.sqrt(3)*math.cos(angle)*self.C/2],
                  [x + math.cos(angle)*self.C/2 + math.sqrt(3)*math.sin(angle)*self.C/2,
                   y - math.sin(angle)*self.C/2 + math.sqrt(3)*math.cos(angle)*self.C/2]]

        qpoly = QPolygonF([QPointF(p[0], p[1]) for p in points])
        self.map_scene.addPolygon(qpoly, QPen(Qt.blue), QBrush(Qt.blue))
        self.ui.map_graphicsView.setScene(self.map_scene)
        self.ui.label.setText(f"Robot Position: ({self.robot_x:.2f}, {self.robot_y:.2f}, {self.robot_theta:.2f})")


    def ros_update(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)
        self.pub_cmd_vel.publish(self.cmd_vel)

    def repaint_controller(self, x, y):
        self.controller_scene.clear()
        self.controller_scene.addEllipse(0, 0, 150, 150, QPen(QColor(0,180,100)), QBrush(QColor(0,180,100)))
        self.controller_scene.addEllipse(int(x-5), int(y-5), 10, 10, QPen(Qt.red), QBrush(Qt.red))
        self.ui.controller_graphicsView.setScene(self.controller_scene)

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        super(Path_Visualization_Simulator, self).closeEvent(event)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Path_Visualization_Simulator()
    window.show()
    sys.exit(app.exec_())
