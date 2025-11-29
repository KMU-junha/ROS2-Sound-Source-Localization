#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
import message_filters
import numpy as np
import math
import time 

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf_transformations import euler_from_quaternion
import rclpy.duration

from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

class TriangulatorTF(Node):

    def __init__(self):
        super().__init__('triangulator_tf') 
        self.get_logger().info('--- Triangulator Node (Stagnation Filter Added) Ready ---')

        self.marker_id_counter = 0
        self.target_id_counter = 1 

        # [클러스터링 파라미터]
        self.recent_points = []
        self.CLUSTER_TIME_WINDOW = 2.0  
        self.CLUSTER_STD_LIMIT = 0.2
        self.MIN_SAMPLES = 4

        # [중복 방지 파라미터]
        self.confirmed_targets = [] 
        self.DUPLICATE_DIST_LIMIT = 0.5

        # [NEW] 고인물(Frozen) 데이터 감지 변수
        self.prev_pc_data = None
        self.pc_stale_count = 0
        
        self.prev_robot_data = None
        self.robot_stale_count = 0
        
        # 값이 연속으로 5번 똑같이 들어오면 "죽은 센서"로 간주하고 무시
        # (실제 센서는 노이즈 때문에 소수점 끝자리가 계속 바뀌어야 함)
        self.STALE_THRESHOLD = 5 

        # TF 리스너
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- 구독 ---
        self.pc_sub = message_filters.Subscriber(
            self, Float32, '/pc/sound_direction')
        self.robot_sub = message_filters.Subscriber(
            self, Float32, '/robot/sound_direction')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.pc_sub, self.robot_sub],
            queue_size=10,
            slop=0.5, 
            allow_headerless=True
        )
        self.ts.registerCallback(self.calculate_intersection)

        latched_qos = QoSProfile(
            depth=100, 
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        self.marker_pub = self.create_publisher(
            Marker, '/sound_source_location_marker', latched_qos)
        
        self.target_pub = self.create_publisher(
            Marker, '/sound_source_confirmed_target', latched_qos)
        
        self.get_logger().info('"두뇌" 노드 대기중...')


    def calculate_intersection(self, pc_msg, robot_msg):
        
        # --- [NEW] 고인물 데이터 필터링 (Stagnation Check) ---
        # 1. PC 마이크 검사
        if self.prev_pc_data is not None and pc_msg.data == self.prev_pc_data:
            self.pc_stale_count += 1
        else:
            self.pc_stale_count = 0 # 값이 변하면 카운터 리셋
        self.prev_pc_data = pc_msg.data

        # 2. 로봇 마이크 검사
        if self.prev_robot_data is not None and robot_msg.data == self.prev_robot_data:
            self.robot_stale_count += 1
        else:
            self.robot_stale_count = 0 # 값이 변하면 카운터 리셋
        self.prev_robot_data = robot_msg.data

        # 3. 둘 중 하나라도 고여있으면(멈춰있으면) 계산 중단
        if self.pc_stale_count > self.STALE_THRESHOLD:
            # self.get_logger().info("PC 마이크 값 고정됨(Ghost) - 무시함")
            return
        
        if self.robot_stale_count > self.STALE_THRESHOLD:
            # self.get_logger().info("로봇 마이크 값 고정됨(Ghost) - 무시함")
            return
        # -------------------------------------------------------

        try:
            latest_time = rclpy.time.Time(seconds=0)
            
            trans_pc = self.tf_buffer.lookup_transform(
                'odom', 'pc_mic_link', latest_time, timeout=rclpy.duration.Duration(seconds=0.1))
            p1_pos = np.array([trans_pc.transform.translation.x, trans_pc.transform.translation.y])
            p1_yaw = self.get_yaw_from_quaternion(trans_pc.transform.rotation)

            trans_robot = self.tf_buffer.lookup_transform(
                'odom', 'robot_mic_link', latest_time, timeout=rclpy.duration.Duration(seconds=0.1))
            p2_pos = np.array([trans_robot.transform.translation.x, trans_robot.transform.translation.y])
            p2_yaw = self.get_yaw_from_quaternion(trans_robot.transform.rotation)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            return 

        global_angle_1 = p1_yaw + np.radians(pc_msg.data)
        global_angle_2 = p2_yaw + np.radians(robot_msg.data)

        x, y = self.find_intersection_point(
            p1_pos, global_angle_1, 
            p2_pos, global_angle_2
        )

        if x is not None and y is not None:
            if not math.isinf(x) and not math.isinf(y):
                
                dist_origin = math.sqrt(x**2 + y**2)

                # 1. 기본 필터링
                vec_to_target_1 = np.array([x - p1_pos[0], y - p1_pos[1]])
                direction_vec_1 = np.array([np.cos(global_angle_1), np.sin(global_angle_1)])
                dot_product_1 = np.dot(vec_to_target_1, direction_vec_1)

                vec_to_target_2 = np.array([x - p2_pos[0], y - p2_pos[1]])
                direction_vec_2 = np.array([np.cos(global_angle_2), np.sin(global_angle_2)])
                dot_product_2 = np.dot(vec_to_target_2, direction_vec_2)

                if dot_product_1 < 0 or dot_product_2 < 0: 
                    return

                if dist_origin > 10.0: 
                    return 

                # 2. 빨간 점(Raw Data) 시각화
                angle_diff = global_angle_1 - global_angle_2
                angle_score = abs(math.sin(angle_diff)) 
                dist_score = max(0.0, 1.0 - (dist_origin / 5.0))
                confidence = angle_score * dist_score
                
                final_alpha = 0.5 + (0.5 * (confidence * confidence))

                self.publish_red_dot(x, y, final_alpha)

                # 3. 데이터 수집 (클러스터링용)
                current_time = time.time()
                self.recent_points.append([x, y, current_time])
                
                self.recent_points = [p for p in self.recent_points 
                                      if current_time - p[2] < self.CLUSTER_TIME_WINDOW]

                # 4. 확정 로직 (Green Star + Text)
                if len(self.recent_points) >= self.MIN_SAMPLES:
                    points_np = np.array(self.recent_points)[:, :2]
                    
                    std_dev = np.std(points_np, axis=0) 
                    avg_std = np.mean(std_dev)

                    if avg_std < self.CLUSTER_STD_LIMIT:
                        mean_pos = np.mean(points_np, axis=0)
                        cx, cy = mean_pos[0], mean_pos[1]
                        
                        is_duplicate = False
                        for tx, ty in self.confirmed_targets:
                            dist = math.sqrt((cx - tx)**2 + (cy - ty)**2)
                            if dist < self.DUPLICATE_DIST_LIMIT:
                                is_duplicate = True
                                break
                        
                        if not is_duplicate:
                            self.get_logger().info(f'★ 타겟 확정! #{self.target_id_counter} ({cx:.2f}, {cy:.2f})')
                            self.publish_green_star(cx, cy)
                            self.confirmed_targets.append((cx, cy))
                            self.recent_points = []
            else:
                pass
        else:
            pass


    def publish_red_dot(self, x, y, alpha):
        marker_msg = Marker()
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.header.frame_id = 'odom' 
        marker_msg.ns = "sound_source_raw"
        marker_msg.id = self.marker_id_counter
        self.marker_id_counter += 1 
        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD
        marker_msg.pose.position.x = float(x)
        marker_msg.pose.position.y = float(y)
        marker_msg.pose.position.z = 0.0
        marker_msg.pose.orientation.w = 1.0 
        marker_msg.scale.x = 0.1 
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.1
        marker_msg.color.r = 1.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = float(alpha) 
        marker_msg.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg() 
        self.marker_pub.publish(marker_msg)

    def publish_green_star(self, x, y):
        marker_msg = Marker()
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.header.frame_id = 'odom' 
        marker_msg.ns = "sound_source_confirmed"
        
        current_id = self.target_id_counter
        marker_msg.id = current_id 

        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD
        marker_msg.pose.position.x = float(x)
        marker_msg.pose.position.y = float(y)
        marker_msg.pose.position.z = 0.0
        marker_msg.pose.orientation.w = 1.0 
        
        marker_msg.scale.x = 0.15
        marker_msg.scale.y = 0.15
        marker_msg.scale.z = 0.15
        
        marker_msg.color.r = 0.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 0.8 
        marker_msg.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg() 
        
        self.target_pub.publish(marker_msg)

        text_marker = Marker()
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.header.frame_id = 'odom'
        text_marker.ns = "sound_source_label" 
        
        text_marker.id = 10000 + current_id
        
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        text_marker.text = f"[detected]sound" 
        
        text_marker.pose.position.x = float(x)
        text_marker.pose.position.y = float(y)
        text_marker.pose.position.z = 0.3 
        text_marker.pose.orientation.w = 1.0
        
        text_marker.scale.z = 0.15 
        
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        text_marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()

        self.target_pub.publish(text_marker)

        self.target_id_counter += 1

    def get_yaw_from_quaternion(self, q_msg):
        q = (q_msg.x, q_msg.y, q_msg.z, q_msg.w)
        euler = euler_from_quaternion(q)
        return euler[2]

    def find_intersection_point(self, p1, angle1, p2, angle2):
        x1, y1 = p1
        x2, y2 = p2
        if np.isclose(angle1, np.pi/2) or np.isclose(angle1, -np.pi/2):
            x = x1
            if np.isclose(angle2, np.pi/2) or np.isclose(angle2, -np.pi/2): return (None, None) 
            m2 = np.tan(angle2)
            y = m2 * (x - x2) + y2
            return (x, y)
        if np.isclose(angle2, np.pi/2) or np.isclose(angle2, -np.pi/2):
            x = x2
            m1 = np.tan(angle1)
            y = m1 * (x - x1) + y1
            return (x, y)
        m1 = np.tan(angle1)
        m2 = np.tan(angle2)
        if np.isclose(m1, m2): return (None, None) 
        x = (m1 * x1 - m2 * x2 + y2 - y1) / (m1 - m2)
        y = m1 * (x - x1) + y1
        return (x, y)

def main(args=None):
    rclpy.init(args=args)
    triangulator_tf = TriangulatorTF()
    rclpy.spin(triangulator_tf)
    triangulator_tf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
