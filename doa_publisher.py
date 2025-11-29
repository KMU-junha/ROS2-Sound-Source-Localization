 #!/usr/bin/env python3


import rclpy

from rclpy.node import Node

from std_msgs.msg import Float32

from geometry_msgs.msg import PoseStamped

import numpy as np

import usb.core

import usb.util

import time

import sys

import os # $HOME을 사용하기 위해 추가


# ###############################################################

# ### ❗ [수정] "tuning.py" (성공한 코드)를 "수입"하기 위한 경로 설정 ###


# (중요) /home/junha 대신, 현재 사용자의 홈 디렉토리를 사용하도록 수정

home_dir = os.path.expanduser("~") 

tuning_path = os.path.join(home_dir, 'usb_4_mic_array')

sys.path.append(tuning_path)


try:

    from tuning import Tuning

except ImportError:

    print(f"[ERROR] 'tuning' 모듈을 찾을 수 없습니다.")

    print(f"[ERROR] {tuning_path} 폴더가 있는지 확인하세요.")

    sys.exit(1)

# ###############################################################



class RespeakerDoaPublisher(Node):


    def __init__(self):

        super().__init__('respeaker_doa_publisher')


        # v2.0의 USB ID (lsusb 확인)

        self.VENDOR_ID = 0x2886

        self.PRODUCT_ID = 0x0018

        self.dev = None

        self.Mic_tuning = None

        

        self.TIMER_RATE = 0.1 # 1초에 10번 (10Hz)


        self.angle_publisher = self.create_publisher(Float32, '/sound_direction', 10)

        self.arrow_publisher = self.create_publisher(PoseStamped, '/sound_direction_arrow', 10)


        if not self.setup_usb_device():

            self.get_logger().error('*** 노드를 종료합니다. pavucontrol을 "Off"로 설정한 후 다시 시도하세요. ***')

            return 

        else:

            self.get_logger().info('--- ReSpeaker DOA Publisher (Plan L - FINAL) Ready ---')

            self.get_logger().info('XMOS 칩(v2.0)에서 "Tuning" 클래스로 각도를 읽어옵니다.')

            self.get_logger().info('*** 중요: pavucontrol에서 이 마이크가 "Off"로 설정되었습니다. ***')

            self.timer = self.create_timer(self.TIMER_RATE, self.read_and_publish_doa)



    def setup_usb_device(self):

        """USB 장치를 찾고 "Tuning" 클래스를 초기화합니다."""

        self.dev = usb.core.find(idVendor=self.VENDOR_ID, idProduct=self.PRODUCT_ID)

        

        if self.dev is None:

            self.get_logger().error('ReSpeaker Mic Array v2.0 (0018)를 찾을 수 없습니다.')

            return False

        

        try:

            # "DOA.py" (Plan K)의 "성공한" 로직

            self.Mic_tuning = Tuning(self.dev)

            

            self.get_logger().info('ReSpeaker Mic Array v2.0 (0018)에 성공적으로 연결되었습니다.')

            return True

            

        except usb.core.USBError as e:

            self.get_logger().error(f'USB 장치 연결에 실패했습니다: {e}')

            self.get_logger().error('*** [Errno 16] Resource busy: pavucontrol에서 "Off"로 설정하지 않았습니다! ***')

            return False


    def read_and_publish_doa(self):

        try:

            # "DOA.py" (Plan K)의 "성공한" 로직

            doa_angle_raw = self.Mic_tuning.direction # 0~359

            

            # 0~360도를 ROS 표준(-180 ~ +180)으로 변환

            if doa_angle_raw > 180:

                final_angle = doa_angle_raw - 360

            else:

                final_angle = doa_angle_raw

                

            # [보정] 펌웨어의 "정면" (90°)을 ROS의 "정면" (0°)으로 변환

            final_angle_ros = (final_angle - 90)

            if final_angle_ros < -180:

                final_angle_ros += 360

        

            # (VAD가 내장되어 있으므로, 0이 아닌 유효한 각도일 때만 로깅)

            if doa_angle_raw != 0: 

                self.get_logger().info(f'[Voice Detected] On-Chip DOA: {doa_angle_raw}° | ROS Angle: {final_angle_ros:.0f}°')


            angle_msg = Float32()

            angle_msg.data = float(final_angle_ros) # ROS 표준 각도 발행

            self.angle_publisher.publish(angle_msg)

            

            arrow_msg = PoseStamped()

            arrow_msg.header.stamp = self.get_clock().now().to_msg()

            arrow_msg.header.frame_id = "pc_mic_link" # (로봇 몸체 기준)

            

            q_z = np.sin(np.radians(final_angle_ros) / 2.0)

            q_w = np.cos(np.radians(final_angle_ros) / 2.0)

            

            arrow_msg.pose.orientation.x = 0.0

            arrow_msg.pose.orientation.y = 0.0

            arrow_msg.pose.orientation.z = q_z

            arrow_msg.pose.orientation.w = q_w

            

            self.arrow_publisher.publish(arrow_msg)


        except usb.core.USBError as e:

            self.get_logger().error(f'USB 읽기 실패: {e}')

        except Exception as e:

            self.get_logger().error(f'알 수 없는 오류: {e}')


    def destroy_node(self):

        self.get_logger().info('ReSpeaker DOA Publisher를 종료합니다.')

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    doa_publisher = RespeakerDoaPublisher()

    if doa_publisher.Mic_tuning is not None: # (연결에 성공했을 때만 spin)

        try:

            rclpy.spin(doa_publisher)

        except KeyboardInterrupt:

            pass

        finally:

            doa_publisher.destroy_node()

            rclpy.shutdown()

    else:

        print("[FATAL] RespeakerDoaPublisher 초기화 실패. 노드를 실행할 수 없습니다.")


if __name__ == '__main__':

    main() 
