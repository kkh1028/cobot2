# 2025-02-24
# 젠가 높게 쌓기

# --------진행 상황-------------
# 일단 되는건 만듬


# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
from rclpy.node import Node
import DR_init
from std_msgs.msg import Float64MultiArray
import threading


# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0

class PosTopicSubscriber(Node):
    def __init__(self):
        super().__init__("PosTopicSubscriber")
        print('-----start node------')

        # 메시지 저장 변수 초기화
        self.current_posx_msg = None

        # 토픽 구독 설정
        self.create_subscription(
            Float64MultiArray, "/dsr01/msg/current_posx", self.current_posx_callback, 10
        )
        # self.create_subscription(
        #     Float64MultiArray, "/dsr01/msg/joint_state", self.joint_state_callback, 10
        # )

    def current_posx_callback(self, msg):
        print('-----callback------')
        self.current_posx_msg = msg
        data = [round(d, 3) for d in self.current_posx_msg.data]
        print(data)


    # def joint_state_callback(self, msg):
    #     self.joint_state_msg = msg
    #     data = [round(d, 3) for d in self.joint_state_msg.data]


# def ros_thread():
#     node = PosTopicSubscriber()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_test_node", namespace=ROBOT_ID)
    pose_node = PosTopicSubscriber()

    DR_init.__dsr__node = node

    # # ROS2 스레드 실행
    # ros = threading.Thread(target=ros_thread, args=(None))
    # ros.start()

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            wait,
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            trans,
            set_digital_output,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
 
    # 원하는 위치 미리 지정
    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx([367.48, 8.722, 200.576, 89.91, 179.976, 89.528])
    pos2 = posx([367.48, 8.722, 200.576, 89.91, 179.976, 89.528])
    # pos3 = posx([367.48, 8.722, 200.576, 89.91, 179.976, 89.528])
    # pos4 = posx([367.48, 8.722, 200.576, 89.91, 179.976, 89.528])
    # pos5 = posx([367.48, 8.722, 200.576, 89.91, 179.976, 89.528])
    # pos6 = posx([367.48, 8.722, 200.576, 89.91, 179.976, 89.528])

    # pos = [pos1,pos2,pos3,pos4,pos5,pos6]
    pos = [pos1, pos2]

    pos_wood = posx([367.48, 8.722, 200.576, 89.91, 179.976, 89.528])
    pos_wood_release = posx([0, 0, 2, 0, 0, 0])
    pos_wood_grip = posx([0, 0, -4, 0, 0, 0])
    # pos1 = posx([496.06, 93.46, 296.92, 20.75, 179.00, 19.09])
    # pos2 = posx([548.70, -193.46, 96.92, 20.75, 179.00, 19.09])
    # pos3 = posx([596.70, -7.46, 196.92, 20.75, 179.00, 19.09])

    # 그리퍼가 무언가를 잡으면
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
            pass
    
    # 그리퍼 놓기
    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        # wait_digital_input(2)

    # 그리퍼 잡기, 잡는거 확인하려면 wait_digital_input 주석 해제
    def grip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        # wait_digital_input(1)
    
    # 힘 제어
    def force_control(x = 0,y = 0,z = 0,dir_x = 0,dir_y = 0,dir_z = 0):
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[x, y, z, 0, 0, 0], dir=[dir_x, dir_y, dir_z, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass

        release_compliance_ctrl()    

    # 툴,tcp 정의
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    grip()
    # 초기 위치로 이동
    JReady = [0, 0, 90, 0, 90, 0]
    movej(JReady, vel=VELOCITY, acc=ACC)

    while rclpy.ok():
    
        # 이동
        # print("movej")
        # movej(JReady, vel=VELOCITY, acc=ACC)
        # print("movel")
        # movel(pos1, vel=VELOCITY, acc=ACC)
        # print("movel")
        # movel(pos2, vel=VELOCITY, acc=ACC)
        # print("movel")
        # movel(pos3, vel=VELOCITY, acc=ACC)
        print('initial pose')
        movej(JReady, vel=VELOCITY, acc=ACC)


        print('to wood')
        movel(pos_wood, vel=VELOCITY, acc=ACC)

        print('grap wood')
        force_control(z = -10, dir_z = 1)
        
        print('------spin once--------')
        a = rclpy.spin_once(pose_node)
        print(a)

        # print('wood touch')
        # trans(current, pos_wood_release, DR_BASE, DR_BASE)

        # release()

        # print('grip wood')

        # trans(current, pos_wood_release, DR_BASE, DR_BASE)
        # grip()

        # print('to wood position')
        # movel(pos_wood, vel=VELOCITY, acc=ACC)

        # print('stack')
        # movel(pos[i], vel=VELOCITY, acc=ACC)
        # force_control(y = -10, dir_y = 1)
        # release()

        # print('initial pose')
        # movej(JReady, vel=VELOCITY, acc=ACC)
        
        print('done')

        # 패턴
        # get_pattern_point(pos1, pos2, pos3, pos4, index, pattern, row, column, stack, stack_offset, point_offset)



    rclpy.shutdown()
if __name__ == "__main__":
    main()
