# 2025-02-24
# sorting

# -----------진행 상황----------
# 일단 알고리즘 생성

# -----------해야 하는 거--------
# rclpy 해결
# 알고리즘

# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
from rclpy.node import Node
import DR_init
from std_msgs.msg import Float64MultiArray

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

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
        print(f'data: {data}')


    # def joint_state_callback(self, msg):
    #     self.joint_state_msg = msg
    #     data = [round(d, 3) for d in self.joint_state_msg.data]



def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("sorting", namespace=ROBOT_ID)
    pose_node = PosTopicSubscriber()

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            trans,
            set_digital_output,
            # get_pattern_point,
        )

        from DR_common2 import posx
        
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        # wait_digital_input(2)

    def grip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        # wait_digital_input(1)

    def moving(pattern, current, goal):
        pose_node = PosTopicSubscriber()
        ci, cj = current
        gi, gj = goal
        grip()
        movel(pattern[3*ci+cj], vel=VELOCITY, acc=ACC)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass

        release_compliance_ctrl()
        release()
        poss = rclpy.spin_once(pose_node)
        trans(poss, [0, 0, -2, 0, 0, 0], DR_BASE, DR_BASE)
        grip()

        movel(pattern[3*ci+cj], vel=VELOCITY, acc=ACC)
        movel(pattern[3*gi]+gj)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass

        release_compliance_ctrl()
        release()
        movel(pattern[3*gi+gj])

    # 초기 위치
    JReady = [0, 0, 90, 0, 90, 0]
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    pos1 = posx([500.362, -103.859, 130.0, 0.0, -180.0, 0.0])
    pos2 = posx([500.362, -53.859, 130.0, 0.0, -180.0, 0.0])
    pos3 = posx([500.362, -3.859, 130.0, 0.0, -180.0, 0.0])
    pos4 = posx([449.362, -103.859, 130.0, 0.0, -180.0, 0.0])
    pos5 = posx([449.362, -53.859, 130.0, 0.0, -180.0, 0.0])
    pos6 = posx([449.362, -3.859, 130.0, 0.0, -180.0, 0.0])
    pos7 = posx([400.362, -103.859, 130.0, 0.0, -180.0, 0.0])
    pos8 = posx([400.362, -53.859, 130.0, 0.0, -180.0, 0.0])
    pos9 = posx([400.362, -3.859, 130.0, 0.0, -180.0, 0.0])

    pos = [pos1,pos2,pos3,pos4,pos5,pos6,pos7,pos8,pos9]


    # pattern = get_pattern_point(pos1, pos2, pos3, pos4, 8, 1, 3, 3, 1, 0.0, None)
    center_pose = posx([701.26, -51.964, 153.0, 0.0, -180.0, 0.0])

    while rclpy.ok():
        # 초기 위치로 이동
        print('-----move to init-----')
        movej(JReady, vel=VELOCITY, acc=ACC)

        # 현재 위치별 높이
        height = [[0, 0, 0],[0, 0, 0],[0, 0, 0]]

        grip()
        print('-----check height-----')
        # 높이 확인
        for i in pos:
            
            movel(i,vel=VELOCITY, acc=ACC)


            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass

            release_compliance_ctrl()

            # 현재 위치 받아서 저장
            poss = rclpy.spin_once(pose_node)
            height[i//3][i%3] = poss[2]
            # 이를 바탕으로 trans

        # 가운데거 옮기기
        grip()
        movel(pos[4], vel=VELOCITY, acc=ACC)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass

        release_compliance_ctrl()
        poss = rclpy.spin_once(pose_node)
        release()
        trans(poss, [0, 0, -2, 0, 0, 0], DR_BASE, DR_BASE)
        
        grip()
        movel(pos[4], vel=VELOCITY, acc=ACC)
        movel(center_pose, vel=VELOCITY, acc=ACC)
        # 하강
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass

        release_compliance_ctrl()
        release()
        movel(center_pose, vel=VELOCITY, acc=ACC)

        # 비어 있는 곳의 높이
        whole = [1,1]   # index
        threshold = [[1, 2, 3],[1.5, 2.5, 3.5]]
        wrong = []

    # 틀린애들 확인
    for i in range(3):
        for j in range(3):
            if not (height[i][j] >= threshold[i][0] and height[i][j] < threshold[i][1]):
                wrong.append([i,j])

    # 정렬
    while True:
        moved = 0
        for i, j in wrong:
            if height[i][j] >= threshold[whole[0]][0] and height[i][j] < threshold[whole[0]][1]:
                # 움직이게 하기
                moving(pos, [i,j], [whole[0],whole[1]])

                moved = 1
                whole = [i,j]
                del height[i]
                break
        
        if moved == 0:
            # 가장 먼저 나오는거 빈 공간으로 움직임
            moving(pos, [wrong[0][0],wrong[0][1]], [whole[0],whole[1]])


        if len(wrong) == 0:
            break

    # 가운데 있는거 되돌리기
    grip()
    movel(center_pose, vel=VELOCITY, acc=ACC)
    # 하강
    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    while not check_force_condition(DR_AXIS_Z, max=5):
        pass

    release_compliance_ctrl()
    release()
    trans()
    grip()

    movel(pos[4], vel=VELOCITY, acc=ACC)
    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    while not check_force_condition(DR_AXIS_Z, max=5):
        pass

    release_compliance_ctrl()
    release()
    movel(pos[4], vel=VELOCITY, acc=ACC)


    rclpy.shutdown()


if __name__ == "__main__":
    main()
