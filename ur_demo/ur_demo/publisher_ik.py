import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import math
import numpy as np

ZERO_THRESH = 1e-6
PI = math.pi

def inverse(T):
    d1 = 0.1625
    a2 = -0.425
    a3 = -0.39225
    d4 = 0.1333
    d5 = 0.0997
    d6 = 0.0996

    num_sols = 0
    T02 = -T[0]
    T00 = T[1]
    T01 = T[2]
    T03 = -T[3]
    T12 = -T[4]
    T10 = T[5]
    T11 = T[6] 
    T13 = -T[7] 
    T22 = T[8] 
    T20 = -T[9] 
    T21 = -T[10]
    T23 = T[11]


    q_sols = []

    # Shoulder rotate joint (q1)
    q1 = [0, 0]
    A = d6 * T12 - T13
    B = d6 * T02 - T03
    R = A * A + B * B

    if abs(A) < ZERO_THRESH:
        if abs(abs(d4) - abs(B)) < ZERO_THRESH:
            div = -np.sign(d4) * np.sign(B)
        else:
            div = -d4 / B
        arcsin = math.asin(div)
        if abs(arcsin) < ZERO_THRESH:
            arcsin = 0.0
        if arcsin < 0.0:
            q1[0] = arcsin + 2.0 * PI
        else:
            q1[0] = arcsin
        q1[1] = PI - arcsin
    elif abs(B) < ZERO_THRESH:
        if abs(abs(d4) - abs(A)) < ZERO_THRESH:
            div = np.sign(d4) * np.sign(A)
        else:
            div = d4 / A
        arccos = math.acos(div)
        q1[0] = arccos
        q1[1] = 2.0 * PI - arccos
    elif d4 * d4 > R:
        return num_sols, q_sols
    else:
        arccos = math.acos(d4 / math.sqrt(R))
        arctan = math.atan2(-B, A)
        pos = arccos + arctan
        neg = -arccos + arctan
        if abs(pos) < ZERO_THRESH:
            pos = 0.0
        if abs(neg) < ZERO_THRESH:
            neg = 0.0
        if pos >= 0.0:
            q1[0] = pos
        else:
            q1[0] = 2.0 * PI + pos
        if neg >= 0.0:
            q1[1] = neg
        else:
            q1[1] = 2.0 * PI + neg

    # Wrist 2 joint (q5)
    q5 = [[0, 0], [0, 0]]
    for i in range(2):
        numer = (T03 * math.sin(q1[i]) - T13 * math.cos(q1[i]) - d4)
        if abs(abs(numer) - abs(d6)) < ZERO_THRESH:
            div = np.sign(numer) * np.sign(d6)
        else:
            div = numer / d6
        arccos = math.acos(div)
        q5[i][0] = arccos
        q5[i][1] = 2.0 * PI - arccos

    for i in range(2):
        for j in range(2):
            c1 = math.cos(q1[i])
            s1 = math.sin(q1[i])
            c5 = math.cos(q5[i][j])
            s5 = math.sin(q5[i][j])
            q6 = 0

            # Wrist 3 joint (q6)
            if abs(s5) < ZERO_THRESH:
                # q6 = q6_des
                q6 = 0.
            else:
                q6 = math.atan2(np.sign(s5) * -(T01 * s1 - T11 * c1), np.sign(s5) * (T00 * s1 - T10 * c1))
                if abs(q6) < ZERO_THRESH:
                    q6 = 0.0
                if q6 < 0.0:
                    q6 += 2.0 * PI

            q2 = [0, 0]
            q3 = [0, 0]
            q4 = [0, 0]

            # RRR joints (q2, q3, q4)
            c6 = math.cos(q6)
            s6 = math.sin(q6)
            x04x = -s5 * (T02 * c1 + T12 * s1) - c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1))
            x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5
            p13x = d5 * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1)) - d6 * (T02 * c1 + T12 * s1) + T03 * c1 + T13 * s1
            p13y = T23 - d1 - d6 * T22 + d5 * (T21 * c6 + T20 * s6)

            c3 = (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3) / (2.0 * a2 * a3)

            if abs(abs(c3) - 1.0) < ZERO_THRESH:
                c3 = np.sign(c3)
            elif abs(c3) > 1.0:
                continue

            arccos = math.acos(c3)
            q3[0] = arccos
            q3[1] = 2.0 * PI - arccos
            denom = a2 * a2 + a3 * a3 + 2 * a2 * a3 * c3
            s3 = math.sin(arccos)
            A = (a2 + a3 * c3)
            B = a3 * s3

            q2[0] = math.atan2((A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom)
            q2[1] = math.atan2((A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom)

            c23_0 = math.cos(q2[0] + q3[0])
            s23_0 = math.sin(q2[0] + q3[0])
            c23_1 = math.cos(q2[1] + q3[1])
            s23_1 = math.sin(q2[1] + q3[1])

            q4[0] = math.atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0)
            q4[1] = math.atan2(c23_1 * x04y - s23_1 * x04x, x04x * c23_1 + x04y * s23_1)

            # Ensure angles are within [0, 2*pi] and set small angles to 0
            for k in range(2):
                if abs(q2[k]) < ZERO_THRESH:
                    q2[k] = 0.0
                elif q2[k] < 0.0:
                    q2[k] += 2.0 * PI
                if abs(q4[k]) < ZERO_THRESH:
                    q4[k] = 0.0
                elif q4[k] < 0.0:
                    q4[k] += 2.0 * PI

                # Append the solutions to the list
                q_sols.append([q1[i], q2[k], q3[k], q4[k], q5[i][j], q6])
                num_sols += 1

    for i, sol in enumerate(q_sols):
        for j, q in enumerate(sol):
            if q > PI:
                q_sols[i][j] = - 2*PI + q

    return num_sols, q_sols


class PublisherIK(Node):

    def __init__(self):
        super().__init__('publisher_ik')
        self.joint_state_sub = self.create_subscription(
                Float32MultiArray, "/ee_position", self.ee_position_callback, 1)
        self.publisher_ = self.create_publisher(Float32MultiArray, '/ik_joint_values', 1)
    
        # timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        self.q_sol = []

    def ee_position_callback(self, msg):
        if msg.data:
            T = [msg.data[i] for i in range(len(msg.data))]
            num_solutions, q_sols = inverse(T)
            if num_solutions > 0:
                self.q_sol = q_sols[1]

                msg = Float32MultiArray()
                msg.data = self.q_sol
                self.publisher_.publish(msg)

    # def timer_callback(self):
    #     msg = Float32MultiArray()

    #     msg.data = self.q_sol

    #     self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    publisher_ik = PublisherIK()

    rclpy.spin(publisher_ik)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher_ik.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
