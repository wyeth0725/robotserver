#!/usr/bin/env python
#coding:utf-8

#cdn_dictとかいう名前の変数辞書型じゃないので気が向いたら治す
#というか色々汚いので治す
from __future__ import print_function
import struct
import sys
import rospy
import baxter_interface
import socket
import time
from baxter_interface import CHECK_VERSION
from IKmod import IKsolve
from mover import Mover
from baxter_pykdl import baxter_kinematics
import numpy as np
import math
import string
from contextlib import closing
from std_srvs.srv import Empty
import re
from reflex_msgs.msg import Command
from reflex_msgs.msg import PoseCommand
from reflex_msgs.msg import VelocityCommand
from reflex_msgs.msg import Hand
from reflex_msgs.msg import FingerPressure
from reflex_msgs.srv import SetTactileThreshold, SetTactileThresholdRequest
import signal
import select

class server:
    def __init__(self):
        #サーバー初期化
        baxter = baxter_interface.RobotEnable()
        baxter.enable()
        #右手を使う
        self.limb = "right"
        self.right = baxter_interface.Limb(self.limb)
        #アームの速度上限
        self.right.set_joint_position_speed(0.3)
        self.control_cmd = dict()
        self.IKservice = IKsolve(self.limb)
        self.Mover = Mover()

        self.rate = rospy.Rate(100000)
        #初期位置に移動
        self.right.move_to_neutral()
        #ReFlexHand初期化とか
        # Services can automatically call hand calibration

        self.calibrate_fingers = rospy.ServiceProxy('/reflex_takktile/calibrate_fingers', Empty)
        self.calibrate_tactile = rospy.ServiceProxy('/reflex_takktile/calibrate_tactile', Empty)

        # Services can set tactile thresholds and enable tactile stops
        self.enable_tactile_stops = rospy.ServiceProxy('/reflex_takktile/enable_tactile_stops', Empty)
        self.disable_tactile_stops = rospy.ServiceProxy('/reflex_takktile/disable_tactile_stops', Empty)
        self.set_tactile_threshold = rospy.ServiceProxy('/reflex_takktile/set_tactile_threshold', SetTactileThreshold)

        # This collection of publishers can be used to command the hand
        self.command_pub = rospy.Publisher('/reflex_takktile/command', Command, queue_size=1)
        self.pos_pub = rospy.Publisher('/reflex_takktile/command_position', PoseCommand, queue_size=1)
        self.vel_pub = rospy.Publisher('/reflex_takktile/command_velocity', VelocityCommand, queue_size=1)

        # Constantly capture the current hand state
        #rospy.Subscriber('/reflex_takktile/hand_state', Hand, hand_state_cb)
        f1 = FingerPressure([10, 10, 10, 10, 10, 10, 10, 10, 1000])
        f2 = FingerPressure([10, 10, 10, 10, 10, 10, 10, 10, 1000])
        f3 = FingerPressure([10, 10, 10, 10, 10, 10, 10, 10, 1000])
        threshold = SetTactileThresholdRequest([f1, f2, f3])
        #self.set_tactile_threshold(threshold)

    def mover(self):
        #self.coordinate = [x,y,z]
        #self.limb_joints = self.IKservice.solve(self.cdn_dict[0])
        #コマンド名でスイッチ
        if (self.cmd_name == "self_grasp_object"):
            self.self_grasp_object(self.t,self.s,self.w)


        elif (self.cmd_name == "receive_object"):
            self.receive_object(self.t,self.s,self.w)


        elif (self.cmd_name == "release_object"):
            self.release_object(self.t,self.s,self.w)


        elif (self.cmd_name == "point_object"):
            self.point_object(self.t)


        elif (self.cmd_name == "move_arm_to_point"):
            self.move_arm_to_point(self.t)


        elif (self.cmd_name == "yamove_arm_to_point"):
            self.yamove_arm_to_point(self.t)



        elif (self.cmd_name == "move_arm_traj"):
            self.move_arm_traj(self.t)


        elif (self.cmd_name == "open_hand"):
            self.open_hand(self.s,self.w)


        elif (self.cmd_name == "catch_object"):
            self.catch_object(self.s,self.w)


        elif (self.cmd_name == "gotoready"):
            self.gotoready(self.s,self.w,self.r)


        elif(self.cmd_name == "nullfunc"):
            self.nullfunc()


        elif (self.cmd_name == "init_hand"):
            self.init_hand()


        elif (self.cmd_name == "terminate"):
            self.terminate()


        else:
            modori = ("This line is not command")
            return modori

    def self_grasp_object(self,t,s,w):
        limb_joints = self.IKservice.solve(self.cdn_dict[0])
        if (limb_joints != 0):
            self.right.move_to_joint_positions(limb_joints,1)
        else:
            return 0
        self.pre_grasp(s)
        depth = self.IKservice.solve(self.cdn_dict[1])
        midcdn = [self.cdn_dict[0][0]+ 0.07,self.cdn_dict[0][1],self.cdn_dict[0][2]]
        mid = self.IKservice.solve(midcdn)
        if (depth != 0):
            self.right.move_to_joint_positions(mid,0.5)
            self.right.move_to_joint_positions(depth,1)
            self.grasp()
            time.sleep(t/50)
        elif(depth == 0 or depth == 1 ):
            self.cdn_dict[1][0] = self.cdn_dict[1][0] - 0.05
            depth = self.IKservice.solve(self.cdn_dict[1])
            if(depth != 0):
                self.right.move_to_joint_positions(depth,1)
                self.grasp()
                time.sleep(t/50)
        else:
            return 0



    def receive_object(self,t,w,s):
        limb_joints = self.IKservice.solve(self.cdn_dict[0])
        if (limb_joints != 0):
            self.right.move_to_joint_positions(limb_joints)
        else:
            return 0
        depth = self.IKservice.solve(self.cdn_dict[1])
        if (depth != 0):
            self.right.move_to_joint_positions(depth)
            self.grasp()
        else:
            return 0

    def release_object(self,t,w,s):
        limb_joints = self.IKservice.solve(self.cdn_dict[0])
        if (limb_joints != 0):
            self.right.move_to_joint_positions(limb_joints,1)
        else:
            return 0

        depth = self.IKservice.solve(self.cdn_dict[1])
        midcdn = [self.cdn_dict[0][0]+ 0.07,self.cdn_dict[0][1],self.cdn_dict[0][2]]
        mid = self.IKservice.solve(midcdn)
        if (depth != 0):
            self.right.move_to_joint_positions(mid,0.5)
            self.right.move_to_joint_positions(depth,1)
            self.open()
            time.sleep(t/50)
            self.right.move_to_joint_positions(limb_joints,1)
        elif(depth == 0 or depth == 1 ):
            self.cdn_dict[1][0] = self.cdn_dict[1][0] - 0.05
            depth = self.IKservice.solve(self.cdn_dict[1])
            if(depth != 0):
                self.right.move_to_joint_positions(depth,1)
                self.open()
                time.sleep(t/50)
        else:
            return 0



    def point_object(self,t):
        self.yamove_arm_to_point(t)
        limb_joints = self.IKservice.solve(self.cdn_dict[0])
        if (limb_joints != 0):
            self.right.move_to_joint_positions(limb_joints)
            self.pointing()
            time.sleep(2)
        else:
            return 0




    def move_arm_to_point(self,t):
        limb_joints = self.IKservice.solve(self.cdn_dict[0])
        if (limb_joints != 0):
            self.right.move_to_joint_positions(limb_joints)
        else:
            return 0



    def yamove_arm_to_point(self,t):
        cmd_dict = []
        pose = self.right.endpoint_pose()
        pt_cur = str(pose["position"]).translate(string.maketrans("",""),
                        "xyz= ()Point").split(",")
        pt_cur[1]=float(pt_cur[1])
        pt_cur[2]=float(pt_cur[2])
        pt_cur[0]=float(pt_cur[0])

        dif = abs(self.cdn_dict[0][1]) - abs(pt_cur[1])
        abs(dif)
        top = pt_cur[1] - dif/2
        print(self.cdn_dict[0][1],pt_cur[1],top)
        difx = self.cdn_dict[0][0] - pt_cur[0]
        topx = pt_cur[0] + difx/2
        topx = float(abs(topx))
        topz = pt_cur[2]
        top_cdn = [topx,top,topz]
        cmd = self.IKservice.solve(pt_cur)

        if (cmd != 0):
            self.right.move_to_joint_positions(cmd,1)
        cmd = self.IKservice.solve(top_cdn)
        if (cmd != 0):
            self.right.set_joint_positions(cmd)
        cmd = self.IKservice.solve(self.cdn_dict[0])
        if (cmd != 0):
            self.right.move_to_joint_positions(cmd,2)

    def move_arm_traj(self,t):
        self.Mover.map_file(self.right, self.cdn_dict, self.IKservice)
        self.rate.sleep()

    def open_hand(self):
        self.open()



    def catch_object(self,s,w):
        self.grasp()



    def gotoready(self,s,w,r):
        """
        kin = baxter_kinematics("right")
        kinFK = kin.forward_position_kinematics()
        if(float(kinFK[2]) <= 0.0):
            cdn = [float(kinFK[0]),float(kinFK[1]),0.05]
            cmd = self.IKservice.solve(cdn)
            if(cmd != 0):
                self.right.move_to_joint_positions(cmd,1)
        self.open()
        self.right.move_to_neutral()
        """
        cdn = [0.59648834,-0.62074376,-0.0941753]
        cmd = self.IKservice.solve(cdn)
        self.open()
        self.right.move_to_joint_positions(cmd)

    def nullfunc(self):
        None

    def init_hand(self):
        self.open()


    def terminate(self):
        server_sock.close()
        kin = baxter_kinematics("right")
        kinFK = kin.forward_position_kinematics()
        if(float(kinFK[2]) <= 0.0):
            cdn = [float(kinFK[0]),float(kinFK[1]),0.05]
            cmd = self.IKservice.solve(cdn)
            if(cmd != 0):
                self.right.move_to_joint_positions(cmd,1)
        self.open()
        self.right.move_to_neutral()
        time.sleep(7)
        rospy.is_shutdown()
        sys.exit()


    def make_command(self,msg):
        #コマンドを分割する
        x,y,z,depthx,depthy,depthz,self.t,self.s,self.w,self.r = 0,0,0,0,0,0,0,0,0,0
        self.keyt,self.keys,self.keyw,self.keyr = None,None,None,None
        msgspl = msg.split(" ")
        #コマンドによって場所が違うオプション引数を分離してコマンドから削除
        for i in range(int(len(msgspl))):

            if(re.match(r"-t",msgspl[i])):
                con = msgspl[i].split("-t")
                self.t = con[1]
                self.keyt = msgspl[i]
            elif(re.match(r"-s",msgspl[i])):
                con = msgspl[i].split("-s")
                self.s = con[1]
                self.keys = msgspl[i]
            elif(re.match(r"-w",msgspl[i])):
                con = msgspl[i].split("-w")
                self.w = con[1]
                self.keyw = msgspl[i]
            elif(re.match(r"-r",msgspl[i])):
                con = msgspl[i].split("-r")
                self.r = con[1]
                self.keyr = msgspl[i]
            #print(msgspl)

        else:

            if(self.keyt != None):
                msgspl.remove(self.keyt)
            if(self.keys != None):
                msgspl.remove(self.keys)
            if(self.keyw != None):
                msgspl.remove(self.keyw)
            if(self.keyr != None):
                msgspl.remove(self.keyr)

        #コマンド名は0個目で確定
        self.cmd_name = msgspl[0]
        self.cdn_dict = []

        #コマンド名とオプション引数を除けば残りは全部座標
        #3次元座標だから3つづつ辞書に入れる
        for i in range(1,(len(msgspl)),3):
            cdn = [float(msgspl[i]),float(msgspl[i+1]),float(msgspl[i+2])]
            self.cdn_dict.append(cdn)

        self.t = float(self.t)
        self.s = float(self.s)
        self.w = float(self.w)
        self.r = float(self.r)
        #動作して返り値を渡す
        #本来なら{0}のあと 0 なら動作成功、失敗したら1を返すが正直面倒くさいのでなし
        self.mover()
        if(len(msgspl) >= 4):
            modori = ("{0} 0 {1} {2} {3}\n".format(self.cmd_name,self.cdn_dict[0][0],self.cdn_dict[0][1],self.cdn_dict[0][2]))
            return modori
        else:
            modori2 = ("{0} 0\n".format(self.cmd_name))
            return modori2

    #ReFlexHandの指の開く範囲

    #物体の幅に応じて開き方を変える
    def pre_grasp(self,s):
        #self.pos_pub.publish(VelocityCommand(f1 = s * 5, f2 = s * 5 ,f3 = s * 5, preshape = 0.0))
        self.pos_pub.publish(VelocityCommand(f1 = s * 5, f2 = 0, f3 = s * 5, preshape = 0.0))

    #物体の把持
    def grasp(self):
        #self.pos_pub.publish(VelocityCommand(f1 = 3, f2 = 3, f3 = 3, preshape = 0.0))
        self.pos_pub.publish(VelocityCommand(f1 = 3, f2 = 0, f3 = 3, preshape = 0.0))

    #手を開く
    def open(self):
        #self.pos_pub.publish(VelocityCommand(f1 = 0, f2 = 0, f3 = 0, preshape = 0.0))
        self.pos_pub.publish(VelocityCommand(f1 = 0, f2 = 0, f3 = 0, preshape = 0.0))

    #指差す
    def pointing(self):
        #self.pos_pub.publish(VelocityCommand(f1 = 2, f2 = 3, f3 = 3, preshape = 0.0))
        self.pos_pub.publish(VelocityCommand(f1 = 2, f2 = 0, f3 = 3, preshape = 0.0))

#Ctrl+Cで停止
def stop_handler(signum, frame):
    print("\nget signal ctrl+C")
    server_sock.close()
    time.sleep(7)
    sys.exit()

signal.signal(signal.SIGTERM, stop_handler)
signal.signal(signal.SIGINT, stop_handler)

#non-blockingソケット通信

def main(srv):
    host = '127.0.0.1'
    port = 12345
    backlog = 10
    bufsize = 8192
    global server_sock
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setblocking(0)
    readfds = set([server_sock])
    try:
        server_sock.bind((host, port))
        server_sock.listen(backlog)
        print("OK")
        while True:
            rready, wready, xready = select.select(readfds, [], [])
            for sock in rready:
                if sock is server_sock:
                    conn, address = server_sock.accept()
                    readfds.add(conn)
                else:
                    msg = sock.recv(bufsize)
                    if len(msg) == 0:
                        sock.close()
                        readfds.remove(sock)
                    else:
                        msg = msg.rstrip()
                        print("receive command : %s" %msg)
                        modori = srv.make_command(msg)
                        conn.send(modori)
                        print(modori)
    finally:
        for sock in readfds:
            sock.close()
    return
"""
#blocking一応用意したけどLCoreで使うとエラー出るというか動かない
def main(srv):
    host = '127.0.0.1'
    port = 12345
    backlog = 10
    bufsize = 8192
    global server_sock
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.settimeout(None)
    #server_sock.setblocking(1)

    with closing(server_sock):
        server_sock.bind((host, port))
        server_sock.listen(backlog)
        print("OK")
        while True:
            conn, address = server_sock.accept()
            with closing(conn):
                while True:
                    msg = conn.recv(bufsize)
                    if not msg:
                        break
                    print("receive command : %s" %msg)
                    msg = msg.rstrip()
                    modori = srv.make_command(msg)
                    conn.send(modori)
    return
"""
if __name__ == "__main__":
    #ROS:ノード初期化
    rospy.init_node("robotsrv")
    #serverクラス初期化
    srv = server()
    #main()に入れる
    cmd = main(srv)
