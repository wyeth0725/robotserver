#!/usr/bin/env python
import sys
import rospy
import baxter_interface


class Mover:
    def __init__(self):
        print("Mover init")

    def try_float(self,x):
        try:
            return float(x)
        except ValueError:
            return None

    # 時系列込で座標のリストから軌道生成
    # 右腕しか使わないので右のみ
    # right : 腕 baxter_interface.Limb()の変数
    # cdn_list :　三次元座標のリスト cdn_list = [x1,y1,z1],[x2,y2,z2],...
    # IKservice : 逆運動学のクラス
    # loops : 動作のループ回数　デフォルト1回

    #やっていることは改造前のjoint_position_file_playback.pyに対して与えるはずのrecordファイルを
    #ファイルに書きだした時間と関節角でやるのではなく座標と自分で決めた時間で、座標については逐次IK解きながら
    #同じ機能を使っているだけ
    #時系列無しだとカクつく
    def map_file(self, right, cdn_list, IKservice, loops=1):

        rate = rospy.Rate(1000)


        #print("Playing back: %s" % ("record.rec",))

        #joint.txtは以下の1行を書いているだけ    recordファイルと形を合わせるため
        #time,right_s0,right_s1,right_e0,right_e1,right_w0,right_w1,right_w2
        #ファイルから読み込む必要はまったくないしパスの指定方法どうにかする
        with open("/home/moriya/catkin_ws/lcore-robotd/joint.txt", 'r') as f:
            lines = f.readlines()
        keys = lines[0].rstrip().split(',')

        l = 0

        while loops < 1 or l < loops:
            i = 0
            l += 1
            t = 0.0
            print("Moving to start position...")

            rcmd_start = IKservice.solve(cdn_list[0]) #開始位置のIK　座標の0番を解く
            right.move_to_joint_positions(rcmd_start, 0.5) #移動
            start_time = rospy.get_time()
            for values in cdn_list[1:]: #0が開始地点ですでに移動済みなので1から
                i += 1
                t += 0.02337099499 * 2 #時系列 点と点の間の時間
                                       #サンプルプログラムにあるjoint_recorder.pyで生成されるrecordファイルのtimeにあたるもの
                                       #間の時間が短すぎた感あるので2倍してみる
                loopstr = str(loops) if loops > 0 else "forever"        #
                sys.stdout.write("\r Record %d of %d, loop %d of %s" %  #ここ4行は動作には関係なし
                                (i, len(cdn_list) - 1, l, loopstr))     #
                sys.stdout.flush()                                      #
                rcmd = IKservice.solve(cdn_list[i]) ###IKといて
                values = [t]
                values.extend(rcmd.values()) #時間+関節角
                                             #time,right_s0,right_s1,right_e0,right_e1,right_w0,right_w1,right_w2
                                             #0.023... s0角度 s1角度 ...
                while (rospy.get_time() - start_time) < values[0]: ###時間使いながら
                    if rospy.is_shutdown():
                        print("\n Aborting - ROS shutdown")
                        return False
                    if len(cdn_list):
                        right.set_joint_positions(rcmd) ###解いた関節角に移動でループ
                    rate.sleep()
            return True
