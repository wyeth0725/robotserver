import struct
import sys

import baxter_interface
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


class IKsolve:
    def __init__(self,arm):
        self.limb = arm
        ns = "ExternalTools/" + self.limb + "/PositionKinematicsNode/IKService"
        rospy.wait_for_service(ns, 5.0)
        self.iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        #self.FAIL = '\033[91m'

    def solve(self,coordinate): #第3引数にQuaternion追加　[x,y,z,qx,qy,qz,qw]のような形になっているとしたら必要なし

        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = {
            'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    #length
                    x=coordinate[0],
                    #width
                    y=coordinate[1],
                    #height
                    z=coordinate[2],
                ),
                orientation=Quaternion(
                    x=-0.3816802752430421,
                    y=0.9225560682320805,
                    z=-0.023329563346999555,
                    w=0.05163525861849474,
                ),
            ),
        ),
            'right': PoseStamped( #右手の座標とQuaternion
                header = hdr,
                pose=Pose(
                    position=Point(
                        #length
                        x=coordinate[0],
                        #width
                        y=coordinate[1],
                        #height
                        z=coordinate[2],
                    ),
                    orientation=Quaternion(     #ここにxyzと同じような形で指定
                                                #第2引数と同じ変数名の中にあるとしたら x = coordinate[3] y = coordinate[4]のように指定
                        x=0,
                        y=0.707106781186547524,
                        z=0,
                        w=0.707106781186547524,
                        #x = 0.707106781186547524,
                        #y = 0,
                        #z = 0.707106781186547524,
                        #w = 0,
                    ),
                ),
            ),
        }

        ikreq.pose_stamp.append(poses[self.limb])
        try:
            resp = self.iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                        }.get(resp_seeds[0], 'None')
            print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                        (seed_str,))
            self.limb_joints = dict(zip(resp.joints[0].name,
                                        resp.joints[0].position))
            #print ("Response Message:\n", resp)
            return self.limb_joints
        else:#解けなかった場合 メッセージ表示して辞書型の関節角度ではなく、スカラーの0を返す
            print("INVALID POSE - No Valid Joint Solution Found.")
            return 0
