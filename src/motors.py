#!/usr/bin/env python
#encoding: utf8
import sys, rospy, math, tf
from pimouse_ros.msg import MotorFreqs
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.srv import TimedMotion                                               #追加
from nav_msgs.msg import Odometry

class Motor():
    def __init__(self):
        if not self.set_power(False): sys.exit(1)   #モータの電源を切る（TrueをFalseに）

        rospy.on_shutdown(self.set_power)
        self.sub_raw = rospy.Subscriber('motor_raw', MotorFreqs, self.callback_raw_freq)
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.callback_cmd_vel)
        self.srv_on = rospy.Service('motor_on', Trigger, self.callback_on)
        self.srv_off = rospy.Service('motor_off', Trigger, self.callback_off)
        self.srv_tm = rospy.Service('timed_motion', TimedMotion, self.callback_tm)    #追加
        self.last_time = rospy.Time.now()
        self.using_cmd_vel = False

        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)  # odomというトピックの準備 型はnav_msgs/Odometry
        self.bc_odom = tf.TransformBroadcaster() # tfに情報を送るパブリッシャのようなものを作成

        self.x, self.y, self.th = 0.0, 0.0, 0.0  # デットレコニングに基づいて計算したロボットの現在位置、向き(姿勢)を記録する変数
        self.vx, self.vth = 0.0, 0.0 # ロボットの速度と角速度を格納する変数 cmd_velのメッセージが届いた時にそのまま値を代入する

        self.cur_time = rospy.Time.now() # オドメトリや座標の情報を送る時の時刻
        self.last_time = self.cur_time

    def set_power(self,onoff=False):
        en = "/dev/rtmotoren0"
        try:
            with open(en,'w') as f:
                f.write("1\n" if onoff else "0\n")
            self.is_on = onoff
            return True
        except:
            rospy.logerr("cannot write to " + en)

        return False

    def set_raw_freq(self,left_hz,right_hz):
        if not self.is_on:
            rospy.logerr("not enpowered")
            return

        try:
            with open("/dev/rtmotor_raw_l0",'w') as lf,\
                 open("/dev/rtmotor_raw_r0",'w') as rf:
                lf.write(str(int(round(left_hz))) + "\n")
                rf.write(str(int(round(right_hz))) + "\n")
        except:
            rospy.logerr("cannot write to rtmotor_raw_*")

    def callback_raw_freq(self,message):
        self.set_raw_freq(message.left_hz,message.right_hz)

    def callback_cmd_vel(self,message):
        if not self.is_on:
            return
        self.vx = message.linear.x
        self.vth = message.angular.z

        forward_hz = 80000.0*message.linear.x/(9*math.pi)
        rot_hz = 400.0*message.angular.z/math.pi
        self.set_raw_freq(forward_hz-rot_hz, forward_hz+rot_hz)

        self.using_cmd_vel = True
        self.last_time = rospy.Time.now()

    def onoff_response(self,onoff):                                #以下3つのメソッドを追加
        d = TriggerResponse()
        d.success = self.set_power(onoff)
        d.message = "ON" if self.is_on else "OFF"
        return d

    def callback_on(self,message): return self.onoff_response(True)
    def callback_off(self,message): return self.onoff_response(False)

    def callback_tm(self,message):
        if not self.is_on:
            rospy.logerr("not enpowered")
            return False

        dev = "/dev/rtmotor0"
        try:
            with open(dev,'w') as f:
                f.write("%d %d %d\n" %
                    (message.left_hz,message.right_hz,message.duration_ms))
        except:
            rospy.logerr("cannot write to " + dev)
            return False

        return True

    def send_odom(self):
        self.cur_time = rospy.Time.now()

        dt = self.cur_time.to_sec() - self.last_time.to_sec() # 前回の処理から何秒たったか to_secはcur_time,last_timeのクラスrospy.Timeのメソッド
        self.x += self.vx * math.cos(self.th) * dt # 以下３行は現在の速度角速度からデッドレコニングで計算したロボットの姿勢の値を更新 dtが大きいと誤差が大きくなる
        self.y += self.vx * math.sin(self.th) * dt
        self.th += self.vth * dt

        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.bc_odom.sendTransform((self.x, self.y, 0.0), q, self.cur_time, "base_link", "odom") # tfに情報を送る

        odom = Odometry() # Odeometry型のインスタンス作成
        odom.header.stamp = self.cur_time # 時刻の指定
        odom.header.frame_id = "odom" # 親フレームの指定
        odom.child_frame_id = "base_link" # 子フレームの指定

        odom.pose.pose.position = Point(self.x, self.y, 0) # 以下２行で親フレームに対する子フレームの相対姿勢をセット
        odom.pose.pose.orientation = Quaternion(*q) # *qの型をodom.pose.pose.orientationのQuaternion型に変換 *q(リスト)の中身を分解して引数として渡す

        odom.twist.twist.linear.x = self.vx # 以下３行で子フレームで見た時の小フレームの運動をセット
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth

        self.pub_odom.publish(odom) # オドメトリの情報をpublish

        self.last_time = self.cur_time

if __name__ == '__main__':
    rospy.init_node('motors')
    m = Motor()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
#        if m.using_cmd_vel and rospy.Time.now().to_sec() - m.last_time.to_sec() >= 1.0:
#            m.set_raw_freq(0,0)
#            m.using_cmd_vel = False
#        rate.sleep()
        m.send_odom()  # Motorクラスのsend_odom(オドメトリ情報)メソッドを定期的に呼び出している処理
        rate.sleep()

# Copyright 2016 Ryuichi Ueda
# Released under the BSD License.
# To make line numbers be identical with the book, this statement is written here. Don't move it to the header.
