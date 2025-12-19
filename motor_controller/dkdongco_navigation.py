#!/usr/bin/env python3
import pigpio
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ================= GPIO MOTOR =================
PIN_L_FWD = 22
PIN_L_REV = 23
PIN_R_FWD = 24
PIN_R_REV = 25
PWM_FREQ = 100

# ================= POWER =================
PWM_MAX  = 255
PWM_BASE = 250
PWM_MIN  = 100

# ================= ROBOT =================
WHEEL_BASE = 0.08

# ================= TUNING =================
LIN_GAIN = 1.0
ANG_GAIN = 1.0
ANG_DEADZONE = 0.08

SPIN_THRESH = 0.6
SPIN_PWM = 100

CMD_TIMEOUT = 0.3

# ================= BUTTON & LED =================
BTN1 = 5
BTN2 = 6
LED1 = 12
LED2 = 13


class MotorNode:
    def __init__(self):
        rospy.init_node("motor_node_nav_led")

        # ---------- pigpio ----------
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio not running (sudo pigpiod)")

        # ---------- Motor GPIO ----------
        for p in [PIN_L_FWD, PIN_L_REV, PIN_R_FWD, PIN_R_REV]:
            self.pi.set_mode(p, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(p, PWM_FREQ)
            self.pi.set_PWM_dutycycle(p, 0)

        # ---------- LED GPIO ----------
        for led in [LED1, LED2]:
            self.pi.set_mode(led, pigpio.OUTPUT)
            self.pi.set_PWM_dutycycle(led, 0)

        # ---------- Button GPIO (PULL-UP) ----------
        for btn in [BTN1, BTN2]:
            self.pi.set_mode(btn, pigpio.INPUT)
            self.pi.set_pull_up_down(btn, pigpio.PUD_UP)

        # ---------- LED states ----------
        self.led1_state = False
        self.led2_state = False

        # ---------- Button interrupts ----------
        self.cb_btn1 = self.pi.callback(BTN1, pigpio.FALLING_EDGE, self.btn1_cb)
        self.cb_btn2 = self.pi.callback(BTN2, pigpio.FALLING_EDGE, self.btn2_cb)

        # ---------- ROS ----------
        self.v = 0.0
        self.w = 0.0
        self.last_cmd = rospy.Time.now()

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb)
        self.led_pub = rospy.Publisher("/led_state", String, queue_size=10)

        rospy.Timer(rospy.Duration(0.02), self.update)  # 50 Hz

        rospy.loginfo("Motor + LED node READY")

    # ================= CMD_VEL =================
    def cmd_cb(self, msg):
        self.v = msg.linear.x * LIN_GAIN
        self.w = msg.angular.z * ANG_GAIN
        self.last_cmd = rospy.Time.now()

    # ================= MAIN LOOP =================
    def update(self, event):
        if (rospy.Time.now() - self.last_cmd).to_sec() > CMD_TIMEOUT:
            self.stop()
            return

        v = self.v
        w = self.w

        if abs(w) < ANG_DEADZONE:
            w = 0.0

        if abs(w) > SPIN_THRESH:
            self.spin(w)
            return

        v_l = v - w * WHEEL_BASE / 2.0
        v_r = v + w * WHEEL_BASE / 2.0
        self.drive(v_l, v_r)

    # ================= DRIVE =================
    def pwm(self, v):
        if abs(v) < 0.01:
            return 0
        p = int(PWM_BASE * min(1.0, abs(v)))
        return max(PWM_MIN, min(PWM_MAX, p))

    def drive(self, vl, vr):
        pl = self.pwm(vl)
        pr = self.pwm(vr)
        self.pi.set_PWM_dutycycle(PIN_L_FWD, pl if vl > 0 else 0)
        self.pi.set_PWM_dutycycle(PIN_L_REV, pl if vl < 0 else 0)
        self.pi.set_PWM_dutycycle(PIN_R_FWD, pr if vr > 0 else 0)
        self.pi.set_PWM_dutycycle(PIN_R_REV, pr if vr < 0 else 0)

    # ================= SLOW SPIN =================
    def spin(self, w):
        p = SPIN_PWM
        if w > 0:
            self.pi.set_PWM_dutycycle(PIN_L_FWD, p)
            self.pi.set_PWM_dutycycle(PIN_R_REV, p)
            self.pi.set_PWM_dutycycle(PIN_L_REV, 0)
            self.pi.set_PWM_dutycycle(PIN_R_FWD, 0)
        else:
            self.pi.set_PWM_dutycycle(PIN_L_REV, p)
            self.pi.set_PWM_dutycycle(PIN_R_FWD, p)
            self.pi.set_PWM_dutycycle(PIN_L_FWD, 0)
            self.pi.set_PWM_dutycycle(PIN_R_REV, 0)

    # ================= STOP =================
    def stop(self):
        for p in [PIN_L_FWD, PIN_L_REV, PIN_R_FWD, PIN_R_REV]:
            self.pi.set_PWM_dutycycle(p, 0)

    # ================= BUTTON CALLBACK =================
    def btn1_cb(self, gpio, level, tick):
        self.led1_state = not self.led1_state
        self.pi.set_PWM_dutycycle(LED1, 255 if self.led1_state else 0)
        msg = f"LED1 {'ON' if self.led1_state else 'OFF'}"
        self.led_pub.publish(msg)
        rospy.loginfo(msg)

    def btn2_cb(self, gpio, level, tick):
        self.led2_state = not self.led2_state
        self.pi.set_PWM_dutycycle(LED2, 255 if self.led2_state else 0)
        msg = f"LED2 {'ON' if self.led2_state else 'OFF'}"
        self.led_pub.publish(msg)
        rospy.loginfo(msg)


# ================= MAIN =================
if __name__ == "__main__":
    try:
        node = MotorNode()
        rospy.spin()
    finally:
        node.stop()
        node.pi.stop()
