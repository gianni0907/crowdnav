import rospy
import cProfile
import crowd_navigation_msgs.msg
from crowd_navigation_core.Hparams import *

class TargetSetter:
    def __init__(self, hparams):
        self.hparams = hparams
        self.target_positions = hparams.target_positions
        self.current_index = 0
        self.pub = rospy.Publisher('/target_position', crowd_navigation_msgs.msg.TargetPosition, queue_size=1000)
        rospy.Subscriber('/target_achieved', crowd_navigation_msgs.msg.TargetAchieved, self.target_achieved_callback)

    def publish_next_target(self):
        if self.current_index < len(self.target_positions):
            target = self.target_positions[self.current_index]
            target_msg = crowd_navigation_msgs.msg.TargetPosition(target[0], target[1])
            self.pub.publish(target_msg)
            rospy.loginfo(f"Published target: {target}")
        else:
            rospy.loginfo("All targets published and achieved")

    def target_achieved_callback(self, msg):
        if msg.success:
            rospy.loginfo(f"Target {self.target_positions[self.current_index]} achieved.")
            self.current_index += 1
            self.publish_next_target()
    
    def run(self):
        if len(self.target_positions) == 0:
            rospy.logwarn("No target positions")
            return
        rate = rospy.Rate(1)
        while not(rospy.is_shutdown()):
            self.publish_next_target()
            rate.sleep()

def main():
    rospy.init_node('tiago_target_setter', log_level=rospy.INFO)
    rospy.loginfo('TIAGo target setter module [OK]')
    target_setter = TargetSetter(Hparams())
    prof_filename = '/tmp/target_setter.prof'
    cProfile.runctx(
        'target_setter.run()',
        globals=globals(),
        locals=locals(),
        filename=prof_filename
    )
