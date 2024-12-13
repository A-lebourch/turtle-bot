from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
import time
from return_home import follow_me

class RobotNavigator(Node):
    def __init__(self):
        super().__init__('robot_navigator')
        self.current_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',  # Topic pour récupérer la position du robot
            self.amcl_pose_callback,
            QoSProfile(depth=10)  # QoS adapté pour un topic fréquent
        )

    def amcl_pose_callback(self, msg):
        self.current_pose = msg.pose.pose


def get_pose():
    # teleop.avance()
    navigator_node = RobotNavigator()
    print("Waiting for the robot's current pose...")
    while navigator_node.current_pose is None:
        time.sleep(0.1)
        rclpy.spin_once(navigator_node)

    print("Pose received. Initializing navigation...")
    return [navigator_node.current_pose.position.x, navigator_node.current_pose.position.y, navigator_node.current_pose.orientation.w]

def main():
    rclpy.init()
    navigator = BasicNavigator()
    init_pose = get_pose()
    print(init_pose)
    input("appuyer sur entrée pour start")

    # Attend que Nav2 soit actif
    navigator.waitUntilNav2Active()
    follow_me.main()
    # Définit une destination
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = init_pose[0]
    goal_pose.pose.position.y = init_pose[1]
    goal_pose.pose.orientation.w = init_pose[2]

    # Envoie la commande pour atteindre la destination
    navigator.goToPose(goal_pose)

    # Boucle pour suivre le progrès
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                f"Estimated time of arrival to destination: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.2f} seconds"
            )

    # Vérifie le résultat final
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal reached successfully!')
    elif result == TaskResult.CANCELED:
        print('Navigation was canceled.')
    elif result == TaskResult.FAILED:
        print('Navigation failed!')
    
    time.sleep(5)

    goal_pose.pose.position.x = -0.67
    goal_pose.pose.position.y = 5.87
    goal_pose.pose.orientation.w = 1.1

    # Envoie la commande pour atteindre la destination
    navigator.goToPose(goal_pose)
    # Boucle pour suivre le progrès
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                f"Estimated time of arrival to destination: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.2f} seconds"
            )

    # Vérifie le résultat final
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal reached successfully!')
    elif result == TaskResult.CANCELED:
        print('Navigation was canceled.')
    elif result == TaskResult.FAILED:
        print('Navigation failed!')

    # Arrêt du nœud
    rclpy.shutdown()