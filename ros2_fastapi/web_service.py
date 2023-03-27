import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from fastapi import FastAPI
from pydantic import BaseModel
import uvicorn
import requests
import threading

from std_msgs.msg import String

app = FastAPI()

class Pose(BaseModel):
    x: float
    y: float
    z: float
    w: float

class WebServiceNode(Node):

    def __init__(self):
        super().__init__('web_service_node')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for Nav2 service...')
        self.subscription = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.subscription
        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self.pub_robot_status)

        @app.get('/get_goal_pose')
        async def get_goal_pose(pose: Pose):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = pose.x
            pose_stamped.pose.position.y = pose.y
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = pose.z
            pose_stamped.pose.orientation.w = pose.w

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose_stamped
            goal_future = self.client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, goal_future)
            goal_handle = goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().warning('Goal rejected')
                return {'message': 'Goal rejected'}
            else:
                self.get_logger().info('Goal accepted')
                
            goal_state = goal_handle.status
            if goal_state == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Goal succeeded')
                return {'message': 'Goal succeeded'}
            elif goal_state == GoalStatus.STATUS_ABORTED:
                self.get_logger().warning('Goal aborted')
                return {'message': 'Goal aborted'}
            else:
                self.get_logger().info('goal_state: {}'.format(goal_state))

        @app.post('/pub_robot_status')
        async def post_robot_status(pose: Pose):
            print('Pose: ', pose)
            return {'message': 'Robot status received'}

    def odometry_callback(self, msg):
        # self.get_logger().info(f'Odometry: ({msg.pose.pose.position.x}, {msg.pose.pose.position.y})')
        self.latest_odom = (msg.pose.pose.position.x,
                            msg.pose.pose.position.y,
                            msg.pose.pose.orientation.z,
                            msg.pose.pose.orientation.w)
        
    def pub_robot_status(self):
        if self.latest_odom is not None:
            odom = Pose(x=self.latest_odom[0], y=self.latest_odom[1], z=self.latest_odom[2], w=self.latest_odom[3])
            odom_dict = odom.dict()
            response = requests.post('http://localhost:8000/pub_robot_status', json=odom_dict)

            # odom = Pose(x=self.latest_odom[0], y=self.latest_odom[1], z=self.latest_odom[2], w=self.latest_odom[3])
            # odom_json = json.dumps(odom.dict(), allow_nan=False, ensure_ascii=False)
            # headers = {'Content-Type': 'application/json'}
            # response = requests.post('http://localhost:8000/pub_robot_status', data=odom_json, headers=headers)
            
            if response.status_code != 200:
                self.get_logger().error('Failed to publish to FastAPI endpoint: %s', response.text)

    # def goal_state_callback(self, goal_future):
    #     goal_handle = goal_future.result()
    #     self.goal_state = goal_handle.status
    #     self.get_logger().info('goal_state: {}'.format(self.goal_state))
    #     if self.goal_state == GoalStatus.STATUS_SUCCEEDED:
    #         self.get_logger().info('Goal succeeded')
    #         return {'message': 'Goal succeeded'}
    #     elif self.goal_state == GoalStatus.STATUS_ABORTED:
    #         self.get_logger().warning('Goal aborted')
    #         return {'message': 'Goal aborted'}
    #     else:
    #         self.get_logger().info('goal_state: {}'.format(self.goal_state))

def main(args=None):
    rclpy.init(args=args)
    web_service_node = WebServiceNode()
    # Multi-threading is needed for both ROS2 node and uvicorn
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(web_service_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    uvicorn.run(app, host="localhost", port=8000)
    
    web_service_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

