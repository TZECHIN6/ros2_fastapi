import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from fastapi import FastAPI
from pydantic import BaseModel
import uvicorn
import requests
import threading
import json

app = FastAPI()

class Pose(BaseModel):
    x: float
    y: float
    z: float
    w: float

# @app.post('/pub_robot_status')
# async def publish_robot_status(pose: Pose):
#     print('Pose: ', pose)
#     return {'message': 'Received robot status'}


class WebServiceNode(Node):

    def __init__(self):
        super().__init__('web_service_node')
        #self.subscriptions = self.create_subscription()
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for Nav2 service...')
        self.latest_odom = None
        self.subscription = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.subscription
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.pub_robot_status)

        @app.get('/get_goal_pose')
        async def get_goal_pose(pose: Pose):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = pose.x
            pose_stamped.pose.position.y = pose.y
            pose_stamped.pose.position.z = pose.z
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = pose.w
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose_stamped
            goal_future = self.client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, goal_future)
            if goal_future.result() is not None:
                goal_handle = goal_future.result()
                if not goal_handle.accepted:
                    self.get_logger().warning('Goal rejected')
                    return {'message': 'Goal rejected'}
                else:
                    self.get_logger().info('Goal accepted')
                    return {'message': 'Goal accepted'}
            else:
                self.get_logger().error('Failed to send goal')
                return {'message': 'Failed to send goal'}

        @app.post('/pub_robot_status', response_model=Pose)
        async def post_robot_status(pose: Pose):
            print('Pose: ', pose)
            return {'message': 'Robot status received'}

    def odometry_callback(self, msg):
        # self.get_logger().info(f'Odometry: ({msg.pose.pose.position.x}, {msg.pose.pose.position.y})')
        self.latest_odom = (msg.pose.pose.position.x,
                            msg.pose.pose.position.y,
                            msg.pose.pose.position.z,
                            msg.pose.pose.orientation.w)
        
    def pub_robot_status(self):
        if self.latest_odom is not None:
            odom = Pose(x=self.latest_odom[0], y=self.latest_odom[1], z=self.latest_odom[2], w=self.latest_odom[3])
            odom_json = json.dumps(odom.dict(), ensure_ascii=False).replace('"', "'")
            print('odom_json: ', odom_json)
            #current_pose = {'x': 1.0, 'y': 2.0, 'z': 0.0, 'w':1.0}
            response = requests.post('http://localhost:8000/pub_robot_status', json=odom_json)
            #print('Response:', response)
            if response.status_code != 200:
                self.get_logger().error('Failed to publish to FastAPI endpoint: %s', response.text)


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


    # def get_goal_pose(self):
    #     url = "http://localhost:8000/get_goal_pose"
    #     response = requests.get(url)
    #     goal_pose = response.json()
    #     pose = PoseStamped()
    #     pose.header.frame_id = 'map'
    #     pose.pose.position.x = goal_pose['x']
    #     pose.pose.position.y = goal_pose['y']
    #     pose.pose.position.z = goal_pose['z']
    #     pose.pose.orientation.x = 0.0
    #     pose.pose.orientation.y = 0.0
    #     pose.pose.orientation.z = 0.0
    #     pose.pose.orientation.w = goal_pose['w']
    #     self.get_logger().info('Received goal pose: ')
    #     return pose

    # def navigate_to_pose(self):
    #     goal_pose = self.get_goal_pose()
    #     goal = NavigateToPose.Goal()
    #     goal.pose = goal_pose
    #     future = self.client.send_goal_async(goal)
    #     rclpy.spin_until_future_complete(self, future)
    #     result = future.result()
    #     print(result)

    # def publish_to_fastapi(self):
    #     # Get the current pose
    #     current_pose = {'x': 1.0, 'y': 2.0, 'z': 0.0, 'w':1.0}
    #     url = "http://localhost:8000/pub_robot_status"
    #     response = requests.post(url, json=current_pose)
    #     if response.status_code != 200:
    #         self.get_logger().error('Failed to publish to FastAPI endpoint: %s', response.text)

