import rclpy
from rclpy.node import Node
import threading
from std_msgs.msg import String
import requests
from fastapi import FastAPI
import uvicorn
from pydantic import BaseModel


app = FastAPI()

class Message(BaseModel):
    data: str

@app.post("/endpoint")
async def receive_data(data: Message):
    print(data.data)
    return {"message": "Data received"}


class WebServiceNode(Node):

    def __init__(self):
        super().__init__('web_service_node')
        self.publisher_ = self.create_publisher(String, 'web_service_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        message = "Hello World!"
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info('Published message: "%s"' % msg.data)
        self.publish_to_fastapi(msg.data)

    def publish_to_fastapi(self, data):
        url = "http://localhost:8000/endpoint"
        payload = {"data": data}
        response = requests.post(url, json=payload)
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
    
    uvicorn.run(app, host="localhost", port=8000, log_level="info")
    
    web_service_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
