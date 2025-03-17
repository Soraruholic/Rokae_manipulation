import numpy as np
import zmq
import json
import threading
import time

class RokaeReceiveInterface:
    def __init__(self, hostname='127.0.0.1', sender_port="5556"):
        self.sender_endpoint = f"tcp://{hostname}:{sender_port}"
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(self.sender_endpoint)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")

        self.latest_message = {}
        self.lock = threading.Lock()
        self.running = True

        # 初始化 Poller
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

        self.listener_thread = threading.Thread(target=self._listen)
        self.listener_thread.daemon = True
        self.listener_thread.start()
        print(f"ZMQReceiver initialized and connected to {self.sender_endpoint}")

    def _listen(self):
        while self.running:
            try:
                # Poll 1000毫秒（1秒）超时
                socks = dict(self.poller.poll(1000))
                if self.socket in socks and socks[self.socket] == zmq.POLLIN:
                    message = self.socket.recv_string()
                    data = json.loads(message)
                    with self.lock:
                        self.latest_message = data
                    # print(f"Received and updated message: {data}")
            except json.JSONDecodeError as e:
                print(f"JSON 解码错误: {e}")
            except zmq.ZMQError as e:
                if not self.running:
                    break
                print(f"ZMQ 错误: {e}")
            except Exception as e:
                print(f"接收消息时发生错误: {e}")

    def getRobotState(self):
        tcp_pose = self.latest_message.get('ActualTCPPose')
        tcp_pose_np = np.asarray(tcp_pose)
        joints_state = self.latest_message.get('ActualJointPose')
        joints_state_np = np.asarray(joints_state)
        gripper_pose = self.latest_message.get('GripperPose')
        gripper_state = self.latest_message.get('GripperState')

        return tcp_pose_np, joints_state_np, gripper_pose, gripper_state

    def getField(self, field_name):
        with self.lock:
            return self.latest_message.get(field_name)

    def disconnect(self):
        self.running = False
        self.poller.unregister(self.socket)
        self.socket.close(linger=0)
        self.context.term()
        self.listener_thread.join()
        # print("ZMQReceiver stopped and resources cleaned up.")

    def __del__(self):
        self.disconnect()

class RokaeControlInterface:
    def __init__(self, bind_ip='127.0.0.1', bind_port='5555'):
        self.bind_ip = bind_ip
        self.bind_port = bind_port
        self.endpoint = f"tcp://{self.bind_ip}:{self.bind_port}"
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(self.endpoint)
        print(f"RokaeControlInterface initialized and bound to {self.endpoint}")

        # 等待一段时间，确保订阅者有时间连接
        time.sleep(1)

    def sendCmd(self, message_dict):
        if not isinstance(message_dict, dict):
            raise ValueError("message_dict 必须是一个字典")
        try:
            message_json = json.dumps(message_dict)
            self.socket.send_string(message_json)  # 去掉 zmq.NOBLOCK
        except Exception as e:
            print(f"Error sending message: {e}")

    def disconnect(self):
        self.socket.close(linger=0)
        self.context.term()
        print("RokaeControlInterface closed and resources cleaned up.")

    def __del__(self):
        self.disconnect()
