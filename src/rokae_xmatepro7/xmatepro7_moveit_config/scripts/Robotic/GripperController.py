import zmq
import orjson  # 更快的JSON序列化库
import time
import threading
import json

class DHGripperControlInterface:
    def __init__(self, bind_ip='127.0.0.1', bind_port='5557'):
        """
        初始化 ZMQSender 类，设置绑定参数并启动发布套接字。

        :param bind_ip: 绑定的 IP 地址，默认为 '*'（监听所有接口）
        :param bind_port: 绑定的端口号，默认为 '5555'
        """
        self.bind_ip = bind_ip
        self.bind_port = bind_port
        self.endpoint = f"tcp://{self.bind_ip}:{self.bind_port}"
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(self.endpoint)
        print(f"ZMQSender initialized and bound to {self.endpoint}")

        # 等待一段时间，确保订阅者有时间连接
        time.sleep(1)

    def sendCmd(self, message_dict):
        """
        接受一个字典，将其转换为 JSON 字符串并发送。

        :param message_dict: 要发送的字典消息
        """
        if not isinstance(message_dict, dict):
            raise ValueError("message_dict 必须是一个字典")

        try:
            # 使用orjson进行更快的JSON序列化
            message_json = orjson.dumps(message_dict).decode('utf-8')
            self.socket.send_string(message_json, zmq.NOBLOCK)
        except zmq.Again:
            # 发送缓冲区已满，消息被丢弃
            pass
        except Exception as e:
            # 在生产环境中，建议使用日志记录而非print
            pass

    def disconnect(self):
        """
        关闭发送器，释放资源。
        """
        self.socket.close(linger=0)  # 立即关闭套接字
        self.context.term()
        print("ZMQSender closed and resources cleaned up.")

    def __del__(self):
        """
        析构函数，确保资源释放。
        """
        self.disconnect()

class DHGripperReceiveInterface:
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

    def getGripperPosition(self):
        with self.lock:
            # print(self.latest_message)
            return self.latest_message.get("GripperState"), self.latest_message.get("GripperPose")

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