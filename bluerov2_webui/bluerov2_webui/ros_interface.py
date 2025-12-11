import base64
import cv2
import asyncio
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# --- IMPORTS FOR DYNAMIC MESSAGING ---
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rcl_interfaces.srv import SetParameters, ListParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

class ROVNode(Node):
    def __init__(self):
        super().__init__('rov_webui_node')
        
        self.bridge = CvBridge()
        
        # --- IMAGE BUFFERS ---
        self.latest_image_raw = None
        self.latest_image_processed = None 
        self.latest_image_servo = None # <--- NEW: Visual Servo Buffer

        # --- SUBSCRIBERS ---
        self.create_subscription(Image, 'camera_detections', self._processed_callback, 10)
        self.create_subscription(Image, 'camera/image', self._raw_callback, 10)
        
        # NEW: Visual Servo Subscriber
        self.create_subscription(Image, 'visual_servo_image', self._servo_callback, 10)

        self.dynamic_subs = {} 

    # --- CALLBACKS ---
    def _processed_callback(self, msg): self.latest_image_processed = self._process_msg(msg)
    def _raw_callback(self, msg): self.latest_image_raw = self._process_msg(msg)
    
    # NEW: Servo Callback
    def _servo_callback(self, msg): self.latest_image_servo = self._process_msg(msg)

    def _process_msg(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buffer = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            b64_string = base64.b64encode(buffer).decode('utf-8')
            return f'data:image/jpeg;base64,{b64_string}'
        except: return None

    # ... (Keep all the dynamic parameter/topic functions exactly as they were) ...
    # (Copy the rest of the file from Phase 4 here)
    def get_all_topics(self): return self.get_topic_names_and_types()
    
    def subscribe_dynamic(self, topic_name, msg_type_str, callback):
        if topic_name in self.dynamic_subs:
            self.destroy_subscription(self.dynamic_subs[topic_name])
            del self.dynamic_subs[topic_name]
        try: msg_class = get_message(msg_type_str)
        except LookupError: return False
        def wrapped_callback(msg): callback(message_to_ordereddict(msg))
        self.dynamic_subs[topic_name] = self.create_subscription(msg_class, topic_name, wrapped_callback, 10)
        return True

    async def _wait_for_future(self, ros_future):
        loop = asyncio.get_event_loop()
        aio_future = loop.create_future()
        def on_done(f):
            try: loop.call_soon_threadsafe(aio_future.set_result, f.result())
            except Exception as e: loop.call_soon_threadsafe(aio_future.set_exception, e)
        ros_future.add_done_callback(on_done)
        return await aio_future

    def get_active_nodes(self): return self.get_node_names_and_namespaces()
    
    async def get_node_parameters(self, node_name):
        list_client = self.create_client(ListParameters, f'/{node_name}/list_parameters')
        if not list_client.service_is_ready(): return None
        list_req = ListParameters.Request()
        try: list_resp = await self._wait_for_future(list_client.call_async(list_req))
        except: return None
        param_names = list_resp.result.names
        if not param_names: return {}
        get_client = self.create_client(GetParameters, f'/{node_name}/get_parameters')
        get_req = GetParameters.Request()
        get_req.names = param_names
        try: get_resp = await self._wait_for_future(get_client.call_async(get_req))
        except: return None
        params = {}
        for name, p_value in zip(param_names, get_resp.values):
            if p_value.type == ParameterType.PARAMETER_BOOL: params[name] = p_value.bool_value
            elif p_value.type == ParameterType.PARAMETER_INTEGER: params[name] = p_value.integer_value
            elif p_value.type == ParameterType.PARAMETER_DOUBLE: params[name] = p_value.double_value
            elif p_value.type == ParameterType.PARAMETER_STRING: params[name] = p_value.string_value
        return params

    def set_remote_parameter(self, node_name, param_name, value):
        service_name = f'/{node_name}/set_parameters'
        client = self.create_client(SetParameters, service_name)
        if not client.service_is_ready(): return
        val_msg = ParameterValue()
        if isinstance(value, bool): val_msg.type = ParameterType.PARAMETER_BOOL; val_msg.bool_value = value
        elif isinstance(value, int): val_msg.type = ParameterType.PARAMETER_INTEGER; val_msg.integer_value = value
        elif isinstance(value, float): val_msg.type = ParameterType.PARAMETER_DOUBLE; val_msg.double_value = value
        elif isinstance(value, str): val_msg.type = ParameterType.PARAMETER_STRING; val_msg.string_value = value
        new_param = Parameter(); new_param.name = param_name; new_param.value = val_msg
        req = SetParameters.Request(); req.parameters = [new_param]; client.call_async(req)