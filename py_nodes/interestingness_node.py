'''
TODO: replace rospy.numpy_msg with ros2 equivalent
TODO: check if pillow (also PIL) needs to be added. PIL is for python2
'''
import os
import sys

import PIL
import torch
import torchvision.transforms as transforms
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError

from rosutil import msg_to_torch
from interestingness.torchutil import MovAvg, show_batch_box
from sensor_msgs.msg import Image
from interestingness_ros.msg import InterestInfo, UnInterests

int_ros_dir = get_package_share_directory('interestingness_ros')
int_dir = os.path.join(int_ros_dir,'interestingness')
sys.path.append(int_ros_dir)
sys.path.append(int_dir)


class InterestNode(Node):
    def __init__(self):
        super().__init__("interestingness_node")
        self._init_parameters()
        
        # Setup image processing steps
        self.bridge = CvBridge()
        self.movavg = MovAvg(self.window_size)
        self.transform = transforms.Compose([
            # VerticalFlip(), # Front camera of UGV0 in SubTF is mounted vertical flipped. Uncomment this line when needed.
            transforms.CenterCrop(self.crop_size),
            transforms.Resize((self.crop_size, self.crop_size)),
            transforms.ToTensor()]
        )
        self.normalize = transforms.Normalize(
            mean=[0.485, 0.456, 0.406], 
            std=[0.229, 0.224, 0.225]
        )
        
        # Load and setup model
        net = torch.load(self.model_save)
        net.set_train(False)
        net.memory.set_learning_rate(rr=self.rr, wr=self.wr)
        self.net = net.cuda() if torch.cuda.is_available() else net
        
        # Create publishers and subscribers
        for topic in self.image_topic:
            self.create_subscription(Image, topic, self.image_callback, 10)
        # self.create_subscription(numpy_msg(UnInterests), topic, self.interaction_callback, 10) # TODO
        self.frame_pub = self.create_publisher(Image, 'interestingness/image', 10) #publishes in image_callback
        self.info_pub = self.create_publisher(numpy_msg(InterestInfo), 'interestingness/info', 10) #publishes in image_callback


    def _init_parameters(self):
        # Declare by list of tuples, each (name, default, descriptor)
        self.declare_parameters(
            parameters=[
                ('image-topic', ['/rs_front/color/image']),
                ('interaction-topic', '/interaction/feature_map'), 
                ('data-root', '/data/datasets', ParameterDescriptor(description='dataset root folder')), #unused
                ('model-save', pack_path+'/saves/ae.pt.SubTF.n1000.mse', ParameterDescriptor(description='read model')),
                ('crop-size', 320, ParameterDescriptor(description='crop size')),
                ('num-interest', 10, ParameterDescriptor(description='loss compute by grid')), #unused
                ('skip-frames', 1, ParameterDescriptor(description='number of skip frame')), 
                ('window-size', 1, ParameterDescriptor(description='smooth window size >=1')),
                ('save-flag', 'interest', ParameterDescriptor(description='save name flag')), #unused
                ('rr', 5, ParameterDescriptor(description='reading rate')),
                ('wr', 5, ParameterDescriptor(description='writing rate')),
            ]
        )
        # Get value of parameters
        self.image_topic, self.interaction_topic, self.model_save, self.crop_size, self.skip_frames, self.window_size, self.rr, self.wr = [
                param.value for param in self.get_parameters(
                    'image_topic', 'interaction_topic', 'model_save', 'crop_size', 'skip_frames', 'window_size', 'rr', 'wr'
                )
        ]


    def image_callback(self, msg):
        '''
        Process input image and pass to model which returns a loss.
        Publishes processed image and info to 2 different topics.
        '''
        if msg.header.seq % self.skip_frames != 0:
            return
        self.get_logger.info(f"Received image {msg.header.frame_id}: {msg.header.seq}")
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            frame = PIL.Image.fromarray(frame)
            image = self.transform(frame)
            frame = self.normalize(image).unsqueeze(dim=0)
        except CvBridgeError:
            self.get_logger().error(CvBridgeError)
        else:
            frame = frame.cuda() if torch.cuda.is_available() else frame
            loss = self.net(frame)
            loss = self.movavg.append(loss)
            frame = 255 * show_batch_box(frame, msg.header.seq, loss.item(),show_now=False)
            frame_msg = self.bridge.cv2_to_imgmsg(frame.astype(np.uint8))
            info = InterestInfo()
            info.level = loss.item()
            info.image_shape = image.shape
            info.image = image.view(-1).numpy()
            info.shape = self.net.states.shape
            info.feature = self.net.states.cpu().view(-1).numpy()
            info.memory = self.net.coding.cpu().view(-1).numpy()
            info.reading_weights = self.net.memory.rw.cpu().view(-1).numpy()
            info.header = frame_msg.header = msg.header
            self.frame_pub.publish(frame_msg)
            self.info_pub.publish(info)

    def interaction_callback(self, msg):
        '''
        Writes information from uninteresting image into memory. 
        '''
        self.get_logger().info(f'Received uninteresting feature maps {msg.header.seq}')
        coding = msg_to_torch(msg.feature, msg.shape)
        coding = coding.cuda() if torch.cuda.is_available() else coding
        self.net.memory.write(coding)


def main(args=None):
    rclpy.init(args=args)
    node = InterestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()