#!/usr/bin/python3

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import torch
import message_filters


sys.path.append("/home/titan/drone_ws/src/drone_sim/MiDaS")
from midas.model_loader import default_models, load_model


def process(device, model, model_type, image, input_size, target_size, optimize, use_camera):
    """
    Run the inference and interpolate.

    Args:
        device (torch.device): the torch device used
        model: the model used for inference
        model_type: the type of the model
        image: the image fed into the neural network
        input_size: the size (width, height) of the neural network input (for OpenVINO)
        target_size: the size (width, height) the neural network output is interpolated to
        optimize: optimize the model to half-floats on CUDA?
        use_camera: is the camera used?

    Returns:
        the prediction
    """
    # global first_execution


    sample = torch.from_numpy(image).to(device).unsqueeze(0)

    if optimize and device == torch.device("cuda"):
        # if first_execution:
            # print("  Optimization to half-floats activated. Use with caution, because models like Swin require\n"
            #         "  float precision to work properly and may yield non-finite depth values to some extent for\n"
            #         "  half-floats.")
        sample = sample.to(memory_format=torch.channels_last)
        sample = sample.half()


    height, width = sample.shape[2:]
    # print(f"    Input resized to {width}x{height} before entering the encoder")
    # first_execution = False

    prediction = model.forward(sample)
    prediction = (
        torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=target_size[::-1],
            mode="bicubic",
            align_corners=False,
        )
        .squeeze()
        .cpu()
        .numpy()
    )

    return prediction


rospy.init_node('opencv', anonymous=True)

torch.cuda.empty_cache()
model_type = "dpt_swin2_large_384"
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model_path = "/home/titan/drone_ws/src/drone_sim/MiDaS/weights/dpt_swin2_large_384.pt"
optimize=False
side=False
height=None
square=False
grayscale=False

bridge = CvBridge()
model, transform, net_w, net_h = load_model(device, model_path, model_type, optimize, height, square)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


# from midas.model_loader import default_models, load_model



# print("Device: %s" % device)

# model, transform, net_w, net_h = load_model(device, model_path, model_type, optimize, height, square)
pub = rospy.Publisher("cam_depth",Image,queue_size=10)
pubrgb = rospy.Publisher("cam_rgb",Image,queue_size=10)
pubinfo = rospy.Publisher("cam_info",CameraInfo,queue_size=10)
rate  = rospy.Rate(10)
while not rospy.is_shutdown():

    with torch.no_grad():
        data = rospy.wait_for_message("/webcam/image_raw", Image)
        try:
            img = bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError:
            rospy.logerr("CvBawridge Error: {0}".format(e))

        
        camera_info  = rospy.wait_for_message("/webcam/camera_info", CameraInfo)
        #print(np.array(camera_info))
        # cap=cv2.VideoCapture(0)
        # ret, frame = cap.read()
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = np.array(img)
        original_image_rgb = np.flip(frame, 2)  # in [0, 255] (flip required to get RGB)
        image = transform({"image": original_image_rgb/255})["image"]
    

        depth = process(device, model, model_type, image, (net_w, net_h),
                                            original_image_rgb.shape[1::-1], optimize, True)
        # for x in range(480):
        #     for y in range(640):
        #         if depth[x][y]  <= 100:
        #             depth[x][y] = 100
        #         elif depth[x][y]  >= 8000:
        #             depth[x][y] = 8000
        depth[depth <= 0] = 0
        depth[depth >= 3000] = 3000
        
       
     
        depth_min = 00
        depth_max = 3000
        
        normalized_depth = 255 * (depth) / (depth_max-depth_min)
        normalized_depth *= 2
        
        right_side = np.repeat(np.expand_dims(normalized_depth, 2), 3, axis=2) / 3
        right_side = cv2.cvtColor(right_side,cv2.COLOR_BGR2GRAY)
        right_side = 255-right_side
        #right_side = right_side.astype(np.float32)
        #right_side = cv2.normalize(right_side, None, 0, 20, cv2.NORM_MINMAX)
        
       


        #right_side = cv2.applyColorMap(np.uint8(right_side), cv2.COLORMAP_BONE)
        #right_side = np.uint16(right_side)
        
        ros_image = bridge.cv2_to_imgmsg(right_side,"32FC1")
   
        
        ros_image_rgb = bridge.cv2_to_imgmsg(original_image_rgb,"bgr8")
        
        #both = np.concatenate((original_image_rgb, right_side), axis=0)
        #print(ros_image)
        curtime= rospy.Time.now()
        ros_image.header.stamp = curtime
        ros_image.header.frame_id = "camera_link"
        
        ros_image_rgb.header.frame_id = "camera_link"
        ros_image_rgb.header.stamp = curtime
        camera_info.header.stamp = curtime
        pub.publish(ros_image)
        pubrgb.publish(ros_image_rgb)
        pubinfo.publish(camera_info)
        
        #ts = message_filters.ApproximateTimeSynchronizer([ros_image, ros_image_rgb,camera_info], 10,10, 0.2)
#         cv2.imshow("",both)

#         if cv2.waitKey(1) & 0xFF == ord('q') :
#                 break

cv2.destroyAllWindows()
