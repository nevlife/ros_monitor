def process_camera_image(msg, window_name):
    """
    공통적인 카메라 이미지 처리 로직을 별도의 함수로 분리
    """
    try:
        # 메시지 데이터를 OpenCV 이미지로 변환
        np_arr = np.frombuffer(msg.data, np.uint8)
        camera_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # 유효성 검사 및 이미지 출력
        if camera_image is not None and camera_image.shape[0] > 0 and camera_image.shape[1] > 0:
            cv2.imshow(window_name, camera_image)
            cv2.waitKey(1)
        else:
            rospy.logwarn(f"Received invalid camera image in {window_name}")
        
        return camera_image
    except Exception as e:
        rospy.logerr(f"Error processing camera image in {window_name}: {e}")
        return None

# 콜백 함수들
def callback_usb_camera(self, msg):
    self.usb_camera = process_camera_image(msg, "USB Camera")

def callback_zed_node_left_image_rect_color(self, msg):
    self.zed_left_camera_img = process_camera_image(msg, "ZED Left Camera")

def callback_zed_node_depth_depth_registered(self, msg):
    self.zed_depth_camera_img = process_camera_image(msg, "ZED Depth Camera")
