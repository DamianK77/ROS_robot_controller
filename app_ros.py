from flask import Flask, Response, render_template
import cv2
import threading
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
import numpy as np

app = Flask(__name__)
current_frame_camera = None
current_frame_map = None
bridge = CvBridge() 

@app.route('/camera_feed')
def video_feed_camera():
    def generate():
        while True:
            if current_frame_camera is not None:
                _, buffer = cv2.imencode('.jpg', current_frame_camera)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/map_feed')
def video_feed_map():
    def generate():
        while True:
            if current_frame_map is not None:
                _, buffer = cv2.imencode('.jpg', current_frame_map)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template('index.html')

def update_camera_frame():
    global current_frame_camera
    def image_callback(msg):
        global current_frame_camera
        try:
            current_frame_camera = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print("Error converting image from ROS:", e)

    rospy.Subscriber("/camera/image_raw", Image, image_callback)
    rospy.spin()  

def update_map_frame():
    global current_frame_map
    def map_callback(msg):
        global current_frame_map
        try:
            map_data = np.array(msg.data, dtype=np.int8)
            width = msg.info.width
            height = msg.info.height

            map_image = map_data.reshape((height, width))

            current_frame_map = cv2.convertScaleAbs(map_image, alpha=255) 
        except Exception as e:
            print("Error converting map from ROS:", e)

    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.spin() 

if __name__ == '__main__':
    rospy.init_node('flask_ros_node', anonymous=True)

    threading.Thread(target=update_camera_frame, daemon=True).start()
    threading.Thread(target=update_map_frame, daemon=True).start()

    app.run(host='0.0.0.0', port=5000)
