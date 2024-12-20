from flask import Flask, Response, render_template
import cv2
import threading

app = Flask(__name__)
current_frame_camera = None
current_frame_map = None

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
    while True:
        current_frame_camera = cv2.imread('camera_image.jpg') 

def update_map_frame():
    global current_frame_map
    while True:
        current_frame_map = cv2.imread('map_image.jpg')

if __name__ == '__main__':
    threading.Thread(target=update_camera_frame, daemon=True).start()
    threading.Thread(target=update_map_frame, daemon=True).start()

    app.run(host='0.0.0.0', port=5000)
