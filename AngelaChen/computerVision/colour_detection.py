import cv2
import numpy as np
from flask import Flask, Response

app = Flask(__name__)
cap = cv2.VideoCapture(0)

def generate_frames():
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # HSV range for green (Dipsy the Teletubby)
        lower = np.array([40, 50, 50])
        upper = np.array([80, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Stack original and result side by side
        combined = np.hstack((frame, result))

        ret, buffer = cv2.imencode('.jpg', combined)
        frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)