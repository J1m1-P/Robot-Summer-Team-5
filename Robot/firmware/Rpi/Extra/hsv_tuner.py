"""
HSV Color Tuner
=================
This script opens webcam feed on browser and allows user to tune HSV values for color detection.
The user can adjust the HSV values using sliders on the web interface, and the changes will be reflected in real-time on the video feed.

How it works:
- Flask runs a web server on port 5000
- The browser loads a page with 6 sliders (H min/max, S min/max, V min/max)
- The webcam feed is streamed to the browser as MJPEG (a continuous sequence of JPEG frames)
- Each frame has the HSV mask applied before being sent, so you see only the detected colour
- When a slider moves, the browser sends the new values to Flask via a POST request
    -How to use: 
        - start with H min/max to get the right hue
        - then adjust S min  to remove grey areas
        - V min to remove shadow areas
        -copy the values to teletubby_detector.py
    - A clean mask looks like a solid blob of the target colour with no holes or speckles
    - If there is speckle noise, increase S min
    - If teleubby has dark patches, decrease V min
- Flask updates the shared hsv_state dictionary, which generate_frames() reads on the next frame
- Press "Copy values" to get the numpy arrays ready to paste into teletubby_detector.py
"""

import cv2
import numpy as np
from flask import Flask, Response, render_template_string, request, jsonify

# Keep the camera open for the entire lifetime of the app.
# VideoCapture(1) uses the second camera — change to 0 if only one camera is connected.
cap = cv2.VideoCapture(1)

# Flask is the web framework. __name__ tells Flask where to find resources.
app = Flask(__name__)

# Shared HSV state dictionary.
# Both generate_frames() (reader) and the /update route (writer) access this.
# Starting values are set to detect green (Dipsy) as a default.
# Notes: red ~0, green ~60, blue ~120, yellow ~30, purple ~150, cyan ~90
hsv_state = {
    "h_min":  40,   # Minimum hue (0-179). Green starts around 40.
    "h_max":  80,   # Maximum hue. Green ends around 80.
    "s_min":  60,   # Minimum saturation. Keeps out washed-out/grey colours.
    "s_max": 255,   # Maximum saturation. 255 = accept any saturation above s_min.
    "v_min":  60,   # Minimum value (brightness). Keeps out very dark pixels.
    "v_max": 255,   # Maximum value. 255 = accept any brightness above v_min.
}

# ─────────────────────────────────────────────
# FRAME GENERATOR
# ─────────────────────────────────────────────

def generate_frames():
    """
    Generator function that runs in an infinite loop, capturing frames from
    the webcam, applying the HSV mask, encoding as JPEG, and yielding each
    frame in MJPEG format so the browser can display it as live video.

    A generator uses 'yield' instead of 'return' — it pauses after each yield
    and resumes from the same point on the next call, allowing continuous streaming.
    """
    while True:
        success, frame = cap.read()  # Read one frame from the webcam
        if not success:
            break  # Stop if the camera feed fails

        # Build lower and upper HSV bounds from the current shared state.
        # np.array converts the list into a format OpenCV can use for comparisons.
        lower = np.array([hsv_state["h_min"], hsv_state["s_min"], hsv_state["v_min"]])
        upper = np.array([hsv_state["h_max"], hsv_state["s_max"], hsv_state["v_max"]])

        # Convert frame from BGR (OpenCV default) to HSV colour space.
        # HSV separates colour (hue) from brightness (value), making colour
        # detection much more robust to lighting changes than using BGR directly.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a binary mask: white (255) where pixels fall within the HSV
        # range, black (0) everywhere else.
        mask = cv2.inRange(hsv, lower, upper)

        # Apply the mask to the original frame using bitwise AND.
        # Pixels where mask=255 pass through unchanged; pixels where mask=0 become black.
        # This leaves only the detected colour visible in the output.
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Encode the result as a JPEG image and convert to raw bytes for streaming.
        _, buffer = cv2.imencode('.jpg', result)
        result_bytes = buffer.tobytes()

        # Yield the frame in MJPEG multipart format.
        # The browser reads this as a continuous video stream via the <img> tag.
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + result_bytes + b'\r\n')

# ─────────────────────────────────────────────
# ROUTES
# ─────────────────────────────────────────────

@app.route('/')
def index():
    """
    Serves the main HTML page when the browser visits http://<ip>:5000.
    Contains the live video feed, six HSV sliders, and a copy button.

    The JavaScript on the page:
    - Listens for slider movement
    - Updates the displayed numeric value next to each slider
    - Sends all six values to /update via a POST request on every change
    - copyValues() formats the current values as numpy arrays and displays them
    """
    html = '''
    <html>
    <head>
        <title>HSV Tuner</title>
        <style>
            body  { font-family: monospace; background: #1a1a1a; color: #eee; padding: 20px; }
            h1    { color: #7fc; }
            img   { display: block; margin-bottom: 20px; width: 640px; border: 1px solid #444; }
            label { display: block; margin: 8px 0; font-size: 14px; }
            input[type=range] { width: 300px; accent-color: #7fc; }
            .val  { color: #7fc; font-weight: bold; margin-left: 10px; }
            pre   { background: #222; padding: 12px; border-radius: 4px;
                    border: 1px solid #444; color: #7fc; display: none; }
            button { margin-top: 12px; padding: 8px 20px; background: #7fc;
                     color: #111; border: none; font-family: monospace;
                     font-size: 14px; font-weight: bold; cursor: pointer; border-radius: 4px; }
        </style>
    </head>
    <body>
        <h1>HSV Colour Tuner</h1>

        <!-- The img tag points to /stream. The browser keeps this connection open
             and updates the image continuously as new JPEG frames arrive. -->
        <img src="/stream">

        <!-- Six sliders — one for each HSV bound. H is 0-179, S and V are 0-255. -->
        <label>H min <span class="val" id="hmin_v">40</span>
            <br><input type="range" id="h_min" min="0" max="179" value="40">
        </label>
        <label>H max <span class="val" id="hmax_v">80</span>
            <br><input type="range" id="h_max" min="0" max="179" value="80">
        </label>
        <label>S min <span class="val" id="smin_v">60</span>
            <br><input type="range" id="s_min" min="0" max="255" value="60">
        </label>
        <label>S max <span class="val" id="smax_v">255</span>
            <br><input type="range" id="s_max" min="0" max="255" value="255">
        </label>
        <label>V min <span class="val" id="vmin_v">60</span>
            <br><input type="range" id="v_min" min="0" max="255" value="60">
        </label>
        <label>V max <span class="val" id="vmax_v">255</span>
            <br><input type="range" id="v_max" min="0" max="255" value="255">
        </label>

        <button onclick="copyValues()">Copy values</button>
        <!-- Output box — hidden until copyValues() is called -->
        <pre id="output"></pre>

        <script>
            // Map each slider ID to its displayed value label ID
            const sliders = ["h_min","h_max","s_min","s_max","v_min","v_max"];
            const labels  = {
                h_min:"hmin_v", h_max:"hmax_v",
                s_min:"smin_v", s_max:"smax_v",
                v_min:"vmin_v", v_max:"vmax_v"
            };

            // Attach an event listener to every slider.
            // On every movement, update the label and send values to Flask.
            sliders.forEach(id => {
                document.getElementById(id).addEventListener("input", () => {
                    const val = document.getElementById(id).value;
                    document.getElementById(labels[id]).textContent = val;
                    sendUpdate();
                });
            });

            // Collect all slider values and POST them to /update as JSON.
            // Flask receives this, updates hsv_state, and generate_frames()
            // picks up the new values on the next frame.
            function sendUpdate() {
                const body = {};
                sliders.forEach(id => body[id] = parseInt(document.getElementById(id).value));
                fetch("/update", {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify(body)
                });
            }

            // Format current slider values as numpy arrays and display them.
            // Also copies to clipboard so you can paste directly into teletubby_detector.py.
            function copyValues() {
                const v = {};
                sliders.forEach(id => v[id] = document.getElementById(id).value);
                const text =
                    `lower = np.array([${v.h_min}, ${v.s_min}, ${v.v_min}])\\n` +
                    `upper = np.array([${v.h_max}, ${v.s_max}, ${v.v_max}])`;
                const box = document.getElementById("output");
                box.textContent = text;
                box.style.display = "block";
                navigator.clipboard.writeText(text).catch(() => {});
            }
        </script>
    </body>
    </html>
    '''
    return render_template_string(html)


@app.route('/stream')
def stream():
    """
    Returns the MJPEG video stream.
    The browser's <img src="/stream"> tag keeps this connection open permanently,
    continuously receiving new JPEG frames from generate_frames().
    """
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/update', methods=['POST'])
def update():
    """
    Receives slider values from the browser as a JSON POST request and
    updates the shared hsv_state dictionary. generate_frames() reads from
    hsv_state on every frame, so the change takes effect immediately.
    Returns a simple JSON confirmation so the browser knows the update landed.
    """
    data = request.get_json()
    for key in hsv_state:
        if key in data:
            hsv_state[key] = int(data[key])
    return jsonify(ok=True)


# ─────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────

if __name__ == '__main__':
    # host='0.0.0.0' makes the server accessible from other devices on the network
    # (e.g. opening http://<pi-ip>:5000 from your laptop when running on the Pi).
    # debug=False is important — debug mode restarts the app on code changes,
    # which would break the camera stream.
    app.run(host='0.0.0.0', port=5000, debug=False)