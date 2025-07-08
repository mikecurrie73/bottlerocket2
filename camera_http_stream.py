from picamera2 import Picamera2
from flask import Flask, Response
import io
from PIL import Image

app = Flask(__name__)

# Initialize camera
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))

# Enable continuous autofocus (AfMode = 2)
picam2.set_controls({"AfMode": 2})

# Start camera
picam2.start()

def generate_frames():
    while True:
        frame = picam2.capture_array("main")  # NumPy array in RGBA
        image = Image.fromarray(frame)
        if image.mode == "RGBA":
            image = image.convert("RGB")  # Convert RGBA -> RGB to fix JPEG saving error
        stream = io.BytesIO()
        image.save(stream, format='JPEG', quality=85)
        jpeg_bytes = stream.getvalue()

        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" +
               jpeg_bytes + b"\r\n")

@app.route('/')
def index():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)

