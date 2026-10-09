import threading
import time
from io import BytesIO

from flask import Flask, request, Response
from PIL import Image, UnidentifiedImageError

app = Flask(__name__)
app.config['MAX_CONTENT_LENGTH'] = 5 * 1024 * 1024

latest_frame = None
frame_lock = threading.Lock()


@app.route('/image', methods=['POST'])
def receive_image():
    global latest_frame
    if 'image' not in request.files:
        return 'Missing image field', 400
    data = request.files['image'].read()
    try:
        Image.open(BytesIO(data)).verify()
    except Exception:
        return 'Invalid image', 400
    with frame_lock:
        latest_frame = data
    return 'OK', 200


@app.route('/')
def index():
    return Response(stream_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def stream_frames():
    while True:
        with frame_lock:
            frame = latest_frame
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            try:
                with open("/app/data/placeholder.jpg", "rb") as f:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + f.read() + b'\r\n')
            except FileNotFoundError:
                pass
        time.sleep(0.03)


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=14080, debug=False, threaded=True)
