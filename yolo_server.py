from flask import Flask, request
from ultralytics import YOLO
import numpy as np
import cv2

app = Flask(__name__)

# Load model (you can switch to yolov8s.pt for better accuracy)
model = YOLO("yolov8s.pt")


@app.route("/")
def home():
    return "YOLO server is running"


@app.route("/detect", methods=["POST"])
def detect():
    # READ IMAGE FROM ESP32
    file_bytes = request.data

    npimg = np.frombuffer(file_bytes, np.uint8)
    img = cv2.imdecode(npimg, cv2.IMREAD_COLOR)

    if img is None:
        return ("Invalid image", 400)

    # RUN YOLO 
    results = model(img, conf=0.25)[0]

    person_detected = False

    print("\nYOLO FRAME ")
    print("Detections:", len(results.boxes))

    # DRAW AND PROCESS DETECTIONS
    for box in results.boxes:
        cls = int(box.cls[0])
        label = model.names[cls]
        conf = float(box.conf[0])

        print(f"Detected: {label}, conf={conf:.2f}")

        # only care about people
        if label == "person" and conf > 0.25:
            person_detected = True

        # draw bounding box
        x1, y1, x2, y2 = map(int, box.xyxy[0])

        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            img,
            f"{label} {conf:.2f}",
            (x1, y1 - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2
        )

    print("PERSON DETECTED:", person_detected)

    # ENCODE IMAGE 
    success, buffer = cv2.imencode(".jpg", img)

    if not success:
        return ("Encoding failed", 500)

    # RESPONSE FORMAT FOR ESP32 
    headers = {
        "Content-Type": "image/jpeg",
        "X-Person": "1" if person_detected else "0"
    }

    return (buffer.tobytes(), 200, headers)


if __name__ == "__main__":
    # ESP32 stability
    app.run(host="0.0.0.0", port=5000, threaded=True)