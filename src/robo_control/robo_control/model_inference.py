import cv2
import json
import requests

# Import the inference SDK
from inference_sdk import InferenceHTTPClient

# Initialize the client
CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key="alK8EEqJGxv5cm4KiuF0"
)

# Function to infer on an image and get detection results
def infer_on_frame(frame):
    # Save the frame to a temporary file
    temp_filename = "temp_frame.jpg"
    cv2.imwrite(temp_filename, frame)
    
    # Infer on the saved image
    result = CLIENT.infer(temp_filename, model_id="summer_project_morphobot/1")
    
    # Process the result
    return result

# Open a connection to the webcam
cap = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    # Perform inference on the current frame
    if ret:
        result = infer_on_frame(frame)
        detections = result.get("predictions", [])

        # Draw bounding boxes on the frame
        for detection in detections:
            x, y, w, h = detection["x"], detection["y"], detection["width"], detection["height"]
            class_name = detection["class"]
            confidence = detection["confidence"]

            # Calculate bounding box coordinates
            x1 = int(x - w / 2)
            y1 = int(y - h / 2)
            x2 = int(x + w / 2)
            y2 = int(y + h / 2)
            
            # Draw the bounding box and label on the frame
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{class_name}: {confidence:.2f}"
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the resulting frame
        cv2.imshow('Live Feed', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture and close windows
cap.release()
cv2.destroyAllWindows()

