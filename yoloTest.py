import cv2

from ultralytics import YOLO

# Load the pretrained YOLOv8 model
model = YOLO("yolov8s_playing_cards.pt")

# Open the default camera
cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO inference
    results = model(frame)
    
    # Plot the results on the frame
    for r in results:
        annotated_frame = r.plot()

    # Show the frame
    cv2.imshow('YOLO Card Detection', annotated_frame)

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()