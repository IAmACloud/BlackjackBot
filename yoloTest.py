import cv2

from ultralytics import YOLO

# Load the pretrained YOLOv8 model
model = YOLO("yolov8s_playing_cards.pt")

# Open the default camera
cap = cv2.VideoCapture(0)

print("Starting YOLO inference. Press 'q' to exit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO inference
    results = model(frame)
    
    # Print detected objects to terminal
    for r in results:
        for box in r.boxes:
            class_id = int(box.cls)
            confidence = box.conf.item()
            class_name = model.names[class_id]
            print(f"Detected: {class_name} with confidence {confidence:.2f}")
    
    # Plot the results on the frame
    annotated_frame = frame
    for r in results:
        annotated_frame = r.plot()

    # Show the frame
    cv2.imshow('YOLO Card Detection', annotated_frame)

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()