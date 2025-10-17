import cv2

def main():
    # Connect to the first USB camera (index 0)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    print("Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break

        # Detect frame type
        channels = frame.shape[2] if len(frame.shape) == 3 else 1
        if channels == 3:
            print("Camera output format: RGB (3 channels)")
        elif channels == 4:
            print("Camera output format: ARGB/RGBA (4 channels)")
        else:
            print("Camera output format: Grayscale (1 channel)")

        cv2.imshow('USB Camera Stream', frame)

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()