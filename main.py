
from detect import Detection

def main():
    detection = Detection()
    print("Starting the camera feed. Press 'q' to quit.")

    # Main loop to continuously capture and display images
    try:
        while True:
            if not detection.publish_image():
                break  # Exit the loop if 'q' is pressed
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        detection.shutdown()
        print("Application terminated.")

if __name__ == "__main__":
    main()