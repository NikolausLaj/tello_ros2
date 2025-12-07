import cv2
import os
import argparse
import djitellopy

# Argument parser setup
parser = argparse.ArgumentParser(description="Capture images of a checkerboard for camera calibration.")
parser.add_argument("--num_images", type=int, default=10, help="Number of images to capture.")
parser.add_argument("--save_directory", type=str, default="calibration_images", help="Directory to save the images.")
parser.add_argument("--image_prefix", type=str, default="checkerboard_image_", help="Prefix for saved images.")
args = parser.parse_args()

# Create save directory if it doesn't exist
if not os.path.exists(args.save_directory):
    os.makedirs(args.save_directory)

# Connect to Tello drone
tello = djitellopy.Tello()
tello.connect()
tello.streamon()

frame_read = tello.get_frame_read()

if not frame_read:
    print("Error: Could not start Tello video stream.")
    tello.streamoff()
    tello.end()
    exit()

print("Press 'c' to capture an image or 'q' to quit.")

count = 0
while count < args.num_images:
    frame = frame_read.frame
    if frame is None:
        print("Error: Failed to capture image from Tello.")
        break

    # Convert BGR to RGB for correct color display (Tello provides BGR)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    cv2.imshow("Tello Video Stream", rgb_frame)

    # Wait for key press
    key = cv2.waitKey(1) & 0xFF
    if key == ord('c'):
        # Capture and save the image in RGB
        image_path = os.path.join(args.save_directory, f"{args.image_prefix}{count+1}.jpg")
        cv2.imwrite(image_path, rgb_frame)
        print(f"Captured image {count+1} at {image_path}")
        count += 1
    elif key == ord('q'):
        print("Quitting...")
        break

# Release resources
cv2.destroyAllWindows()
tello.streamoff()
tello.end()
print("Image capture complete.")