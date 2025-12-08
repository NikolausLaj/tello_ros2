
import numpy as np
import cv2
import glob
import os
import sys
import yaml


def perform_calibration(
    checkerboard_dims=(7, 10),
    square_size=22.75,
    image_path='/home/nikolaus/ros2_ws/src/tello_ros2/calibration_images',
    output_path='/home/nikolaus/ros2_ws/src/tello_ros2/config'
):
    print(f"Checkerboard dimensions: {checkerboard_dims}")
    print(f"Square size: {square_size}")
    print(f"Image path: {image_path}")
    print(f"Output path: {output_path}")

    # Verify the image path
    if not os.path.exists(image_path):
        print(f"ERROR: The specified image path does not exist: {image_path}")
        sys.exit(1)

    # Define the criteria for corner sub-pixel refinement
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points for the checkerboard pattern
    objp = np.zeros((checkerboard_dims[0] * checkerboard_dims[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard_dims[0], 0:checkerboard_dims[1]].T.reshape(-1, 2)
    objp *= square_size

    object_points = []  # 3D points in real-world space
    image_points = []   # 2D points in image plane

    # Load images from the specified path
    images = glob.glob(f'{image_path}/*.jpg')
    if not images:
        print(f"ERROR: No images found in the specified path: {image_path}")
        sys.exit(1)

    # Process each image
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the checkerboard corners
        ret, corners = cv2.findChessboardCorners(gray, checkerboard_dims, None)
        if ret:
            object_points.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            image_points.append(corners2)

            # Optional: display the corners
            cv2.drawChessboardCorners(img, checkerboard_dims, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(100)

    cv2.destroyAllWindows()

    # Check if any valid images were processed
    if len(object_points) == 0 or len(image_points) == 0:
        print("ERROR: No valid checkerboard images found. Calibration aborted.")
        return

    # Perform camera calibration
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, gray.shape[::-1], None, None
    )

    # Log calibration results
    print(f"Camera Matrix:\n{camera_matrix}")
    print(f"Distortion Coefficients:\n{dist_coeffs}")
    print(f"fx: {camera_matrix[0][0]}, fy: {camera_matrix[1][1]}, cx: {camera_matrix[0][2]}, cy: {camera_matrix[1][2]}, k1: {dist_coeffs[0][0]}, k2: {dist_coeffs[0][1]}, p1: {dist_coeffs[0][2]}, p2: {dist_coeffs[0][3]}, k3: {dist_coeffs[0][4]}")

    # Save calibration data to YAML
    calibration_data = {
        'rect_image':{
            'ros__parameters':{
                'camera_matrix': {
                    'rows': 3,
                    'cols': 3,
                    'data': [
                        float(camera_matrix[0][0]), 0.0, float(camera_matrix[0][2]),
                        0.0, float(camera_matrix[1][1]), float(camera_matrix[1][2]),
                        0.0, 0.0, 1.0
                    ]
                },
            'distortion_coefficients': {
                'rows': 1,
                'cols': 5,
                'data': [
                    float(dist_coeffs[0][0]),
                    float(dist_coeffs[0][1]),
                    float(dist_coeffs[0][2]),
                    float(dist_coeffs[0][3]),
                    float(dist_coeffs[0][4])
                ]
                }
            }
        }
    }

    with open(os.path.join(output_path, 'camera_parameters.yaml'), 'w') as f:
        yaml.dump(calibration_data, f)
    print("Calibration data saved to 'camera_parameters.yaml'")

    def perform_calibration(self):
        # Define the criteria for corner sub-pixel refinement
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Prepare object points for the checkerboard pattern
        objp = np.zeros((self.checkerboard_size[0] * self.checkerboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.checkerboard_size[0], 0:self.checkerboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size

        object_points = []  # 3D points in real-world space
        image_points = []   # 2D points in image plane

        # Load images from the specified path
        images = glob.glob(f'{self.image_path}/*.jpg')
        if not images:
            self.get_logger().error(f"No images found in the specified path: {self.image_path}")
            sys.exit(1)

        # Process each image
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the checkerboard corners
            ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)
            if ret:
                object_points.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                image_points.append(corners2)

                # Optional: display the corners
                cv2.drawChessboardCorners(img, self.checkerboard_size, corners2, ret)
                cv2.imshow('img', img)
                cv2.waitKey(100)

        cv2.destroyAllWindows()


        # Check if any valid images were processed
        if len(object_points) == 0 or len(image_points) == 0:
            self.get_logger().error("No valid checkerboard images found. Calibration aborted.")
            return

        # Perform camera calibration
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            object_points, image_points, gray.shape[::-1], None, None
        )

        # Log calibration results
        self.get_logger().info(f"Camera Matrix:\n{camera_matrix}")
        self.get_logger().info(f"Distortion Coefficients:\n{dist_coeffs}")
        self.get_logger().info(f"fx: {camera_matrix[0][0]},fy: {camera_matrix[1][1]},cx: {camera_matrix[0][2]},cy: {camera_matrix[1][2]},k1: {dist_coeffs[0][0]},k2: {dist_coeffs[0][1]},p1: {dist_coeffs[0][2]},p2: {dist_coeffs[0][3]},k3: {dist_coeffs[0][4]}")

        # Save calibration data to YAML
        calibration_data = {
            'camera_parameters':{
                'ros__parameters':{
                    'camera_matrix': {
                        'rows': 3,
                        'cols': 3,
                        'data': [
                            float(camera_matrix[0][0]), 0, float(camera_matrix[0][2]),
                            0, float(camera_matrix[1][1]), float(camera_matrix[1][2]),
                            0, 0, 1
                        ]
                    },
                'distortion_coefficients': {
                    'rows': 1,
                    'cols': 5,
                    'data': [
                        float(dist_coeffs[0][0]),
                        float(dist_coeffs[0][1]),
                        float(dist_coeffs[0][2]),
                        float(dist_coeffs[0][3]),
                        float(dist_coeffs[0][4])
                    ]
                    }
                }
            }
        }

        with open(self.output_path + '/camera_parameters.yaml', 'w') as f:
            yaml.dump(calibration_data, f)
        
        self.get_logger().info("Calibration data saved to 'camera_parameters.yaml'")


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Camera calibration script')
    parser.add_argument('--checkerboard_dims', nargs=2, type=int, default=[7, 10], help='Checkerboard dimensions (rows cols)')
    parser.add_argument('--square_size', type=float, default=22.75, help='Size of a checkerboard square (mm)')
    parser.add_argument('--image_path', type=str, default='../calibration_images', help='Path to calibration images (default: ../calibration_images)')
    parser.add_argument('--output_path', type=str, default='../config', help='Path to save calibration data (default: ../config)')
    args = parser.parse_args()

    perform_calibration(
        checkerboard_dims=tuple(args.checkerboard_dims),
        square_size=args.square_size,
        image_path=args.image_path,
        output_path=args.output_path
    )