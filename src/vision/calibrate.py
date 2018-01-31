
import argparse
import cv2
import glob
import yaml
import numpy as np
import progressbar
import os

assert cv2.__version__[0] == '3', \
       'The fisheye module requires opencv version >= 3.0.0'


# Define the internal dimensions of the checker board pattern.
CHECKERBOARD = (7, 7)

# Pre-define OpenCV camera calibration flags.
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + \
                    cv2.fisheye.CALIB_CHECK_COND + \
                    cv2.fisheye.CALIB_FIX_SKEW

# Define an array for storing the locations of the checkerboard corners.
objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Calibrates a fisheye camera '
            'from a set of checkerboard images.')

    parser.add_argument('input_dir', type=str, help='Directory containing '
            'training image files.')
    parser.add_argument('output', type=str, help='File to save the '
            'configuration to.')

    args = parser.parse_args()

    images = glob.glob('{}/*.jpg'.format(args.input_dir))

    # Store the image shape. All images in the folder must have identical
    # dimensions.
    _img_shape = None

    # Store the object and image points from each image for calibration.
    objpoints = []
    imgpoints = []

    bar = progressbar.ProgressBar(max_value=len(images))

    for count, fname in enumerate(images):
        bar.update(count)

        img = cv2.imread(fname)
        if _img_shape is None:
            _img_shape = img.shape[:2]
        else:
            assert _img_shape == img.shape[:2], \
                "All images must share the same size."

        # Convert the image to grayscale.
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray,
                                                 CHECKERBOARD,
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH +
                                                 cv2.CALIB_CB_FAST_CHECK +
                                                 cv2.CALIB_CB_NORMALIZE_IMAGE)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)

    # Construct the matrices needed for calibration
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))

    N_OK = len(objpoints)
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

    print 'Checkerboard pattern was discovered in {} images.'.format(N_OK)
    print 'Calibrating... (This may take a few minutes)'

    # Perform the calibration of the fisheye camera lens.
    rms, _, _, _, _ = cv2.fisheye.calibrate(objpoints,
                                            imgpoints,
                                            gray.shape[::-1],
                                            K,
                                            D,
                                            rvecs,
                                            tvecs,
                                            calibration_flags,
                                            (cv2.TERM_CRITERIA_EPS +
                                             cv2.TERM_CRITERIA_MAX_ITER,
                                             30, 1e-6))

    # Write the calculated calibration to a YAML file.
    with open(args.output, 'w') as f:
        data = dict(
            camera_matrix=K.tolist(),
            distortion_coefficients=D.reshape((4)).tolist(),
            size=dict(
                cols=_img_shape[0],
                rows=_img_shape[1]
            )
        )
        yaml.dump(data, f)

    print 'Calibration was stored in {} as a YAML.'.format(args.output)
