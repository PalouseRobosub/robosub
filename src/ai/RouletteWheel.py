import cv2
import enum
import numpy as np
import rospy

class Color(enum.Enum):
    """Describes a roulette color."""
    RED = 1
    GREEN = 2
    BLACK = 3


class Slice:
    """Describes a colored slice of the roulette wheel.

    Attributes:
        origin: A tuple of the X and Y coordinates of the origin of the slice.
        color: The Color type of the slice.
    """

    def __init__(self, origin, color):
        """Initializes a slice.

        Args:
            origin: The origin tuple point of the slice in (x, y) form. Units
                are in OpenCV pixel coordinates.
            color: The Color type of the slice.
        """
        self.color = color
        self.origin = origin


class RouletteWheel:
    """Parses an image of the roulette wheel into colored slices.

    Attributes:
        image: The raw image with boundary and origin circles drawn on top.
        slices: A list of Slice objects describing the slices of the wheel.
    """

    def __init__(self, image):
        """Initializes and parses the roulette wheel image.

        Args:
            image: The image of the roulette wheel.
        """
        self.image = image.copy()
        self.slices = self.process()


    def get_circle(self, mask, min_cnt=2):
        """Gets the minimum encompassing circle of a threshold image with a
            number of contours.

        Args:
            mask: The binary image to bound with a circle.
            min_cnt: The minimum number of contours that should be in the
                threshold image.
        """
        im, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE)

        # Ensure atleast a number of contours exists.
        if len(contours) < min_cnt:
            return (0, 0), 0

        # Sort the contours in descending order by size.
        sorted_contours = sorted(contours, key=lambda x: cv2.contourArea(x),
                reverse=True)

        # Enclose a circle around all of the largest contours. All contours
        # that are atleast 33% as big as the largest are signficant.
        largest_area = cv2.contourArea(sorted_contours[0])
        for i in range(1, len(sorted_contours)):
            contour = sorted_contours[i]
            if cv2.contourArea(contour) > largest_area / 3:
                min_cnt += 1

        # Calculate the minimum enclosing circle for the largest contours.
        # This should be the border of the roulette wheel.
        points = np.concatenate(sorted_contours[:min_cnt])
        (x, y), radius = cv2.minEnclosingCircle(points)
        return (x, y), radius


    def get_slice_moments(self, mask, color):
        """Calculates the moments of slices of a specific color.

        Args:
            mask: A threshold image that should contain two distinct slice
                objects.
            color: The Color type to assign to the slice.
        """
        im, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE)

        # Ensure atleast two contours exists. Each slice has a mirrored counter
        # part. If two do not exist, there was improper filtering done
        # previously.
        if len(contours) < 2:
            return []

        # Sort the contours in descending order by size.
        sorted_contours = sorted(contours, key=lambda x: cv2.contourArea(x),
                reverse=True)

        # Calculate the center of mass for each contour.
        slices = []
        for i in range(0, 2):
            contour = sorted_contours[i]
            M = cv2.moments(contour)
            if M['m00'] == 0:
                cX = 0
                cY = 0
            else:
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])
            slices.append(Slice((cX, cY), color))

        return slices


    def process(self):
        """Processes the image of the roulette wheel into a number of Slice
            objects.
        """
        # Blur the image and convert it to an HSV image.
        blurred = cv2.GaussianBlur(self.image, (5, 5), 0)
        hsv_im = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Find all red portions of the image.
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        red_low_hues = cv2.inRange(hsv_im, lower_red, upper_red)

        lower_red = np.array([160, 100, 100])
        upper_red = np.array([179, 255, 255])
        red_high_hues = cv2.inRange(hsv_im, lower_red, upper_red)

        red_filtered = cv2.addWeighted(red_high_hues, 1.0, red_low_hues, 1.0, 0)

        # Dilate and close holes within the detected red portions.
        red_closed = cv2.morphologyEx(red_filtered, cv2.MORPH_CLOSE, (9, 9))
        red_mask = cv2.dilate(red_closed, (13, 13), iterations=6)

        # Construct the bounding circle of the entire roulette wheel from the
        # detected red portions of the image.
        (x, y), radius = self.get_circle(red_mask, 1)

        # Create a mask to remove the center of the roulette wheel from all
        # threshold masks. The center has a tendendency to cause two slice
        # contours to merge, but this doesn't become a problem if the center is
        # removed from the mask. Additionally, ignoring colors in the center
        # pushes the contour origin towards the edge of the wheel, which gives
        # more room for error in dropping the marker.
        center_mask = np.zeros(shape=red_mask.shape, dtype=np.uint8)
        cv2.circle(center_mask, (int(x), int(y)), int(radius * 0.3), 255, -1)
        mask_inv = cv2.bitwise_not(center_mask)

        # Remove the center fron the red mask and grab the slices from it.
        red_mask = cv2.bitwise_and(red_mask, mask_inv)

        red_slices = self.get_slice_moments(red_mask, Color.RED)
        if len(red_slices) != 2:
            return []

        # For black, we don't do color thresholding. Instead, assume the whole
        # wheel is potentially black and subtract the red and green regions
        # as we find them. Any remaining area is then checked for low color
        # value, which indicates blackness is present.
        self.origin = (int(x), int(y))
        self.radius = int(radius * 0.85)
        binary_wheel = np.zeros(shape=red_mask.shape, dtype=np.uint8)
        cv2.circle(binary_wheel, (int(x), int(y)), int(radius), 255, -1)

        # Filter for green sections of the wheel
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([75, 255, 255])
        green_hues = cv2.inRange(hsv_im, lower_green, upper_green)

        # Remove the center from the green mask and grab slices from it.
        green_hues = cv2.bitwise_and(mask_inv, green_hues)
        green_mask = cv2.morphologyEx(green_hues, cv2.MORPH_CLOSE, (9, 9))
        green_slices = self.get_slice_moments(green_mask, Color.GREEN)
        if len(green_slices) != 2:
            return []

        # Subtract the green and red slices from the binary wheel. They can't
        # be considered black if they have already been used.
        binary_wheel = cv2.bitwise_and(binary_wheel, cv2.bitwise_not(red_mask))
        binary_wheel = cv2.bitwise_and(binary_wheel,
                                       cv2.bitwise_not(green_mask))

        # Filter for black colors by searching for low value in the HSV space.
        low_black = np.array([0, 0, 0])
        high_black = np.array([180, 255, 30])
        black_values = cv2.inRange(hsv_im, low_black, high_black)

        # Check for black values only in areas within the wheel mask that
        # haven't been used for red and green. Remove the center of the wheel.
        binary_wheel = cv2.bitwise_and(binary_wheel, mask_inv)
        black_mask_closed = cv2.morphologyEx(binary_wheel, cv2.MORPH_CLOSE,
                (9, 9))
        black_mask = cv2.bitwise_and(black_mask_closed, black_values)

        black_slices = self.get_slice_moments(black_mask, Color.BLACK)
        if len(black_slices) != 2:
            return []

        # Draw a circle around the border of the bounding circle on the
        # roulette wheel. Also draw circles around the origins of each slice.
        cv2.circle(self.image, self.origin, self.radius, (255, 255, 0), 2)

        for red in red_slices:
            cv2.circle(self.image, red.origin, 4, (255, 0, 0), 2)
        for green in green_slices:
            cv2.circle(self.image, green.origin, 4, (0, 0, 255), 2)
        for black in black_slices:
            cv2.circle(self.image, black.origin, 4, (0, 155, 0), 2)

        # Create a colored image of the slices for debugging purposes. The red
        # regions are shown in red, green regions in green, and black regions
        # in blue.
        wheel = np.zeros(shape=(red_mask.shape[0],
                    red_mask.shape[1], 3), dtype=np.uint8)
        cv2.circle(wheel, self.origin, self.radius, (255, 255, 255), -1)
        wheel = cv2.bitwise_not(wheel)

        green_mask_color = cv2.cvtColor(green_mask, cv2.COLOR_GRAY2BGR)
        green_mask_color[green_mask > 0] = (0, 255, 0)

        red_mask_color = cv2.cvtColor(red_mask, cv2.COLOR_GRAY2BGR)
        red_mask_color[red_mask > 0] = (0, 0, 255)

        black_mask_color = cv2.cvtColor(black_mask, cv2.COLOR_GRAY2BGR)
        black_mask_color[black_mask > 0] = (255, 0, 0)

        wheel = cv2.bitwise_or(wheel, green_mask_color)
        wheel = cv2.bitwise_or(wheel, red_mask_color)
        wheel = cv2.bitwise_or(wheel, black_mask_color)

        self.visual_slices = cv2.resize(wheel, (0, 0), fx=0.5, fy=0.5)

        return red_slices + green_slices + black_slices
