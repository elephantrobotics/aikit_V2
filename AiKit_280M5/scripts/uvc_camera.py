"""uvc_camera.py
This module defines a UVC camera interface for capturing video frames.

Author: Your Name
Date: 2025-09-05
"""

import cv2
import numpy as np
import time
import typing


class UVCCamera:
    """A class to handle UVC camera operations including frame capture and retrieval."""

    def __init__(
        self,
        cam_index: int = 0,
        mtx: typing.Optional[np.ndarray] = None,
        dist: typing.Optional[np.ndarray] = None,
        capture_size: typing.Tuple[int, int] = (640, 480),
    ):
        """Initializes the UVCCamera object.

        Args:
            cam_index: The index of the camera to use (default is 0).
            mtx: Optional camera intrinsic matrix.
            dist: Optional camera distortion coefficients.
            capture_size: Tuple of (width, height) for the capture resolution.
        """
        super().__init__()
        self.cam_index = cam_index
        self.mtx = mtx
        self.dist = dist
        self.curr_color_frame: typing.Optional[np.ndarray] = None
        self.capture_size = capture_size
        self.cap: typing.Optional[cv2.VideoCapture] = None

    def capture(self) -> None:
        """Opens the camera and sets the capture resolution."""
        self.cap = cv2.VideoCapture(self.cam_index, cv2.CAP_DSHOW)  # Windows
        # self.cap = cv2.VideoCapture(self.cam_index, cv2.CAP_V4L)  # Linux
        width, height = self.capture_size
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Keep only the latest frame
        # cv2.setUseOptimized(True)

    def update_frame(self) -> bool:
        """Reads the latest frame from the camera.

        Returns:
            True if a frame was successfully read, False otherwise.
        """
        ret, self.curr_color_frame = self.cap.read()
        return ret

    def color_frame(self) -> typing.Optional[np.ndarray]:
        """Returns the latest captured color frame.

        Returns:
            The current color frame as a NumPy array or None if no frame is available.
        """
        return self.curr_color_frame

    def release(self) -> None:
        """Releases the camera and closes all OpenCV windows."""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    # Initialize the camera
    cam = UVCCamera(1)
    cam.capture()

    window_name = "Preview"
    while True:
        if not cam.update_frame():
            continue

        frame = cam.color_frame()
        if frame is None:
            time.sleep(0.01)
            continue

        # print(frame.shape)
        cv2.imshow(window_name, frame)

        # Press 'q' to exit
        if cv2.waitKey(1) == ord("q"):
            break

    cam.release()
