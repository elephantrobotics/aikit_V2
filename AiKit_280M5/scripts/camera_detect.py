import argparse
import cv2
from uvc_camera import UVCCamera
import stag
import numpy as np
import json
import time
from marker_utils import *
from scipy.linalg import svd
from pymycobot import *
from pymycobot.utils import get_port_list

# List available serial ports
plist = get_port_list()
print('serial:', plist)

# Initialize MyCobot280 with serial port and baud rate
parser = argparse.ArgumentParser(description='Camera flange demonstration')
parser.add_argument('--port', type=str, default='/dev/ttyACM0', help='Device serial port number')
parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
args = parser.parse_args()
print(f"Using serial port: {args.port}, baud rate: {args.baud}")
mc = MyCobot280(args.port, args.baud)

# Detect robot version (280 or 320)
type = mc.get_system_version()
offset_j5 = 0
if type > 2:
    offset_j5 = -90
    print("280")
else:
    print("320")

# NumPy print config
np.set_printoptions(suppress=True, formatter={'float_kind': '{:.2f}'.format})


class camera_detect:
    """Camera-based marker detection and robot-eye-in-hand calibration."""

    def __init__(self, camera_id, marker_size, mtx, dist):
        # Camera initialization
        self.camera_id = camera_id
        self.mtx = mtx
        self.dist = dist
        self.marker_size = marker_size
        self.camera = UVCCamera(self.camera_id, self.mtx, self.dist)
        self.camera_open()

        # Default robot positions
        self.origin_mycbot_horizontal = [-45, -35.85, -52.91, 88.59, 90+offset_j5, 40.0]
        self.origin_mycbot_level = [-45, 5, -104, 14, 90 + offset_j5, 40]

        # Minimum distance to maintain when identifying markers
        self.IDENTIFY_LEN = 300

        # Load calibration matrix if available
        self.EyesInHand_matrix = None
        self.load_matrix()

    def save_matrix(self, filename="/home/er/aikit_V2/AiKit_280M5/scripts/EyesInHand_matrix.json"):
        """Save EyesInHand_matrix to JSON file."""
        if self.EyesInHand_matrix is not None:
            with open(filename, 'w') as f:
                json.dump(self.EyesInHand_matrix.tolist(), f)

    def load_matrix(self, filename="/home/er/aikit_V2/AiKit_280M5/scripts/EyesInHand_matrix.json"):
        """Load EyesInHand_matrix from JSON file."""
        try:
            with open(filename, 'r') as f:
                self.EyesInHand_matrix = np.array(json.load(f))
        except FileNotFoundError:
            print("Matrix file not found. EyesInHand_matrix will be initialized later.")

    def wait(self):
        """Wait until the robot finishes moving."""
        time.sleep(0.5)
        while mc.is_moving() == 1:
            time.sleep(0.2)

    def coord_limit(self, coords):
        """Clamp coordinates within predefined limits."""
        min_coord = [-350, -350, 300]
        max_coord = [350, 350, 500]
        for i in range(3):
            coords[i] = max(min(coords[i], max_coord[i]), min_coord[i])

    def camera_open(self):
        """Start camera capture."""
        self.camera.capture()

    def calc_markers_base_position(self, corners, ids):
        """Calculate object coordinates in the camera system."""
        if len(corners) == 0:
            return []
        rvecs, tvecs = solve_marker_pnp(corners, self.marker_size, self.mtx, self.dist)
        for i, tvec, rvec in zip(ids, tvecs, rvecs):
            tvec = tvec.squeeze().tolist()
            rvec = rvec.squeeze().tolist()
            rotvector = np.array([[rvec[0], rvec[1], rvec[2]]])
            Rotation = cv2.Rodrigues(rotvector)[0]
            Euler = self.CvtRotationMatrixToEulerAngle(Rotation)
            target_coords = np.array([tvec[0], tvec[1], tvec[2], Euler[0], Euler[1], Euler[2]])
        return target_coords

    def stag_robot_identify(self, ml):
        """Identify marker and convert to robot coordinates.

        Args:
            ml: Robot control object with get_coords method.

        Returns:
            tuple: Updated target coordinates and marker IDs.
        """
        marker_pos_pack, ids = self.stag_identify()
        target_coords = ml.get_coords()  # Get current robot coordinates
        while target_coords is None:
            target_coords = ml.get_coords()

        cur_coords = np.array(target_coords.copy())
        cur_coords[-3:] *= (np.pi / 180)  # Convert degrees to radians

        # Transform object coordinates (camera system) to base coordinate system
        fact_bcl = self.Eyes_in_hand(cur_coords, marker_pos_pack, self.EyesInHand_matrix)

        for i in range(3):
            target_coords[i] = fact_bcl[i]

        return target_coords, ids

    def stag_robot_identify_loop(self, ml):
        """Continuously identify marker and update robot coordinates.

        Args:
            ml: Robot control object with get_coords method.
        """
        while True:
            self.camera.update_frame()  # Refresh camera frame
            frame = self.camera.color_frame()  # Get current frame
            cv2.imshow("Enter", frame)

            # Exit loop when a key is pressed
            if cv2.waitKey(1) & 0xFF != 255:
                break

            marker_pos_pack, _ = self.stag_identify()
            target_coords = ml.get_coords()
            while target_coords is None:
                target_coords = ml.get_coords()

            cur_coords = np.array(target_coords.copy())
            cur_coords[-3:] *= (np.pi / 180)  # Convert degrees to radians

            # Transform object coordinates (camera system) to base coordinate system
            fact_bcl = self.Eyes_in_hand(cur_coords, marker_pos_pack, self.EyesInHand_matrix)

            for i in range(3):
                target_coords[i] = fact_bcl[i]

            print("robot_coords", target_coords)

    def CvtRotationMatrixToEulerAngle(self, pdtRotationMatrix):
        """Convert rotation matrix to Euler angles.

        Args:
            pdtRotationMatrix (np.ndarray): 3x3 rotation matrix.

        Returns:
            np.ndarray: Euler angles (roll, pitch, yaw).
        """
        pdtEulerAngle = np.zeros(3)
        pdtEulerAngle[2] = np.arctan2(pdtRotationMatrix[1, 0], pdtRotationMatrix[0, 0])
        fCosRoll = np.cos(pdtEulerAngle[2])
        fSinRoll = np.sin(pdtEulerAngle[2])
        pdtEulerAngle[1] = np.arctan2(
            -pdtRotationMatrix[2, 0],
            (fCosRoll * pdtRotationMatrix[0, 0]) + (fSinRoll * pdtRotationMatrix[1, 0]),
        )
        pdtEulerAngle[0] = np.arctan2(
            (fSinRoll * pdtRotationMatrix[0, 2]) - (fCosRoll * pdtRotationMatrix[1, 2]),
            (-fSinRoll * pdtRotationMatrix[0, 1]) + (fCosRoll * pdtRotationMatrix[1, 1]),
        )
        return pdtEulerAngle

    def CvtEulerAngleToRotationMatrix(self, ptrEulerAngle):
        """Convert Euler angles to rotation matrix.

        Args:
            ptrEulerAngle (np.ndarray): Euler angles (roll, pitch, yaw).

        Returns:
            np.ndarray: 3x3 rotation matrix.
        """
        ptrSinAngle = np.sin(ptrEulerAngle)
        ptrCosAngle = np.cos(ptrEulerAngle)
        ptrRotationMatrix = np.zeros((3, 3))
        ptrRotationMatrix[0, 0] = ptrCosAngle[2] * ptrCosAngle[1]
        ptrRotationMatrix[0, 1] = (
            ptrCosAngle[2] * ptrSinAngle[1] * ptrSinAngle[0]
            - ptrSinAngle[2] * ptrCosAngle[0]
        )
        ptrRotationMatrix[0, 2] = (
            ptrCosAngle[2] * ptrSinAngle[1] * ptrCosAngle[0]
            + ptrSinAngle[2] * ptrSinAngle[0]
        )
        ptrRotationMatrix[1, 0] = ptrSinAngle[2] * ptrCosAngle[1]
        ptrRotationMatrix[1, 1] = (
            ptrSinAngle[2] * ptrSinAngle[1] * ptrSinAngle[0]
            + ptrCosAngle[2] * ptrCosAngle[0]
        )
        ptrRotationMatrix[1, 2] = (
            ptrSinAngle[2] * ptrSinAngle[1] * ptrCosAngle[0]
            - ptrCosAngle[2] * ptrSinAngle[0]
        )
        ptrRotationMatrix[2, 0] = -ptrSinAngle[1]
        ptrRotationMatrix[2, 1] = ptrCosAngle[1] * ptrSinAngle[0]
        ptrRotationMatrix[2, 2] = ptrCosAngle[1] * ptrCosAngle[0]
        return ptrRotationMatrix

    def eyes_in_hand_calculate(self, pose, tbe, Mc, Mr):
        """Calculate hand-eye calibration matrix.

        Args:
            pose (np.ndarray): Robot pose (degrees).
            tbe (np.ndarray): Transformation base-to-end.
            Mc (np.ndarray): Marker coordinates.
            Mr (np.ndarray): Marker reference.

        Returns:
            np.ndarray: Transformation matrix for eyes-in-hand.
        """
        pose, Mr = map(np.array, [pose, Mr])
        euler = pose * np.pi / 180  # Convert degrees to radians
        Rbe = self.CvtEulerAngleToRotationMatrix(euler)
        Reb = Rbe.T

        A = np.empty((3, 0))
        b_comb = np.empty((3, 0))
        r = tbe.shape[0]

        for i in range(1, r):
            A = np.hstack((A, (Mc[i, :].reshape(3, 1) - Mc[0, :].reshape(3, 1))))
            b_comb = np.hstack((b_comb, (tbe[0, :].reshape(3, 1) - tbe[i, :].reshape(3, 1))))

        b = Reb @ b_comb
        U, _, Vt = svd(A @ b.T)
        Rce = Vt.T @ U.T

        tbe_sum = np.sum(tbe, axis=0)
        Mc_sum = np.sum(Mc, axis=0)

        tce = Reb @ (
            Mr.reshape(3, 1)
            - (1 / r) * tbe_sum.reshape(3, 1)
            - (1 / r) * (Rbe @ Rce @ Mc_sum.reshape(3, 1))
        )
        tce[2] -= self.IDENTIFY_LEN  # Keep recognition distance

        EyesInHand_matrix = np.vstack((np.hstack((Rce, tce)), np.array([0, 0, 0, 1])))
        print("EyesInHand_matrix = ", EyesInHand_matrix)
        return EyesInHand_matrix

    def Transformation_matrix(self, coord):
        """Convert coordinates to homogeneous transformation matrix.

        Args:
            coord (list): [x, y, z, rx, ry, rz] in radians.

        Returns:
            np.ndarray: 4x4 homogeneous transformation matrix.
        """
        position_robot = coord[:3]
        pose_robot = coord[3:]
        RBT = self.CvtEulerAngleToRotationMatrix(pose_robot)
        PBT = np.array([[position_robot[0]], [position_robot[1]], [position_robot[2]]])
        temp = np.concatenate((RBT, PBT), axis=1)
        array_1x4 = np.array([[0, 0, 0, 1]])
        matrix = np.concatenate((temp, array_1x4), axis=0)
        return matrix

    def Eyes_in_hand(self, coord, camera, Matrix_TC):
        """Transform object coordinates from camera to base system.

        Args:
            coord (np.ndarray): Robot coordinates.
            camera (np.ndarray): Camera coordinates.
            Matrix_TC (np.ndarray): Transformation matrix camera-to-object.

        Returns:
            np.ndarray: Object coordinates in base system.
        """
        Position_Camera = np.transpose(camera[:3])
        Matrix_BT = self.Transformation_matrix(coord)

        Position_Camera = np.append(Position_Camera, 1)
        Position_B = Matrix_BT @ Matrix_TC @ Position_Camera
        return Position_B

    def camera_open_loop(self):
        """Continuously display camera stream until key press."""
        while True:
            self.camera.update_frame()
            frame = self.camera.color_frame()
            cv2.imshow("Enter", frame)

            # Exit loop when a key is pressed
            if cv2.waitKey(1) & 0xFF != 255:
                break

    def stag_identify(self):
        """Capture camera coordinates (single-shot).

        Returns:
            tuple: Marker positions and IDs.
        """
        self.camera.update_frame()
        frame = self.camera.color_frame()
        corners, ids, rejected_corners = stag.detectMarkers(frame, 11)

        marker_pos_pack = self.calc_markers_base_position(corners, ids)
        if len(marker_pos_pack) == 0:
            marker_pos_pack, ids = self.stag_identify()
        return marker_pos_pack, ids

    def stag_identify_loop(self):
        """Continuously capture camera coordinates."""
        while True:
            self.camera.update_frame()
            frame = self.camera.color_frame()
            corners, ids, rejected_corners = stag.detectMarkers(frame, 11)
            marker_pos_pack = self.calc_markers_base_position(corners, ids)
            print("Camera coords = ", marker_pos_pack, ids)
            cv2.imshow("Enter", frame)
            cv2.waitKey(1)

            # Exit loop when a key is pressed
            if cv2.waitKey(1) & 0xFF != 255:
                break

    def Test(self):
        """Test function to initialize EyesInHand_matrix using sample poses and coordinates.

        This function sets up predefined poses and camera/object coordinates,
        then computes the EyesInHand calibration matrix.
        """
        pose = [-173.58, 0.0, -179.64]
        tbe1 =  np.array([-87.80, -207.50, 236.70])
        Mc1 =  np.array([48.47, -7.54, 178.45])
        tbe2 =  np.array([-36.40, -209.00, 237.90])
        Mc2 =  np.array([50.77, -56.50 ,178.21])
        tbe3 =  np.array([-36.50, -208.80, 253.30])
        Mc3 =  np.array([49.81, -55.38, 194.22])
        tbe4 =  np.array([-36.40, -187.40, 258.90])
        Mc4 =  np.array([36.50, -58.00, 202.12])
        tbe5 =  np.array([-66.70, -186.10, 257.30])
        Mc5 =  np.array([31.82, -28.74, 199.48])
        Mr = [-96.7, -168.3, 42.8]
        tbe = np.vstack([tbe1, tbe2, tbe3, tbe4, tbe5])
        Mc = np.vstack([Mc1, Mc2, Mc3, Mc4, Mc5])
        self.EyesInHand_matrix = self.eyes_in_hand_calculate(pose, tbe, Mc, Mr)

    def Matrix_identify(self, ml):
        """Identify calibration points for EyesInHand matrix calculation.

        Moves the robot to several observation points to collect robot and camera coordinates,
        then checks the consistency of the collected data.

        Args:
            ml: Instance of the robot control class for sending commands and getting coordinates.

        Returns:
            tuple: (pose, tbe, Mc, state)
                pose: Robot's orientation (Euler angles).
                tbe: Camera-based object coordinates.
                Mc: Robot coordinates corresponding to tbe.
                state: Boolean indicating if matrix calibration is consistent.
        """
        ml.send_angles(self.origin_mycbot_level, 50)  # Move to observation point
        self.wait()
        input("make sure camera can observe the stag, enter any key quit")
        coords = ml.get_coords()
        pose = coords[3:6]
        print(pose)
        # self.camera_open_loop()
        Mc1,tbe1,pos1 = self.reg_get(ml)
        ml.send_coord(1, coords[0] + 50, 30)
        self.wait()
        Mc2,tbe2,pos2 = self.reg_get(ml)
        ml.send_coord(3, coords[2] + 20, 30)
        self.wait()
        Mc3,tbe3,pos3 = self.reg_get(ml)
        ml.send_coord(2, coords[1] + 20, 30)
        self.wait()
        Mc4,tbe4,pos4 = self.reg_get(ml)
        ml.send_coord(1, coords[0] + 20, 30)
        self.wait()
        Mc5,tbe5,pos5 = self.reg_get(ml)
        tbe = np.vstack([tbe1, tbe2, tbe3, tbe4, tbe5])
        Mc = np.vstack([Mc1, Mc2, Mc3, Mc4, Mc5])
        state = None
        if self.EyesInHand_matrix is not None:
            state = True
            pos = np.vstack([pos1, pos2, pos3, pos4, pos5])
            r = pos.shape[0]
            for i in range(1, r):
                for j in range(3):
                    err = abs(pos[i][j] - pos[0][j])
                    if err > 10:
                        state = False
                        # print("matrix error")
        return pose, tbe, Mc, state

    def Eyes_in_hand_calibration(self, ml):
        """Perform Eyes-in-Hand calibration procedure.

        This function checks if EyesInHand_matrix is already valid.
        If not, it guides the user through manual calibration steps
        and computes the matrix from collected robot and camera coordinates.

        Args:
            ml: Instance of the robot control class for sending commands and getting coordinates.
        """
        mc.set_end_type(0)
        pose, tbe, Mc, state = self.Matrix_identify(ml)
        if(state == True):
            print("Calibration Complete EyesInHand_matrix = ", self.EyesInHand_matrix)
            return

        input("Move the end of the robot arm to the calibration point, press any key to release servo")
        ml.release_all_servos()
        input("focus servo and get current coords")
        ml.power_on()
        time.sleep(1)
        coords = ml.get_coords()
        while len(coords) == 0:
            coords = ml.get_coords()
        Mr = coords[0:3]
        print(Mr)


        self.EyesInHand_matrix = self.eyes_in_hand_calculate(pose, tbe, Mc, Mr)
        print("EyesInHand_matrix = ", self.EyesInHand_matrix)
        self.save_matrix()  # Save the matrix to a file after calculating it
        print("save successe, wait to verify")

        pose, tbe, Mc, state = self.Matrix_identify(ml)
        if state != True:
            self.EyesInHand_matrix = self.eyes_in_hand_calculate(pose, tbe, Mc, Mr)

    def reg_get(self, ml):
        """Get robot and camera coordinates for a single registration point.

        This function identifies the STAG marker, retrieves robot coordinates,
        and waits until valid data is available.

        Args:
            ml: Instance of the robot control class for sending commands and getting coordinates.

        Returns:
            tuple: (Mc, tbe, target_coords)
                Mc: Robot coordinates corresponding to the detected marker.
                tbe: Camera-based object coordinates.
                target_coords: Target coordinates calculated by Eyes-in-Hand transformation.
        """
        target_coords = None
        for i in range(30):
            Mc_all, _ = self.stag_identify()
        if self.EyesInHand_matrix is not None:
            target_coords, _ = self.stag_robot_identify(ml)

        tbe_all = ml.get_coords()  # Get current robot coordinates
        while tbe_all is None:
            tbe_all = ml.get_coords()

        tbe = np.array(tbe_all[0:3])
        Mc = np.array(Mc_all[0:3])
        print("tbe = ", tbe)
        print("Mc = ", Mc)
        return Mc, tbe, target_coords

    def vision_trace(self, mode, ml):
        """Perform a one-time vision-based trace operation.

        The robot moves to a predefined observation point and then
        moves to the detected marker's coordinates.

        Args:
            mode (int): Trace mode. 0 = horizontal plane grasp.
            ml: Instance of the robot control class for sending commands and getting coordinates.
        """
        sp = 40  # Set movement speed

        if mode == 0:  # Horizontal plane grasp
            ml.send_angles(self.origin_mycbot_horizontal, sp)  # Move to observation point
            self.wait()  # Wait for robot motion to finish
            input("Press any key to start trace")

            target_coords, _ = self.stag_robot_identify(ml)
            print(target_coords)

            time.sleep(1)
            ml.send_coords(target_coords, 30)  # Move robot to the marker
            self.wait()  # Wait for motion completion

    def vision_trace_loop(self, ml):
        """Continuously perform vision-based trace operation.

        The robot detects STAG markers in real-time and updates its coordinates
        accordingly. Uses Eyes-in-Hand calibration if available.

        Args:
            ml: Instance of the robot control class for sending commands and getting coordinates.
        """
        mc.set_fresh_mode(1)
        mc.set_vision_mode(1)  # Enable vision mode
        time.sleep(1)

        ml.send_angles(self.origin_mycbot_horizontal, 50)  # Move to observation point
        self.wait()
        origin = ml.get_coords()
        while origin is None:
            origin = ml.get_coords()
        time.sleep(1)

        while True:
            _, ids = self.stag_identify()
            if ids[0] == 0:
                self.camera.update_frame()  # Refresh camera frame
                frame = self.camera.color_frame()
                target_coords, _ = self.stag_robot_identify(ml)
                self.coord_limit(target_coords)
                print(target_coords)

                for i in range(3):
                    target_coords[i + 3] = origin[i + 3]

                ml.send_coords(target_coords, 30, _async=True)  # Move robot to marker
            elif ids[0] == 1:
                ml.send_angles(self.origin_mycbot_horizontal, 50)  # Reset to observation point


if __name__ == "__main__":
    # Load camera configuration parameters
    camera_params = np.load("/home/er/aikit_V2/AiKit_280M5/scripts/camera_params.npz")
    mtx, dist = camera_params["mtx"], camera_params["dist"]

    # Initialize the camera detector
    m = camera_detect(0, 32, mtx, dist)

    # Disable vision mode initially
    mc.set_vision_mode(0)

    # Uncomment the following lines to run specific functions
    # m.camera_open_loop()               # Open camera in a continuous loop
    # m.stag_identify_loop()             # Continuously detect STAG markers
    # m.stag_robot_identify_loop(mc)     # Continuously calculate robot coordinates based on STAG
    # m.Eyes_in_hand_calibration(mc)     # Calibrate Eyes-in-Hand matrix
    # m.vision_trace(0, mc)              # Perform a one-time vision trace

    # Start continuous vision trace loop
    m.vision_trace_loop(mc)

