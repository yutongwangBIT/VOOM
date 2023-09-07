import cv2
import numpy as np
import scipy.spatial.transform as trans
import matplotlib.pyplot as plt

R = np.array([[0.8660254, -0.5000000, 0.0000000],
              [0.5000000, 0.8660254, 0.00],
              [0.0000000, 0.000000, 1.000]])

freiburg_K = [[535.4, 0, 320.1], [0, 539.2, 247.6], [0, 0, 1]]
freiburg_PP = [320.1, 247.6]
freiburg_Focal = [535.4, 539.2]


def matrix2quaternion(R):
    return trans.Rotation.from_matrix(R).as_quat()  # xyzw


def quaternion2matrix(q): # q is wxyz
    #quad = [q[1], q[2], q[3], q[0]] # xyzw
    return trans.Rotation.from_quat(q).as_matrix()


def transform_inverse(T):
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    R_inv = R.T
    t_inv = - np.dot(R_inv, t)
    T_inv = np.eye(4)
    T_inv[0:3, 0:3] = R_inv
    T_inv[0:3, 3] = t_inv
    return T_inv


def transform2distance(T):
    t = T[0:3, 3]
    distance = np.linalg.norm(t)
    return distance


def pose2pose_transform(Tw1, Tw2):
    T12 = np.dot(transform_inverse(Tw1), Tw2)
    return T12


def calc_vec_transform(m1, m2):
    T_ab_measured = pose2pose_transform(m1, m2)
    p_ab = T_ab_measured[:3, 3]
    R_ab = T_ab_measured[:3, :3]
    q_ab = matrix2quaternion(R_ab)
    t_ab_measured = [p_ab[0], p_ab[1], p_ab[2], q_ab[0], q_ab[1], q_ab[2], q_ab[3]]
    return t_ab_measured


class Calib(object):
    # CALIB: https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats#intrinsic_camera_calibration_of_the_kinect
    def __init__(self, K=np.eye(3), PP=freiburg_PP, focal=freiburg_Focal):  # TODO
        self.K = K
        self.PRINCIPAL_POINT = PP
        self.FOCAL = focal
        self.K[0, 0] = self.FOCAL[0]
        self.K[1, 1] = self.FOCAL[1]
        self.K[0, 2] = self.PRINCIPAL_POINT[0]
        self.K[1, 2] = self.PRINCIPAL_POINT[1]
        # self.K = [[525.0, 0, 319.5], [0, 525.0, 239.5], [0, 0, 1]]
        # self.PRINCIPAL_POINT = [319.5, 239.5]
        # self.FOCAL = [525.0, 525.0]
        self.K_inv = np.linalg.inv(np.array(self.K))


class CameraPoseObject(object):
    def __init__(self, position=np.zeros(3), rotation=np.eye(3), matrix44=None):
        if matrix44 is not None:
            self.matrix44 = matrix44
            self.position = matrix44[0:3, 3]
            self.rotation = matrix44[0:3, 0:3]
        else:
            self.position = position  # twc
            self.rotation = rotation  # .dot(R)  # Rwc
            self.matrix44 = np.eye(4)  # Twc
            self.matrix44[:3, :3] = rotation
            self.matrix44[:3, 3] = position
        xyzw = matrix2quaternion(self.rotation)
        self.quaternion = xyzw
        #self.quaternion = [xyzw[3], xyzw[0], xyzw[1], xyzw[2]]
        #self.opt = OptimizationCameraPose()

    def update(self, pose):
        self.position = pose[:3]
        self.quaternion = pose[3:]
        self.rotation = quaternion2matrix(self.quaternion)
        self.matrix44[:3, :3] = self.rotation
        self.matrix44[:3, 3] = self.position

    def update_with_matrix(self, matrix):
        self.matrix44 = matrix
        self.position = self.matrix44[:3, 3]
        self.rotation = self.matrix44[:3, :3]
        self.quaternion = matrix2quaternion(self.rotation)

    def transform2image(self, K):
        image2cam = np.array(K)
        R_cam2world = self.rotation.transpose()
        t_cam2world = -R_cam2world.dot(self.position)
        R_im2world = image2cam.dot(R_cam2world)
        t_im2world = image2cam.dot(t_cam2world)
        return R_im2world, t_im2world

    def get_P(self, K):
        R_im2world, t_im2world = self.transform2image(K)
        return np.hstack([R_im2world, t_im2world.reshape([3, 1])])

    def transformFrom(self, p):
        # return self.rotation.T.dot(p) - self.rotation.T.dot(self.position)
        return self.rotation.dot(p) + self.position  # Rwc*pc+twc

    def transformFromInverse(self, pw):
        R_cam2world = self.rotation.transpose()
        t_cam2world = -R_cam2world.dot(self.position)
        return np.dot(R_cam2world, pw) + t_cam2world



    '''def optimize_with_quadric_info(self, qs, K):
        self.position[0] = -0.9
        self.position[1] = -2.6
        self.position[2] = 1.4
        self.opt.initialize_pose(self.position, self.quaternion)
        print(self.position, self.quaternion)
        for q in qs:
            para_q = np.array([q.euler, q.position, q.radii]).reshape(9)
            bbox = q.BBoxes[-1]
            self.opt.add_residual_bbox(para_q, bbox, K)
        self.opt.optimize()
        print(self.opt.p)'''

    def distance_from(self, pose_prev):
        T12 = pose2pose_transform(self.matrix44, pose_prev.matrix44)
        return transform2distance(T12)


class OdometryWrapper(object):
    """Example odometry system.

    Parameters
    ----------
    intrinsics: ndarray
        the 3x3 intrinsic calibration matrix

    """

    def __init__(self, intrinsics, method='Rgbd'):
        if method == 'Rgbd':
            self.odometry = cv2.rgbd.RgbdOdometry_create(intrinsics)
        elif method == 'ICP':
            self.odometry = cv2.rgbd.ICPOdometry_create(intrinsics)
        elif method == 'Fast':
            self.odometry = cv2.rgbd.FastICPOdometry_create(intrinsics)
        elif method == 'RgbdICP':
            self.odometry = cv2.rgbd.RgbdICPOdometry_create(intrinsics)
        self.prev_image = None
        self.prev_depth = None

    def compute(self, gray, depth, prev_pose):
        """Estimate odometry from new rgb+depth images.

        Parameters
        ----------
        gray : ndarray (h,w) np.uint8
            Grayscale image from camera.
        depth : ndarray (h,w) np.float32
            Depth image in meters.

        Returns
        -------
        current pose
        """
        if self.prev_image is None:
            self.prev_image = gray
            self.prev_depth = depth
            return prev_pose  # first frame

        transform = np.ones((4, 4))
        mask = np.ones(gray.shape[0:2], np.uint8)
        self.odometry.compute(self.prev_image, self.prev_depth, mask, gray, depth, mask, transform)
        # odom = gtsam.Pose3(transform).inverse()
        # print('tra:', transform_inverse(transform)[0:3, 3])
        current_pose = np.dot(transform_inverse(transform), prev_pose)

        self.prev_image = gray
        self.prev_depth = depth

        return current_pose
