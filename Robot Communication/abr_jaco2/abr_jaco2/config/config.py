import abr_control
import numpy as np
import sympy as sp
from abr_control.arms.base_config import BaseConfig
from abr_control.utils.paths import cache_dir


class Config(BaseConfig):
    """Robot config file for the Kinova Jaco^2 V2 with force sensors

    Attributes
    ----------
    START_ANGLES : numpy.array
        The joint angles for a safe home or rest position
    _M_LINKS : sympy.diag
        inertia matrix of the links
    _M_JOINTS : sympy.diag
        inertia matrix of the joints
    L : numpy.array
        segment lengths of arm [meters]
    L_HANDCOM : numpy.array
        offset to the center of mass of the hand [meters]

    Transform Naming Convention: Tpoint1point2
    ex: Tj1l1 transforms from joint 1 to link 1

    Transforms are broken up into two matrices for simplification
    ex: Tj0l1a and Tj0l1b where the former transform accounts for
    joint rotations and the latter accounts for static rotations
    and translations
    """

    def __init__(self, **kwargs):

        N_LINKS = 7
        N_JOINTS = 6

        self._T = {}  # dictionary for storing calculated transforms

        super().__init__(
            N_JOINTS=N_JOINTS, N_LINKS=N_LINKS, ROBOT_NAME="jaco2", **kwargs
        )

        # set up saved functions folder to be in the abr_jaco repo
        self.config_folder = cache_dir + "/abr_jaco2/saved_functions_"
        self.config_folder += "_" + self.config_hash
        # make folder if it doesn't exist
        abr_control.utils.os_utils.makedirs(self.config_folder)

        # position to move to before switching to torque mode
        self.START_ANGLES = np.array(
            [3.14, 0.0, 0.0, 3.14, 0.0, 1.57], dtype="float32"
        )

        # position to move to before switching to torque mode
        self.JACO_HOME = np.array(
            [3.14, 4.18879, 0, 1.0472, 1.5708, 1.5708], dtype="float32"
        )

        # position to move to before switching to torque mode
        self.JACO_HORIZONTAL_POSE = np.array(
            [3.14, 3.14, 3.14, 3.14, 3.14, 3.14], dtype="float32"
        )

        # create inertia matrices for each link of the Kinova Jaco^2
        self._M_LINKS = [
            sp.Matrix(
                [  # link0
                    [0.640, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.640, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.640, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.1, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.1, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
                ]
            ),
            sp.Matrix(
                [  # link1
                    [0.182, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.182, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.182, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.15, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.15, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.15],
                ]
            ),
            sp.Matrix(
                [  # link2
                    [0.424, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.424, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.424, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.15, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.15, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.15],
                ]
            ),
            sp.Matrix(
                [  # link3
                    [0.211, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.211, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.2113, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.125, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.125, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.125],
                ]
            ),
            sp.Matrix(
                [  # link4
                    [0.069, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.069, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.069, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.025, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.025, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.025],
                ]
            ),
            sp.Matrix(
                [  # link5
                    [0.069, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.069, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.069, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.025, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.025, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.025],
                ]
            ),
            sp.Matrix(
                [  # hand
                    [0.727, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.727, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.727, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.025, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.025, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.025],
                ]
            ),
        ]

        # create inertia matrices for each joint (motor) of the Kinova Jaco^2
        self._M_JOINTS = [  # mass of rings added
            sp.Matrix(
                [  # motor 0
                    [0.586, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.586, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.586, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.15, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.15, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.15],
                ]
            ),
            sp.Matrix(
                [  # motor 1
                    [0.572, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.572, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.572, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.15, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.15, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.15],
                ]
            ),
            sp.Matrix(
                [  # motor2
                    [0.586, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.586, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.586, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.15, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.15, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.15],
                ]
            ),
            sp.Matrix(
                [  # motor3
                    [0.348, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.348, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.348, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.15, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.15, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.15],
                ]
            ),
            sp.Matrix(
                [  # motor4
                    [0.348, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.348, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.348, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.15, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.15, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.15],
                ]
            ),
            sp.Matrix(
                [  # motor5
                    [0.348, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.348, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.348, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.15, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.15, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.15],
                ]
            ),
        ]

        # segment lengths associated with each transform
        # ignoring lengths < 1e-6
        self.L = [
            [0.0, 0.0, 7.8369e-02],  # link 0 offset
            [-3.2712e-05, -1.7324e-05, 7.8381e-02],  # joint 0 offset
            [2.1217e-05, 4.8455e-05, -7.9515e-02],  # link 1 offset
            [-2.2042e-05, 1.3245e-04, -3.8863e-02],  # joint 1 offset
            [-1.9519e-03, 2.0902e-01, -2.8839e-02],  # link 2 offset
            [-2.3094e-02, -1.0980e-06, 2.0503e-01],  # joint 2 offset
            [-4.8786e-04, -8.1945e-02, -1.2931e-02],  # link 3 offset
            [2.5923e-04, -3.8935e-03, -1.2393e-01],  # joint 3 offset
            [-4.0053e-04, 1.2581e-02, -3.5270e-02],  # link 4 offset
            [-2.3603e-03, -4.8662e-03, 3.7097e-02],  # joint 4 offset
            [-5.2974e-04, 1.2272e-02, -3.5485e-02],  # link 5 offset
            [-1.9534e-03, 5.0298e-03, -3.7176e-02],  # joint 5 offset
            [0.0003, 0.00684, -0.08222],  # distance to palm
            [0.0, 0.0, 0.12],
        ]  # distance to fingertips

        self.L = np.array(self.L)

        # ---- Transform Matrices ----

        # Transform matrix : origin -> link 0
        # no change of axes, account for offsets
        self.Torgl0 = sp.Matrix(
            [
                [1, 0, 0, self.L[0, 0]],
                [0, 1, 0, self.L[0, 1]],
                [0, 0, 1, self.L[0, 2]],
                [0, 0, 0, 1],
            ]
        )

        # Transform matrix : link0 -> joint 0
        # account for change of axes and offsets
        self.Tl0j0 = sp.Matrix(
            [
                [1, 0, 0, self.L[1, 0]],
                [0, -1, 0, self.L[1, 1]],
                [0, 0, -1, self.L[1, 2]],
                [0, 0, 0, 1],
            ]
        )

        # Transform matrix : joint 0 -> link 1
        # account for rotations due to q
        self.Tj0l1a = sp.Matrix(
            [
                [sp.cos(self.q[0]), -sp.sin(self.q[0]), 0, 0],
                [sp.sin(self.q[0]), sp.cos(self.q[0]), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        # account for change of axes and offsets
        self.Tj0l1b = sp.Matrix(
            [
                [-1, 0, 0, self.L[2, 0]],
                [0, -1, 0, self.L[2, 1]],
                [0, 0, 1, self.L[2, 2]],
                [0, 0, 0, 1],
            ]
        )
        self.Tj0l1 = self.Tj0l1a * self.Tj0l1b

        # Transform matrix : link 1 -> joint 1
        # account for axes rotation and offset
        self.Tl1j1 = sp.Matrix(
            [
                [1, 0, 0, self.L[3, 0]],
                [0, 0, -1, self.L[3, 1]],
                [0, 1, 0, self.L[3, 2]],
                [0, 0, 0, 1],
            ]
        )

        # Transform matrix : joint 1 -> link 2
        # account for rotations due to q
        self.Tj1l2a = sp.Matrix(
            [
                [sp.cos(self.q[1]), -sp.sin(self.q[1]), 0, 0],
                [sp.sin(self.q[1]), sp.cos(self.q[1]), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        # account for axes rotation and offsets
        self.Tj1l2b = sp.Matrix(
            [
                [0, -1, 0, self.L[4, 0]],
                [0, 0, 1, self.L[4, 1]],
                [-1, 0, 0, self.L[4, 2]],
                [0, 0, 0, 1],
            ]
        )
        self.Tj1l2 = self.Tj1l2a * self.Tj1l2b

        # Transform matrix : link 2 -> joint 2
        # account for axes rotation and offsets
        self.Tl2j2 = sp.Matrix(
            [
                [0, 0, 1, self.L[5, 0]],
                [1, 0, 0, self.L[5, 1]],
                [0, 1, 0, self.L[5, 2]],
                [0, 0, 0, 1],
            ]
        )

        # Transform matrix : joint 2 -> link 3
        # account for rotations due to q
        self.Tj2l3a = sp.Matrix(
            [
                [sp.cos(self.q[2]), -sp.sin(self.q[2]), 0, 0],
                [sp.sin(self.q[2]), sp.cos(self.q[2]), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        # account for axes rotation and offsets
        self.Tj2l3b = sp.Matrix(
            [
                [0.14262926, -0.98977618, 0, self.L[6, 0]],
                [0, 0, 1, self.L[6, 1]],
                [-0.98977618, -0.14262926, 0, self.L[6, 2]],
                [0, 0, 0, 1],
            ]
        )
        self.Tj2l3 = self.Tj2l3a * self.Tj2l3b

        # Transform matrix : link 3 -> joint 3
        # account for axes change and offsets
        self.Tl3j3 = sp.Matrix(
            [
                [-0.14262861, -0.98977628, 0, self.L[7, 0]],
                [0.98977628, -0.14262861, 0, self.L[7, 1]],
                [0, 0, 1, self.L[7, 2]],
                [0, 0, 0, 1],
            ]
        )

        # Transform matrix: joint 3 -> link 4
        # account for rotations due to q
        self.Tj3l4a = sp.Matrix(
            [
                [sp.cos(self.q[3]), -sp.sin(self.q[3]), 0, 0],
                [sp.sin(self.q[3]), sp.cos(self.q[3]), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        # account for axes and rotation and offsets
        self.Tj3l4b = sp.Matrix(
            [
                [0.85536427, -0.51802699, 0, self.L[8, 0]],
                [-0.45991232, -0.75940555, 0.46019982, self.L[8, 1]],
                [-0.23839593, -0.39363848, -0.88781537, self.L[8, 2]],
                [0, 0, 0, 1],
            ]
        )
        self.Tj3l4 = self.Tj3l4a * self.Tj3l4b

        # Transform matrix: link 4 -> joint 4
        # no axes change, account for offsets
        self.Tl4j4 = sp.Matrix(
            [
                [-0.855753802, 0.458851168, 0.239041914, self.L[9, 0]],
                [0.517383113, 0.758601438, 0.396028500, self.L[9, 1]],
                [0, 0.462579144, -0.886577910, self.L[9, 2]],
                [0, 0, 0, 1],
            ]
        )

        # Transform matrix: joint 4 -> link 5
        # account for rotations due to q
        self.Tj4l5a = sp.Matrix(
            [
                [sp.cos(self.q[4]), -sp.sin(self.q[4]), 0, 0],
                [sp.sin(self.q[4]), sp.cos(self.q[4]), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        # account for axes and rotation and offsets
        # no axes change, account for offsets
        self.Tj4l5b = sp.Matrix(
            [
                [0.89059413, 0.45479896, 0, self.L[10, 0]],
                [-0.40329059, 0.78972966, -0.46225942, self.L[10, 1]],
                [-0.2102351, 0.41168552, 0.88674474, self.L[10, 2]],
                [0, 0, 0, 1],
            ]
        )
        self.Tj4l5 = self.Tj4l5a * self.Tj4l5b

        # Transform matrix : link 5 -> joint 5
        # account for axes change and offsets
        self.Tl5j5 = sp.Matrix(
            [
                [-0.890598824, 0.403618758, 0.209584432, self.L[11, 0]],
                [-0.454789710, -0.790154512, -0.410879747, self.L[11, 1]],
                [0, -0.461245863, 0.887272337, self.L[11, 2]],
                [0, 0, 0, 1],
            ]
        )

        # Transform matrix: joint 5 -> hand COM
        # account for rotations due to q
        self.Tj5handcoma = sp.Matrix(
            [
                [sp.cos(self.q[5]), -sp.sin(self.q[5]), 0, 0],
                [sp.sin(self.q[5]), sp.cos(self.q[5]), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        # account for axes changes and offsets
        self.Tj5handcomb = sp.Matrix(
            [
                [-1, 0, 0, self.L[12, 0]],
                [0, 1, 0, self.L[12, 1]],
                [0, 0, -1, self.L[12, 2]],
                [0, 0, 0, 1],
            ]
        )
        self.Tj5handcom = self.Tj5handcoma * self.Tj5handcomb

        # no axes change, account for offsets
        self.Thandcomfingers = sp.Matrix(
            [
                [1, 0, 0, self.L[13, 0]],
                [0, 1, 0, self.L[13, 1]],
                [0, 0, 1, self.L[13, 2]],
                [0, 0, 0, 1],
            ]
        )

        # Transform matrix: camera -> origin
        # account for rotation and offset
        self.Torgcama = sp.Matrix(
            [
                [sp.cos(-np.pi / 4.0), -np.sin(-np.pi / 4.0), 0.0, 0.024],
                [sp.sin(-np.pi / 4.0), sp.cos(-np.pi / 4.0), 0, -0.319],
                [0, 0, 1, 0.785],
                [0, 0, 0, 1],
            ]
        )
        # account for axes changes
        self.Torgcamb = sp.Matrix(
            [[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]]
        )
        self.Torgcam = self.Torgcama * self.Torgcamb

        # orientation part of the Jacobian (compensating for angular velocity)
        KZ = sp.Matrix([0, 0, 1])
        self.J_orientation = [
            self._calc_T("joint0")[:3, :3] * KZ,  # joint 0 orientation
            self._calc_T("joint1")[:3, :3] * KZ,  # joint 1 orientation
            self._calc_T("joint2")[:3, :3] * KZ,  # joint 2 orientation
            self._calc_T("joint3")[:3, :3] * KZ,  # joint 3 orientation
            self._calc_T("joint4")[:3, :3] * KZ,  # joint 4 orientation
            self._calc_T("joint5")[:3, :3] * KZ,
        ]  # joint 5 orientation

    def _calc_T(self, name):  # noqa C907
        """Uses Sympy to generate the transform for a joint or link

        name : string
            name of the joint, link, or end-effector
        """

        if self._T.get(name, None) is None:
            if name == "link0":
                self._T[name] = self.Torgl0
            elif name == "joint0":
                self._T[name] = self._calc_T("link0") * self.Tl0j0
            elif name == "link1":
                self._T[name] = self._calc_T("joint0") * self.Tj0l1
            elif name == "joint1":
                self._T[name] = self._calc_T("link1") * self.Tl1j1
            elif name == "link2":
                self._T[name] = self._calc_T("joint1") * self.Tj1l2
            elif name == "joint2":
                self._T[name] = self._calc_T("link2") * self.Tl2j2
            elif name == "link3":
                self._T[name] = self._calc_T("joint2") * self.Tj2l3
            elif name == "joint3":
                self._T[name] = self._calc_T("link3") * self.Tl3j3
            elif name == "link4":
                self._T[name] = self._calc_T("joint3") * self.Tj3l4
            elif name == "joint4":
                self._T[name] = self._calc_T("link4") * self.Tl4j4
            elif name == "link5":
                self._T[name] = self._calc_T("joint4") * self.Tj4l5
            elif name == "joint5":
                self._T[name] = self._calc_T("link5") * self.Tl5j5
            elif name == "link6":
                self._T[name] = self._calc_T("joint5") * self.Tj5handcom
            elif name == "EE":
                self._T[name] = self._calc_T("link6") * self.Thandcomfingers
            elif name == "camera":
                self._T[name] = self.Torgcam

            else:
                raise Exception(f"Invalid transformation name: {name}")

        return self._T[name]
