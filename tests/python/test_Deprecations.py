import unittest

import numpy as np
import tsid


class DeprecationTest(unittest.TestCase):
    def test_trajectory(self):
        q_ref = np.ones(5)
        traj_euclidian = tsid.TrajectoryEuclidianConstant("traj_eucl", q_ref)

        with self.assertWarns(UserWarning):
            traj_euclidian.computeNext().pos()
        with self.assertWarns(UserWarning):
            traj_euclidian.computeNext().vel()
        with self.assertWarns(UserWarning):
            traj_euclidian.computeNext().acc()


if __name__ == "__main__":
    unittest.main()
