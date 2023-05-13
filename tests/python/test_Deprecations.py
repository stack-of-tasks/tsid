import sys
import unittest
import warnings

import numpy as np
import tsid


class DeprecationTest(unittest.TestCase):
    def test_trajectory(self):
        q_ref = np.ones(5)
        traj_euclidian = tsid.TrajectoryEuclidianConstant("traj_eucl", q_ref)

        if sys.version_info >= (3, 2):
            with self.assertWarns(UserWarning):
                traj_euclidian.computeNext().pos()
            with self.assertWarns(UserWarning):
                traj_euclidian.computeNext().vel()
            with self.assertWarns(UserWarning):
                traj_euclidian.computeNext().acc()
        else:
            with warnings.catch_warnings(record=True) as w:
                self.assertEqual(len(w), 0)

                traj_euclidian.computeNext().pos()
                self.assertEqual(len(w), 1)
                self.assertEqual(w[-1].category, UserWarning)

                traj_euclidian.computeNext().vel()
                self.assertEqual(len(w), 2)
                self.assertEqual(w[-1].category, UserWarning)

                traj_euclidian.computeNext().acc()
                self.assertEqual(len(w), 3)
                self.assertEqual(w[-1].category, UserWarning)


if __name__ == "__main__":
    unittest.main()
