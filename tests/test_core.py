import unittest
from robot_inertia_diff import analyze


class InertiaTests(unittest.TestCase):
    def xml(self, rpy="0 0 0", xx=1, yy=2):
        return f'<robot><link name="x"><inertial><origin rpy="{rpy}"/><mass value="1"/><inertia ixx="{xx}" iyy="{yy}" izz="3" ixy="0" ixz="0" iyz="0"/></inertial></link></robot>'

    def test_rotated_equivalence(self):
        r = analyze(
            {"before": self.xml(), "after": self.xml("0 0 1.5707963267948966", 2, 1)}
        )
        self.assertTrue(r["equivalent_within_tolerance"])

    def test_mass_change(self):
        a = self.xml()
        self.assertFalse(
            analyze({"before": a, "after": a.replace('value="1"', 'value="2"')})[
                "equivalent_within_tolerance"
            ]
        )

    def test_invalid_tensor_warns(self):
        a = self.xml(xx=-1)
        self.assertEqual(
            len(analyze({"before": a, "after": a})["physical_warnings"]), 2
        )

    def test_dtd_rejected(self):
        with self.assertRaises(ValueError):
            analyze({"before": "<!DOCTYPE x><robot/>", "after": "<robot/>"})

    def test_no_inertial(self):
        self.assertTrue(
            analyze(
                {
                    "before": '<robot><link name="x"/></robot>',
                    "after": '<robot><link name="x"/></robot>',
                }
            )["equivalent_within_tolerance"]
        )
