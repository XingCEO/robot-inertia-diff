import json
import subprocess
import sys
import unittest


class NativeFileTests(unittest.TestCase):
    def test_committed_native_fixture(self):
        result = subprocess.run(
            [sys.executable, "-m", "robot_inertia_diff"]
            + ["--before", "examples/before.urdf", "--after", "examples/after.urdf"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIsInstance(json.loads(result.stdout), dict)

    def test_missing_native_path_is_clean_error(self):
        args = ["--before", "examples/before.urdf", "--after", "examples/after.urdf"]
        args[1] = "nonexistent-test-input"
        result = subprocess.run(
            [sys.executable, "-m", "robot_inertia_diff"] + args,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 2)
        self.assertNotIn("Traceback", result.stderr)
