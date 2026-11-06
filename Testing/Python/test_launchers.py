"""Launcher regression tests; run with system Python, without Slicer or ROS 2."""

import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest


class LauncherTests(unittest.TestCase):
    def test_source_copied_and_symlinked_launchers(self):
        scripts = Path(__file__).resolve().parents[2] / "scripts"
        recorder = """#!/usr/bin/env python3
import json, os, sys
with open(os.environ['LAUNCHER_CAPTURE'], 'w') as stream:
    json.dump(sys.argv[1:], stream)
sys.exit(int(os.environ['LAUNCHER_EXIT']))
"""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source" / "scripts"
            source.mkdir(parents=True)
            standalone = source.parent / "launch_slicer.sh"
            standalone.write_text(recorder)
            standalone.chmod(0o755)
            fake_bin = root / "bin"
            fake_bin.mkdir()
            ros2 = fake_bin / "ros2"
            ros2.write_text(recorder)
            ros2.chmod(0o755)
            capture = root / "arguments.json"
            environment = dict(os.environ)
            environment.update(
                PATH=str(fake_bin) + os.pathsep + environment.get("PATH", ""),
                LAUNCHER_CAPTURE=str(capture),
            )
            for name in ("slicer", "test", "test_ci"):
                shutil.copy2(scripts / name, source / name)

            for layout in ("source", "copied", "symlinked"):
                directory = source
                if layout != "source":
                    directory = root / layout / "lib" / "slicer_ros2_module"
                    directory.mkdir(parents=True)
                    for name in ("slicer", "test", "test_ci"):
                        if layout == "symlinked":
                            (directory / name).symlink_to(source / name)
                        else:
                            shutil.copy2(source / name, directory / name)
                for name in ("slicer", "test", "test_ci"):
                    for exit_code in (0, 23):
                        with self.subTest(layout=layout, script=name, exit_code=exit_code):
                            environment["LAUNCHER_EXIT"] = str(exit_code)
                            extra = ["--python-script", "/tmp/script with spaces.py"] if name == "slicer" else []
                            result = subprocess.run(
                                [sys.executable, str(directory / name), *extra],
                                env=environment, capture_output=True, text=True,
                            )
                            self.assertEqual(result.returncode, exit_code, result.stderr)
                            arguments = json.loads(capture.read_text())
                            if layout != "source":
                                self.assertEqual(arguments[:3], [
                                    "launch", "slicer_ros2_module", "slicer.launch.py",
                                ])
                                self.assertTrue(arguments[3].startswith("slicer_args:="))
                                arguments = shlex.split(arguments[3].split("=", 1)[1])
                            self.assertNotIn("--modules-to-ignore", arguments)
                            if name == "slicer":
                                self.assertEqual(arguments, extra)
                            else:
                                self.assertIn("--testing", arguments)
                                code = arguments[arguments.index("--python-code") + 1]
                                compile(code, "<forwarded-test-code>", "exec")
                                self.assertIn("tests.run()" if name == "test" else "base64.b64decode", code)
                                if name == "test_ci":
                                    self.assertIn("--no-main-window", arguments)


if __name__ == "__main__":
    unittest.main()
