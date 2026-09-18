"""Static hygiene: no undefined names anywhere in the shipped sources.

``tools/*.py`` is not covered by the unit tests, so a missing import only shows
up when someone runs the tool on the bench with the camera attached - exactly
how ``apply_color_profile`` once reached ``main()`` without being imported.
Compiling is not enough (a name is resolved at run time), so this runs pyflakes
when it is available and fails on any "undefined name" it reports.
"""

from pathlib import Path
import shutil
import subprocess
import sys
import unittest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_DIRS = ("jetson_recognition", "tools", "tests")


def python_files():
    for directory in SOURCE_DIRS:
        for path in sorted((PROJECT_ROOT / directory).glob("*.py")):
            yield path


class SourceHygieneTest(unittest.TestCase):
    def test_every_source_file_compiles(self):
        failures = []
        for path in python_files():
            source = path.read_text(encoding="utf-8")
            try:
                compile(source, str(path), "exec")
            except SyntaxError as error:
                failures.append("%s: %s" % (path, error))
        self.assertEqual(failures, [])

    def test_no_undefined_names(self):
        if shutil.which("pyflakes") is None:
            self.skipTest("pyflakes is not installed")
        completed = subprocess.run(
            [sys.executable, "-m", "pyflakes"] + [str(p) for p in python_files()],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True,
        )
        undefined = [
            line
            for line in completed.stdout.splitlines()
            if "undefined name" in line or "undefined local" in line
        ]
        self.assertEqual(undefined, [])

    def test_tools_that_use_color_profiles_import_it(self):
        # The specific regression this file exists for.
        path = PROJECT_ROOT / "tools" / "test_turntable_color_live.py"
        source = path.read_text(encoding="utf-8")
        for name in ("apply_color_profile", "save_override"):
            if name + "(" not in source:
                continue
            self.assertIn(
                "from jetson_recognition.color_profiles import",
                source,
                "%s uses %s but never imports it" % (path.name, name),
            )


if __name__ == "__main__":
    unittest.main()
