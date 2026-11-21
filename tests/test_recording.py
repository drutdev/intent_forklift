import os
import shutil
import sysconfig
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from pallet_engagement.recording import Recorder


class LiveRecordingChecks(unittest.TestCase):
    def test_viewer_uses_python_environment_when_not_on_path(self):
        viewer = shutil.which("rerun", path=sysconfig.get_path("scripts"))
        self.assertIsNotNone(viewer)
        with (
            tempfile.TemporaryDirectory() as root,
            patch.dict(os.environ, {"PATH": os.defpath}),
            patch("rerun.spawn") as spawn,
        ):
            self.assertIsNone(shutil.which("rerun"))
            recorder = Recorder(Path(root) / "live", {}, live=True)
            try:
                spawn.assert_called_once_with(
                    recording=recorder.live, executable_path=viewer
                )
            finally:
                self.assertTrue(recorder.close())
