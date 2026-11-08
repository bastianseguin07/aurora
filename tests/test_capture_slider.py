"""Check slider mode routing without a display or a physical sensor."""

import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from gui_gtk import AuroraGUI


class CaptureSliderTests(unittest.TestCase):
    def capture(self, seconds, mobile=False):
        ui = Mock()
        ui._capture_request = None
        ui._pending_pose_capture = None
        ui.quick_capture_duration_scale.get_value.return_value = seconds
        ui.capture_persistence_entry.get_text.return_value = "0.6"
        ui.capture_limit_fov_check.get_active.return_value = False
        ui.capture_forward_axis_combo.get_active_text.return_value = "z"

        def run_worker(*, target, daemon):
            thread = Mock()
            thread.start.side_effect = target
            return thread

        with (
            patch("gui_gtk.threading.Thread", side_effect=run_worker),
            patch("gui_gtk.aurora_sensor.capture_snapshot") as snapshot,
            patch("gui_gtk.aurora_sensor.capture_aligned_snapshot") as aligned,
            patch("gui_gtk.aurora_sensor.capture_reference_frame", return_value=(None, None)),
            patch("gui_gtk.aurora_sensor.get_current_pose"),
        ):
            aligned.return_value.accepted_frames = 3
            aligned.return_value.rejected_frames = 0
            aligned.return_value.path_length_m = 0.1
            aligned.return_value.final_translation_m = 0.1
            aligned.return_value.final_rotation_deg = 1.0
            AuroraGUI._capture_clicked(
                ui, "base", aligned_multiview=mobile,
                duration_s=5 if mobile else None,
            )
        # A worker exception would instead dispatch the failure handler.
        self.assertNotIn(ui._on_capture_failed, [call.args[0] for call in ui._ui.call_args_list])
        return ui, snapshot, aligned

    def test_left_endpoint_uses_one_native_frame(self):
        ui, snapshot, aligned = self.capture(0)
        kwargs = snapshot.call_args.kwargs
        self.assertFalse(kwargs["accumulate_static"])
        self.assertEqual(kwargs["capture_duration_s"], 0)
        self.assertEqual(kwargs["stabilization_frames"], 1)
        self.assertEqual(kwargs["persistence_ratio"], 0)
        self.assertIsNone(ui._start_capture_progress.call_args.args[1])
        aligned.assert_not_called()

    def test_positive_slider_values_enable_stationary_accumulation(self):
        for seconds in (1, 10, 60):
            with self.subTest(seconds=seconds):
                ui, snapshot, aligned = self.capture(seconds)
                kwargs = snapshot.call_args.kwargs
                self.assertTrue(kwargs["accumulate_static"])
                self.assertEqual(kwargs["capture_duration_s"], seconds)
                self.assertEqual(kwargs["stabilization_frames"], 15)
                self.assertEqual(ui._start_capture_progress.call_args.args[1], seconds)
                aligned.assert_not_called()

    def test_experimental_mobile_uses_its_own_duration(self):
        _, snapshot, aligned = self.capture(0, mobile=True)
        snapshot.assert_not_called()
        self.assertEqual(aligned.call_args.kwargs["duration_s"], 5)


if __name__ == "__main__":
    unittest.main()
