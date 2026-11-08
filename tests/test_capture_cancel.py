import sys
import tempfile
import threading
import time
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch
import numpy as np
import open3d as o3d
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from gui_gtk import AuroraGUI, Gtk, GLib, CSS, Gdk, CaptureRequest, aurora_sensor
from pointcloud_core import save_reference_pose
from unitree_pose import PoseHistory
from unitree_sensor import UnitreeCloud, UnitreeConfig
from test_unitree import frame


class CaptureCancelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not Gtk.init_check([])[0]:raise unittest.SkipTest('Sin pantalla GTK')
        provider=Gtk.CssProvider();provider.load_from_data(CSS)
        Gtk.StyleContext.add_provider_for_screen(Gdk.Screen.get_default(),provider,Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)

    def setUp(self):
        self.directory=tempfile.TemporaryDirectory();self.root=Path(self.directory.name)
        self.window=Gtk.Window();self.app=AuroraGUI(self.window)
        self.pose=PoseHistory._pose(dict(position=[1,2,3],quaternion_xyzw=[0,0,0,1],
                                       stamp_ns=time.time_ns(),frame_id='unitree_lio_origin_cancel_test'))
        self.cloud=UnitreeCloud([frame(2)],UnitreeConfig())
        self.before,self.after=self.root/'before.ply',self.root/'after.ply'
        for path in (self.before,self.after):
            o3d.io.write_point_cloud(str(path),self.cloud.geometry)
            save_reference_pose(path,*[self.pose.position,self.pose.rpy_deg],source_pose=self.pose)
        self.app._set_base_path(str(self.before));self.app._set_updated_path(str(self.after))
        self.app.scene_panel.feed.set_reference(self.pose)
        self.app.result=object()
        self.app._active_result_kind='cloud_to_cloud'
        self.app.sensor_connection=object()
        self.snapshot=(self.app.base_path,self.app.updated_path,self.app.imu_reference_pose,
                       self.app.scene_panel.feed.reference,self.app.result)
        self.files={p.name:p.read_bytes() for p in self.root.iterdir()}
        self.capabilities=patch('gui_gtk.aurora_sensor.capabilities',return_value=replace(aurora_sensor.UNITREE,pose=True))
        self.capabilities.start()
        self.app._set_sensor_ui_state(True)
        self.current=patch('gui_gtk.aurora_sensor.get_current_pose',return_value=self.pose);self.current.start()
        self.pose_capture=patch('gui_gtk.aurora_sensor.get_capture_pose',return_value=self.pose);self.pose_capture.start()
        self.photo=patch('gui_gtk.aurora_sensor.capture_reference_frame',return_value=(None,None));self.photo.start()
        self.save=patch.object(self.app,'_save_file_dialog',return_value=str(self.root/'new_capture.ply'))
        self.save_dialog=self.save.start()
        self.errors=patch.object(self.app,'_show_error');self.error_dialog=self.errors.start()

    def tearDown(self):
        request=self.app._capture_request
        if request is not None:
            request.cancelled.set();request.stop_event.set()
            self.wait(lambda:self.app._capture_request is None)
        self.app.sensor_connection=None
        self.app._on_close();self.window.destroy()
        self.capabilities.stop();self.current.stop();self.pose_capture.stop();self.photo.stop();self.save.stop();self.errors.stop()
        self.directory.cleanup()

    def wait(self,condition,timeout=3):
        end=time.monotonic()+timeout
        while not condition() and time.monotonic()<end:
            while GLib.MainContext.default().pending():GLib.MainContext.default().iteration(False)
            time.sleep(.003)
        self.assertTrue(condition(),'No terminó la operación de captura')

    def assert_preserved(self):
        before,after,imu,visual,result=self.snapshot
        self.assertEqual(self.app.base_path,before);self.assertEqual(self.app.updated_path,after)
        self.assertIs(self.app.imu_reference_pose,imu)
        self.assertIs(self.app.scene_panel.feed.reference,visual)
        self.assertIs(self.app.result,result)
        self.assertEqual({p.name:p.read_bytes() for p in self.root.iterdir()},self.files)
        self.save_dialog.assert_not_called();self.error_dialog.assert_not_called()
        self.assertFalse(self.app.quick_capture_cancel_button.get_visible())
        self.assertTrue(self.app.quick_capture_base_button.get_sensitive())
        self.assertTrue(self.app.quick_capture_updated_button.get_sensitive())

    def test_cancel_before_and_after_discards_partial_cloud_and_backend_error(self):
        for target,fail in [('base',False),('updated',False),('base',True),('updated',True)]:
            with self.subTest(target=target,fail=fail):
                entered=threading.Event()
                def capture(_connection,**kwargs):
                    entered.set();kwargs['stop_event'].wait(2)
                    if fail:raise RuntimeError('Cancelado antes de recibir puntos')
                    return self.cloud
                with patch('gui_gtk.aurora_sensor.capture_snapshot',side_effect=capture):
                    self.app._capture_clicked(target,wait_for_pose=False)
                    self.assertTrue(entered.wait(1))
                    self.assertTrue(self.app.quick_capture_cancel_button.get_visible())
                    self.app.quick_capture_cancel_button.clicked()
                    self.wait(lambda:self.app._capture_request is None)
                self.assert_preserved()

    def test_cancel_after_worker_finishes_but_before_queued_result_is_saved(self):
        calls=[]
        with patch.object(self.app,'_ui',side_effect=lambda f,*a,**k:calls.append((f,a,k))),\
             patch('gui_gtk.aurora_sensor.capture_snapshot',return_value=self.cloud):
            self.app._capture_clicked('base',wait_for_pose=False)
            end=time.monotonic()+2
            while not calls and time.monotonic()<end:time.sleep(.003)
            self.assertTrue(calls)
            self.app._cancel_capture_clicked()
            for func,args,kwargs in calls:func(*args,**kwargs)
        self.assert_preserved()

    def test_cancel_while_preparing_pose_never_starts_capture(self):
        self.app.sensor_selector.set_active_id('unitree')
        self.app.unitree_transport_combo.set_active_id('ros2')
        with patch('gui_gtk.aurora_sensor.capabilities',return_value=aurora_sensor.UNITREE),\
             patch.object(self.app,'_start_unitree_tracking'),\
             patch('gui_gtk.aurora_sensor.capture_snapshot') as capture:
            self.app._capture_clicked('base')
            self.assertIsNotNone(self.app._pending_pose_capture)
            self.app._cancel_capture_clicked()
            self.assertIsNone(self.app._pending_pose_capture)
            self.assertEqual(self.app._capture_when_tracking_ready(),GLib.SOURCE_REMOVE)
            capture.assert_not_called()
        self.assert_preserved()

    def test_late_callbacks_cannot_overwrite_a_new_capture(self):
        old=CaptureRequest();old.cancelled.set()
        new=CaptureRequest();self.app._capture_request=new
        self.app.capture_stop_event=new.stop_event
        self.app._on_capture_done('base',self.cloud,capture_request=old)
        self.app._on_capture_failed(RuntimeError('old failure'),capture_request=old)
        self.app._on_capture_cancelled(old)
        self.assertIs(self.app._capture_request,new)
        self.app._capture_request=None;self.app.capture_stop_event=None
        self.assert_preserved()

    def test_capture_after_cancellation_can_save_normally(self):
        request=CaptureRequest();self.app._capture_request=request
        self.app._start_capture_progress('Capturando',10)
        self.app._cancel_capture_clicked();self.app._on_capture_cancelled(request)
        self.assert_preserved()
        with patch('gui_gtk.aurora_sensor.capture_snapshot',return_value=self.cloud):
            self.app._capture_clicked('base',wait_for_pose=False)
            self.wait(lambda:self.app._capture_request is None)
        self.save_dialog.assert_called_once()
        self.assertTrue((self.root/'new_capture.ply').exists())
        self.assertEqual(self.app.base_path,str(self.root/'new_capture.ply'))
        self.assertEqual(self.app.updated_path,'')
        self.assertIsNone(self.app.result)

    def test_button_only_visible_during_capture_and_does_not_shift_controls(self):
        self.window.show_all()
        def settle():
            end=time.monotonic()+.12
            while time.monotonic()<end:
                while GLib.MainContext.default().pending():GLib.MainContext.default().iteration(False)
                time.sleep(.003)
        def allocations():
            return [(w.get_allocation().x,w.get_allocation().y,w.get_allocation().width,w.get_allocation().height)
                    for w in (self.app.quick_capture_base_button,self.app.quick_capture_updated_button,
                              self.app.quick_capture_duration_scale,self.app.quick_capture_profile,
                              self.app.quick_crop_face_size_entry,self.app.quick_calculate_button,
                              self.app.quick_capture_status_label)]
        settle();initial=allocations()
        self.assertFalse(self.app.quick_capture_cancel_button.get_visible())
        self.app._start_capture_progress('Capturando ANTES',10)
        settle();self.assertTrue(self.app.quick_capture_cancel_button.get_visible())
        self.assertEqual(allocations(),initial)
        self.app._finish_capture_progress();settle()
        self.assertFalse(self.app.quick_capture_cancel_button.get_visible())
        self.assertEqual(allocations(),initial)


if __name__=='__main__':unittest.main()
