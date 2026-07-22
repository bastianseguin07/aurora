#!/usr/bin/env python3
# /*
#  *  SLAMTEC Aurora - Adaptación para MVP Proyecto CORFO Eureka
#  *  Medición de Espesor Shotcrete en Túneles Mineros
#  */
"""
Integra captura LiDAR y coloreado RGB.
- Interfaz Gráfica (GUI) en PyQt6.
- Eje de coordenadas rotado (X al frente).
- Límite de puntos eliminado.
- Visualización de la Nube Base (Ex-ante) manteniendo los colores originales para preservar la claridad.
"""

import sys
import os
import time
import signal
import threading
from datetime import datetime

# ==========================================
# ESTADO GLOBAL DE LA GUI Y MVP
# ==========================================
gui_state = {
    'max_dist': 5.0,        
    'point_size': 2,        
    'min_thick': 0.05,      
    'max_thick': 0.15,      
    'save_ply': False,      
    'capture_baseline': False,
    'clear_baseline': False,
    'baseline_pcd': None
}

is_ctrl_c = False
point_cloud_data = None
color_data = None
point_cloud_lock = threading.Lock()

def setup_sdk_import():
    try:
        from slamtec_aurora_sdk import (
            AuroraSDK, ENHANCED_IMAGE_TYPE_DEPTH, DEPTHCAM_FRAME_TYPE_POINT3D
        )
        return AuroraSDK, ENHANCED_IMAGE_TYPE_DEPTH, DEPTHCAM_FRAME_TYPE_POINT3D
    except ImportError:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'python_bindings'))
        from slamtec_aurora_sdk import (
            AuroraSDK, ENHANCED_IMAGE_TYPE_DEPTH, DEPTHCAM_FRAME_TYPE_POINT3D
        )
        return AuroraSDK, ENHANCED_IMAGE_TYPE_DEPTH, DEPTHCAM_FRAME_TYPE_POINT3D

AuroraSDK, ENHANCED_IMAGE_TYPE_DEPTH, DEPTHCAM_FRAME_TYPE_POINT3D = setup_sdk_import()

try:
    import numpy as np
    import open3d as o3d
    from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                                 QLabel, QSlider, QPushButton, QGroupBox)
    from PyQt6.QtCore import Qt, QTimer
except ImportError:
    print("Error: Faltan dependencias. Instala: pip install numpy open3d PyQt6")
    sys.exit(1)

def signal_handler(sig, frame):
    global is_ctrl_c
    is_ctrl_c = True

def calculate_thickness_colors(current_points, baseline_pcd, min_thickness, max_thickness):
    temp_pcd = o3d.geometry.PointCloud()
    temp_pcd.points = o3d.utility.Vector3dVector(current_points)
    
    distances = temp_pcd.compute_point_cloud_distance(baseline_pcd)
    distances = np.asarray(distances)
    avg_distance = np.mean(distances) if len(distances) > 0 else 0.0
    
    colors = np.ones((len(current_points), 3)) * 0.2 
    mask_falta = distances < min_thickness
    mask_ideal = (distances >= min_thickness) & (distances <= max_thickness)
    mask_exceso = distances > max_thickness
    
    colors[mask_falta] = [1.0, 0.0, 0.0]  # Rojo (Falta material)
    colors[mask_ideal] = [0.0, 1.0, 0.0]  # Verde (Óptimo)
    colors[mask_exceso] = [0.0, 0.0, 1.0] # Azul (Exceso)
    
    return colors, avg_distance

def _build_strided_uint8_image(data, width, height, channels, stride=0):
    if data is None: return None
    packed_row_bytes = width * channels
    row_stride = stride or packed_row_bytes
    required_size = packed_row_bytes + row_stride * (height - 1 if height > 1 else 0)
    if len(data) < required_size or row_stride < packed_row_bytes: return None
    shape = (height, width) if channels == 1 else (height, width, channels)
    strides = (row_stride, 1) if channels == 1 else (row_stride, channels, 1)
    try:
        return np.ndarray(shape=shape, dtype=np.uint8, buffer=data, strides=strides)
    except: return None

def extract_camera_rgb_image(camera_image):
    if camera_image is None or not getattr(camera_image, 'data', None): return None
    if hasattr(camera_image, 'to_numpy_image'):
        img_rgb = camera_image.to_numpy_image(color_order="rgb")
        if img_rgb is not None:
            return img_rgb.astype(np.float32) / 255.0
    img_height, img_width = camera_image.height, camera_image.width
    pixel_format, img_stride = camera_image.pixel_format, getattr(camera_image, 'stride', 0)
    img_data = camera_image.data
    
    if pixel_format == 0:
        gray = _build_strided_uint8_image(img_data, img_width, img_height, 1, img_stride)
        if gray is None: return None
        return np.repeat((gray.astype(np.float32) / 255.0)[:, :, np.newaxis], 3, axis=2)
    if pixel_format == 1:
        bgr = _build_strided_uint8_image(img_data, img_width, img_height, 3, img_stride)
        if bgr is None: return None
        return bgr[:, :, ::-1].astype(np.float32) / 255.0
    if pixel_format == 2:
        rgba = _build_strided_uint8_image(img_data, img_width, img_height, 4, img_stride)
        if rgba is None: return None
        return rgba[:, :, :3].astype(np.float32) / 255.0
    return None

def parse_point_cloud_data(frame, camera_image=None):
    if frame is None or not frame.is_point3d_frame(): return None, None, None
    width, height = frame.width, frame.height
    points_xyz = frame.to_point3d_array()
    
    if points_xyz is None: return None, None, None
    
    is_organized = (len(points_xyz) == width * height)
    valid_mask = np.ones(len(points_xyz), dtype=bool)
    valid_mask &= ~np.all(points_xyz == 0, axis=1)
    valid_mask &= np.all(np.isfinite(points_xyz), axis=1)
    
    # Filtro de Distancia
    valid_mask &= (np.linalg.norm(points_xyz, axis=1) < gui_state['max_dist'])
    valid_mask &= (np.linalg.norm(points_xyz, axis=1) > 0.1)
    
    original_indices = np.arange(len(points_xyz))
    valid_original_indices = original_indices[valid_mask]
    points_xyz = points_xyz[valid_mask]
    
    if len(points_xyz) == 0: return None, None, None
    
    # Color mapping
    colors_rgb = np.zeros((len(points_xyz), 3))
    color_mapped = False
    
    if camera_image is not None and hasattr(camera_image, 'data'):
        try:
            img_rgb = extract_camera_rgb_image(camera_image)
            if img_rgb is not None:
                img_height, img_width = camera_image.height, camera_image.width
                if is_organized and width == img_width and height == img_height:
                    for i, original_idx in enumerate(valid_original_indices):
                        colors_rgb[i] = img_rgb[original_idx // width, original_idx % width]
                elif is_organized:
                    for i, original_idx in enumerate(valid_original_indices):
                        r, c = original_idx // width, original_idx % width
                        img_c = max(0, min(int(c * img_width / width), img_width - 1))
                        img_r = max(0, min(int(r * img_height / height), img_height - 1))
                        colors_rgb[i] = img_rgb[img_r, img_c]
                color_mapped = True
        except Exception: pass
        
    if not color_mapped:
        heights = points_xyz[:, 1]
        height_norm = (heights - heights.min()) / (heights.max() - heights.min() + 1e-8)
        colors_rgb[:, 0] = height_norm * 0.8 + 0.1
        colors_rgb[:, 1] = (1 - height_norm) * 0.8 + 0.1
        colors_rgb[:, 2] = 0.5

    # === INVERTIR EJES Y & Z ===
    points_xyz[:, 1] *= -1.0
    points_xyz[:, 2] *= -1.0
            
    return points_xyz, colors_rgb, getattr(frame, 'timestamp_ns', 0)

def point_cloud_acquisition_thread(sdk, update_rate_hz):
    global point_cloud_data, color_data, is_ctrl_c
    frame_interval = 1.0 / update_rate_hz
    last_update = 0
    
    while not is_ctrl_c and not sdk.enhanced_imaging.is_depth_camera_ready(): time.sleep(0.1)

    while not is_ctrl_c:
        try:
            current_time = time.time()
            if current_time - last_update < frame_interval:
                time.sleep(0.01)
                continue
            
            if not sdk.enhanced_imaging.wait_depth_camera_next_frame(100): continue
            
            frame = sdk.enhanced_imaging.peek_depth_camera_frame(DEPTHCAM_FRAME_TYPE_POINT3D)
            if frame and frame.data:
                camera_image = None
                if hasattr(frame, 'timestamp_ns') and frame.timestamp_ns > 0:
                    try:
                        camera_image = sdk.enhanced_imaging.peek_depth_camera_related_rectified_image(frame.timestamp_ns)
                    except: pass
                
                # Sin limite de puntos
                points, colors, _ = parse_point_cloud_data(frame, camera_image)
                
                if points is not None and colors is not None:
                    with point_cloud_lock:
                        point_cloud_data = points.copy()
                        color_data = colors.copy()
                    last_update = current_time
            time.sleep(0.01)
        except Exception: time.sleep(0.1)

# ==========================================
# PyQt6 MainWindow
# ==========================================
class MainWindow(QMainWindow):
    def __init__(self, vis, pcd):
        super().__init__()
        self.vis = vis
        self.pcd = pcd
        self.first_frame = True
        self.last_print_time = time.time()
        
        # Nube de puntos extra para visualizar la linea base
        self.baseline_vis_pcd = o3d.geometry.PointCloud()
        self.baseline_vis_pcd.points = o3d.utility.Vector3dVector(np.empty((0, 3)))
        self.baseline_vis_pcd.colors = o3d.utility.Vector3dVector(np.empty((0, 3)))
        self.vis.add_geometry(self.baseline_vis_pcd, reset_bounding_box=False)
        self.update_baseline_vis = False
        
        self.setWindowTitle("Controles MVP Minero - Shotcrete")
        self.resize(380, 520)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # --- Configuración UI ---
        
        self.lbl_dist = QLabel(f"Distancia Máxima: {gui_state['max_dist']:.1f} m")
        self.lbl_dist.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.lbl_dist)
        self.sld_dist = QSlider(Qt.Orientation.Horizontal)
        self.sld_dist.setRange(5, 200)
        self.sld_dist.setValue(int(gui_state['max_dist'] * 10))
        self.sld_dist.valueChanged.connect(self.update_dist)
        layout.addWidget(self.sld_dist)
        
        self.lbl_pt = QLabel(f"Tamaño de Punto (3D): {gui_state['point_size']}")
        self.lbl_pt.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.lbl_pt)
        self.sld_pt = QSlider(Qt.Orientation.Horizontal)
        self.sld_pt.setRange(1, 10)
        self.sld_pt.setValue(gui_state['point_size'])
        self.sld_pt.valueChanged.connect(self.update_pt)
        layout.addWidget(self.sld_pt)
        
        self.lbl_min = QLabel(f"Espesor Mínimo (Falta - Rojo): {int(gui_state['min_thick']*100)} cm")
        self.lbl_min.setStyleSheet("font-weight: bold; color: #D32F2F;")
        layout.addWidget(self.lbl_min)
        self.sld_min = QSlider(Qt.Orientation.Horizontal)
        self.sld_min.setRange(0, 50)
        self.sld_min.setValue(int(gui_state['min_thick'] * 100))
        self.sld_min.valueChanged.connect(self.update_min)
        layout.addWidget(self.sld_min)
        
        self.lbl_max = QLabel(f"Espesor Máximo (Exceso - Azul): {int(gui_state['max_thick']*100)} cm")
        self.lbl_max.setStyleSheet("font-weight: bold; color: #1976D2;")
        layout.addWidget(self.lbl_max)
        self.sld_max = QSlider(Qt.Orientation.Horizontal)
        self.sld_max.setRange(0, 50)
        self.sld_max.setValue(int(gui_state['max_thick'] * 100))
        self.sld_max.valueChanged.connect(self.update_max)
        layout.addWidget(self.sld_max)
        
        layout.addSpacing(15)
        
        self.btn_baseline = QPushButton("Fijar Línea Base (Ex-ante)")
        self.btn_baseline.setMinimumHeight(45)
        self.btn_baseline.setStyleSheet("background-color: #546E7A; color: white; font-weight: bold; font-size: 14px;")
        self.btn_baseline.clicked.connect(self.capture_baseline)
        layout.addWidget(self.btn_baseline)
        
        self.btn_clear_baseline = QPushButton("Borrar Línea Base")
        self.btn_clear_baseline.setMinimumHeight(35)
        self.btn_clear_baseline.setStyleSheet("background-color: #FF7043; color: white; font-weight: bold;")
        self.btn_clear_baseline.clicked.connect(self.clear_baseline)
        layout.addWidget(self.btn_clear_baseline)
        
        self.btn_save = QPushButton("Guardar Nube Actual (PLY)")
        self.btn_save.setMinimumHeight(35)
        self.btn_save.setStyleSheet("background-color: #E0E0E0; color: black; font-weight: bold;")
        self.btn_save.clicked.connect(self.save_ply)
        layout.addWidget(self.btn_save)
        
        # --- Timer Integración Open3D ---
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_open3d_loop)
        self.timer.start(33)

    def update_dist(self, val):
        v = val / 10.0
        gui_state['max_dist'] = v
        self.lbl_dist.setText(f"Distancia Máxima: {v:.1f} m")
        
    def update_pt(self, val):
        gui_state['point_size'] = val
        self.lbl_pt.setText(f"Tamaño de Punto (3D): {val}")
        self.vis.get_render_option().point_size = float(val)

    def update_min(self, val):
        gui_state['min_thick'] = val / 100.0
        self.lbl_min.setText(f"Espesor Mínimo (Falta - Rojo): {val} cm")

    def update_max(self, val):
        gui_state['max_thick'] = val / 100.0
        self.lbl_max.setText(f"Espesor Máximo (Exceso - Azul): {val} cm")

    def capture_baseline(self):
        gui_state['capture_baseline'] = True

    def clear_baseline(self):
        gui_state['clear_baseline'] = True
        
    def save_ply(self):
        gui_state['save_ply'] = True

    def closeEvent(self, event):
        global is_ctrl_c
        is_ctrl_c = True
        self.vis.destroy_window()
        event.accept()

    def update_open3d_loop(self):
        updated = False
        with point_cloud_lock:
            if gui_state['clear_baseline']:
                gui_state['baseline_pcd'] = None
                self.baseline_vis_pcd.points = o3d.utility.Vector3dVector(np.empty((0, 3)))
                self.baseline_vis_pcd.colors = o3d.utility.Vector3dVector(np.empty((0, 3)))
                self.update_baseline_vis = True
                gui_state['clear_baseline'] = False
                print("[OK] Línea base borrada.")
                
            if point_cloud_data is not None and color_data is not None:
                current_points = point_cloud_data.copy()
                current_colors = color_data.copy()
                
                if gui_state['capture_baseline']:
                    gui_state['baseline_pcd'] = o3d.geometry.PointCloud()
                    gui_state['baseline_pcd'].points = o3d.utility.Vector3dVector(current_points)
                    gui_state['capture_baseline'] = False
                    print(f"[OK] Línea base guardada con {len(current_points)} puntos.")
                    
                    self.baseline_vis_pcd.points = o3d.utility.Vector3dVector(current_points)
                    # AQUÍ ESTÁ EL CAMBIO: Se usa current_colors en lugar del gris plano.
                    self.baseline_vis_pcd.colors = o3d.utility.Vector3dVector(current_colors)
                    self.update_baseline_vis = True
                    
                if gui_state['baseline_pcd'] is not None:
                    heat_colors, avg_dist = calculate_thickness_colors(
                        current_points, gui_state['baseline_pcd'], 
                        gui_state['min_thick'], gui_state['max_thick']
                    )
                    self.pcd.colors = o3d.utility.Vector3dVector(heat_colors)
                    
                    # Imprimir el promedio cada 1 segundo para no saturar la terminal
                    curr_time = time.time()
                    if curr_time - self.last_print_time >= 1.0:
                        print(f"[ESPESOR] Distancia promedio del material nuevo: {avg_dist*100:.2f} cm")
                        self.last_print_time = curr_time
                else:
                    self.pcd.colors = o3d.utility.Vector3dVector(current_colors)
                    
                self.pcd.points = o3d.utility.Vector3dVector(current_points)
                updated = True
        
        if updated:
            self.vis.update_geometry(self.pcd)
            if self.first_frame and len(self.pcd.points) > 100:
                self.vis.reset_view_point(True)
                self.first_frame = False
                
        if self.update_baseline_vis:
            self.vis.update_geometry(self.baseline_vis_pcd)
            self.update_baseline_vis = False
        
        if gui_state['save_ply']:
            if len(self.pcd.points) > 0:
                fname = f"shotcrete_scan_{datetime.now().strftime('%Y%m%d_%H%M%S')}.ply"
                o3d.io.write_point_cloud(fname, self.pcd)
                print(f"[EXITO] Archivo guardado: {fname}")
            gui_state['save_ply'] = False
        
        if not self.vis.poll_events():
            self.close()
        self.vis.update_renderer()

def main():
    global is_ctrl_c
    signal.signal(signal.SIGINT, signal_handler)
    
    sdk = AuroraSDK()
    acquisition_thread = None
    
    try:
        devices = sdk.discover_devices(timeout=5.0)
        if not devices:
            return 1
        sdk.connect(device_info=devices[0])
        
        if not sdk.enhanced_imaging.is_depth_camera_supported(): 
            return 1
            
        sdk.controller.set_enhanced_imaging_subscription(ENHANCED_IMAGE_TYPE_DEPTH, True)
        time.sleep(2.0)
        
        acquisition_thread = threading.Thread(target=point_cloud_acquisition_thread, args=(sdk, 10.0), daemon=True)
        acquisition_thread.start()
        
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Visor LiDAR 3D - CORFO MVP", width=1024, height=768)
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array([[0, 0, 0]]))
        pcd.colors = o3d.utility.Vector3dVector(np.array([[1, 1, 1]]))
        vis.add_geometry(pcd)
        
        # === AÑADIR ORIGEN Y ROTAR (90 grados a la derecha) ===
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
        
        # Rotar el eje 90 grados a la DERECHA sobre el eje Y
        R = coord_frame.get_rotation_matrix_from_xyz((0, np.pi / 2, 0))
        coord_frame.rotate(R, center=(0, 0, 0))
        
        vis.add_geometry(coord_frame)
        
        vis.get_render_option().background_color = np.asarray([0.05, 0.05, 0.05])
        vis.get_render_option().show_coordinate_frame = False 
        vis.get_render_option().point_size = float(gui_state['point_size'])
        
        app = QApplication(sys.argv)
        main_win = MainWindow(vis, pcd)
        main_win.show()
        
        app.exec()
            
    except Exception:
        pass
    finally:
        is_ctrl_c = True
        if acquisition_thread and acquisition_thread.is_alive(): 
            acquisition_thread.join(timeout=2.0)
        try:
            sdk.controller.set_enhanced_imaging_subscription(ENHANCED_IMAGE_TYPE_DEPTH, False)
            sdk.disconnect()
            sdk.release()
        except: pass

if __name__ == "__main__":
    sys.exit(main())
