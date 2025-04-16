import pyrealsense2 as rs
import numpy as np
import threading

class Sensing():
    def __init__(self):
        # Configure Intel RealSense D435i pipeline
        self.camera_pipeline = rs.pipeline()
        self.camera_config = rs.config()

        # Camera identification
        self.camera_serial = None
        self._identify_camera()

        # Enable streams for camera
        self.camera_config.enable_device(self.camera_serial)
        self.camera_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.camera_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Data storage
        self.camera_data = {
            'camera_rgb': None,
            'camera_depth': None
        }

        # Thread management
        self.lock = threading.Lock()
        self.threads = {}
        self.running = True

    def _identify_camera(self):
        cameras = [{'name': device.get_info(rs.camera_info.name), 'serial_number': device.get_info(rs.camera_info.serial_number)}
        for device in rs.context().devices
        ]

        for info in cameras:
            if 'D435' in info['name']:
                self.camera_serial = info['serial_number']
                break

    def _fetch_camera_rgb(self):
        while self.running:
            frames = self.camera_pipeline.wait_for_frames()
            rgb_frame = frames.get_color_frame()
            if rgb_frame:
                rgb_data = np.asanyarray(rgb_frame.get_data())
                with self.lock:
                    self.camera_data['camera_rgb'] = rgb_data
    
    def _fetch_camera_depth(self):
        while self.running:
            frames = self.camera_pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                depth_data = np.asanyarray(depth_frame.get_data())
                with self.lock:
                    self.camera_data['camera_depth'] = depth_data

    def start_pipeline(self):
        self.camera_pipeline.start(self.camera_config)

    def stop_pipeline(self):
        self.camera_pipeline.stop()

    def start_camera_rgb(self):
        if "camera_rgb" not in self.threads:
            self.running = True
            self.threads["camera_rgb"] = threading.Thread(target=self._fetch_camera_rgb)
            self.threads["camera_rgb"].start()

    def stop_camera_rgb(self):
        if "camera_rgb" in self.threads:
            self.running = False
            self.threads["camera_rgb"].join()
            del self.threads["camera_rgb"]

    def start_camera_depth(self):
        if "camera_depth" not in self.threads:
            self.running = True
            self.threads["camera_depth"] = threading.Thread(target=self._fetch_camera_depth)
            self.threads["camera_depth"].start()

    def stop_camera_depth(self):
        if "camera_depth" in self.threads:
            self.running = False
            self.threads["camera_depth"].join()
            del self.threads["camera_depth"]

    def start_camera_both(self):
        self.start_camera_rgb()
        self.start_camera_depth()

    def stop_camera_both(self):
        self.stop_camera_rgb()
        self.stop_camera_depth()

    def get_camera_data(self, camera):
        '''
        Gets the latest data from the specified camera.

        :param camera: The camera to get data from (either 'camera_rgb' or 'camera_depth').
        :return: The latest data from the camera.
        '''
        with self.lock:
            return self.camera_data.get(camera, None)
        