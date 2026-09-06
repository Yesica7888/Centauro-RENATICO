# views/view.py
from controller import Camera
import cv2
import numpy as np

class View:
    def __init__(self, robot, timestep):
        # Inicializa la cámara del robot
        self.camera = robot.getDevice("Camera")
        self.camera.enable(timestep)
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()

    def get_frame(self):
        """Devuelve la imagen actual como frame de OpenCV."""
        image = self.camera.getImageArray()
        if image is None:
            return None
        np_image = np.array(image, dtype=np.uint8)
        # Webots entrega RGB, OpenCV usa BGR
        frame = cv2.cvtColor(np_image, cv2.COLOR_RGB2BGR)
        return frame
