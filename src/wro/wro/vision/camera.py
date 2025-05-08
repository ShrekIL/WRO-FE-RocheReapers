from abc import ABC

try:
    from .camera_capture import RealSenseCapture
except:
    from camera_capture import RealSenseCapture

class DepthCamera(ABC):
    def __init__(self):
        self.frame = None

    def shutdown(self):
        pass

    def capture(self) -> RealSenseCapture:
        pass
