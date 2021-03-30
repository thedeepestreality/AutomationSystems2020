import pybullet as pb
from PIL import Image as im
import io


class Camera:
    TIME_DELTA = 1 / 10
    QUALITY = 80

    default_size = {
        'width': 800,
        'height': 800
    }

    def __init__(self, size=None, height=1):
        if size is None:
            size = Camera.default_size
        self.size = size
        self.viewMatrix = pb.computeViewMatrix(
            cameraEyePosition=[1, 0, height],
            cameraTargetPosition=[1, 0, 0.1],
            cameraUpVector=[0, 1, 0])
        self.projectionMatrix = pb.computeProjectionMatrixFOV(
            fov=60.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=100)
        self.cam_image_kwargs = {
            **self.size,
            'viewMatrix': self.viewMatrix,
            'projectionMatrix': self.projectionMatrix,
            'renderer': pb.ER_TINY_RENDERER
        }

    def set_new_height(self, h):
        self.__init__(size=self.size, height=h)

    def get_frame(self):
        """
        returns RGBA array of size (x, y, 4)
        """
        return pb.getCameraImage(**self.cam_image_kwargs)[2]

    def get_frame_bytes(self):
        """
        returns frames as bytes string
        """
        rgba_img = self.get_frame()
        # some magic to get bytestring
        output = io.BytesIO()
        im.fromarray(rgba_img).convert('RGB').save(output, format='JPEG', quality=Camera.QUALITY)
        return output.getvalue()
