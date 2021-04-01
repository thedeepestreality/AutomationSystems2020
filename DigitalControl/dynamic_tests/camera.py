import pybullet as pb

class Camera:
    TIME_DELTA = 1 / 10
    QUALITY = 80

    default_size = {
        'width': 1000,
        'height': 1000
    }

    def __init__(self, size=None, height=0.5, fov = 60):
        if size is None:
            size = Camera.default_size
        self.size = size
        self.viewMatrix = pb.computeViewMatrix(
            cameraEyePosition=[0, 0, height],
            cameraTargetPosition=[0, 0, 0],
            cameraUpVector=[0, 1, 0])
        self.projectionMatrix = pb.computeProjectionMatrixFOV(
            fov=fov,
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
