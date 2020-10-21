import numpy as np
import requests
import cv2


class PenguinPi:
    def __init__(self, ip="localhost") -> None:
        """
        Args:
            ip (str, optional): IP address. Defaults to "localhost".
        """
        self.ip = ip
        self.port = 40000

    def set_velocity(self, lvel: int, rvel: int, time: int = 0) -> None:
        """
        Args:
            lvel (int): Left wheel speed
            rvel (int): Right wheel speed
            time (int, optional): Defaults to 0.
        """
        if time == 0:
            r = requests.get(
                f"http://{self.ip}:{self.port}/robot/set/velocity?value="
                + str(lvel)
                + ","
                + str(rvel)
            )
        else:
            assert time > 0, "Time must be positive."
            assert time < 60, "Time must be less than network timeout (60s)."
            r = requests.get(
                "http://"
                + self.ip
                + ":"
                + str(self.port)
                + "/robot/set/velocity?value="
                + str(lvel)
                + ","
                + str(rvel)
                + "&time="
                + str(time)
            )

    def get_image(self) -> cv2.Frame or np.ndarray:
        try:
            r = requests.get(f"http://{self.ip}:{self.port}/camera/get")
            img = cv2.imdecode(np.frombuffer(r.content, np.uint8), cv2.IMREAD_COLOR)
        except (
            requests.exceptions.ConnectTimeout,
            requests.exceptions.ConnectionError,
            requests.exceptions.ReadTimeout,
        ) as e:
            print("Image retrieval timed out.")
            img = np.zeros((240, 320, 3), dtype=np.uint8)
        return img
