import cv2
import numpy as np


class CvTimer:
    # OPTIMIZED = cv2.setUseOptimized(True)
    WIDTH = 2
    PRECISION = 2

    def __init__(self):
        self._dict = dict()

    def start(self, name: str) -> None:
        """
        Args:
            name (str): Name of process to begin monitor.
        """
        # create entry
        self._dict[name] = [0, 0, 0, 0]
        # start time
        self._dict[name][0] = cv2.getTickCount()

    def stop(self, name: str) -> None:
        """
        Args:
            name (str): Name of process to stop monitoring.
        """
        # end time
        self._dict[name][1] = cv2.getTickCount()
        # time taken (ms)
        self._dict[name][2] = (
            1000 * (self._dict[name][1] - self._dict[name][0]) / cv2.getTickFrequency()
        )
        # rate (Hz)
        self._dict[name][3] = 1000 / self._dict[name][2]

    def get_diagnostics(self, name: str) -> list:
        """
        Args:
            name (str): Name of process to retrieve time taken (ms)
        Returns:
            list: [time taken(ms), rate(Hz)]
        """
        return self._dict[name][2:]

    def print_process(self, proc_name: str) -> None:
        """
        Args:
            proc_name ([type]): Print specific process info
        """
        time_taken, rate = self._dict[proc_name][2:]
        print(f"{proc_name} took {time_taken:.1f}ms @ {rate:.1f}Hz")

    def print_all(self) -> None:
        for process in self._dict:
            self.print_process(process)
        print("")
