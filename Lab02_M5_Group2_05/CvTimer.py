import cv2


class CvTimer:
    OPTIMIZED = cv2.useOptimized()
    WIDTH = 2
    PRECISION = 2

    def __init__(self):
        self._dict = dict()

    def start(self, name: str) -> None:
        """
        Args:
            name (str): Name of process to begin monitor.
        """
        # start time
        self._dict[name] = [0, 0, 0, 0]
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

    def print_summary(self) -> None:
        for process in self._dict:
            time_taken = self._dict[process][2]
            rate = self._dict[process][3]
            print(f"{process} took {time_taken:.2f}ms @ {rate:.2f}Hz\n")
