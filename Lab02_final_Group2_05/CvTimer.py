import cv2
import time

# TODO Use time.perf_counter() instead
class CvTimer:
    # OPTIMIZED = cv2.setUseOptimized(True)
    WIDTH = 2
    PRECISION = 2

    def __init__(self):
        self._dict = dict()

    def start(self, name: str) -> None:
        """
        :param name:  Name of process to start monitoring
        :type name: str
        """
        # create entry
        self._dict[name] = [cv2.getTickCount(), 0, 0]
        # self._dict[name] = [time.perf_counter(), 0, 0]

    def stop(self, name: str) -> None:
        """
        :param name: Name of process to stop monitoring
        :type name: str
        """
        # time taken (seconds)
        self._dict[name][1] = cv2.getTickCount() - self._dict[name][0]
        # self._dict[name][1] = time.perf_counter() - self._dict[name][0]
        # rate (Hz)
        self._dict[name][2] = self._dict[1] / cv2.getTickFrequency()
        # self._dict[name][2] = 1 / self.dict[1]

    def get_diagnostics(self, name: str) -> list:
        """
        :param name: Name of process to retrieve time taken  and rate
        :type name: str
        :return: [time taken(s), rate(Hz)]
        :rtype: list
        """
        return self._dict[name][1:]

    def print_process(self, proc_name: str) -> None:
        """
        :param proc_name: Print specific process info
        :type proc_name: str
        """
        time_taken, rate = self.get_diagnostics(proc_name)
        print(f"{proc_name} took {1e3*time_taken:.2f}ms @ {rate:.2f}Hz")

    def print_all(self) -> None:
        for process in self._dict:
            self.print_process(process)
        print("")
