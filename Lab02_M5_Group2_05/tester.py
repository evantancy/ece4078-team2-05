import cv2
from PenguinPiC import PenguinPi
import time
from CvTimer import CvTimer

from numba.types import DictType, ListType
import numba
import numba.experimental.jitclass as jitclass
from numba import cuda

print(cv2.useOptimized())


if __name__ == "__main__":
    ppi = PenguinPi()
    Timer = CvTimer()
    FONT = cv2.FONT_HERSHEY_SIMPLEX

    @cuda.jit
    def main_loop():
        Timer.start("main")
        Timer.start("img_proc")
        Timer.start("draw")
        frame = ppi.get_image()

        Timer.stop("img_proc")

        elapsed_time, FPS = Timer.get_diagnostics("img_proc")
        label_1 = "%s:%4.2f" % ("FPS", FPS)
        cv2.imshow("Test", frame)
        cv2.putText(
            frame,
            label_1,
            (0, 20),
            FONT,
            1,
            (255, 255, 255),
            1,
        )
        Timer.stop("draw")
        draw = Timer.get_diagnostics("draw")[1]
        label_2 = "%s:%4.2f" % ("TPS", draw)
        cv2.putText(
            frame,
            label_2,
            (0, 40),
            FONT,
            1,
            (255, 255, 255),
            1,
        )
        cv2.imshow("Test", frame)

        Timer.stop("main")
        # Timer.print_summary()
        Timer.print_process("main")

    while True:
        main_loop()
        key = cv2.waitKey(1)
        if key == 27 or key == ord("q"):
            break
    cv2.destroyAllWindows()
