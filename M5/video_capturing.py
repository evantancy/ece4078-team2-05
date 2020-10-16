import cv2
from KeyboardController import Keyboard
from PenguinPiC import PenguinPi

if __name__ == "__main__":
    # Get image from webcam
    ppi = PenguinPi()
    keyboard_control = Keyboard(ppi)

    while True:
        frame = ppi.get_image()
            # Scale to 144p
        scale = 1.25
        width = int(scale * frame.shape[1])
        height = int(scale * frame.shape[0])
        # width = 480
        # height = width
        resized_frame = cv2.resize(frame, (width, height), interpolation = cv2.INTER_AREA)
        cv2.namedWindow('PenguinPi Stream', cv2.WINDOW_AUTOSIZE);
        cv2.imshow('PenguinPi Stream', resized_frame)
        if cv2.waitKey(1) == ord('q'):
            break