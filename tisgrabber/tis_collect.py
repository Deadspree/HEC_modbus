from pathlib import Path
import os
import ctypes 
import tisgrabber as tis
import cv2
import numpy as np
# --- Configuration ---
PROJECT_ROOT = Path(__file__).resolve().parent.parent

SAVE_DIR = PROJECT_ROOT / "input"

IMAGE_PREFIX = "calib"
IMG_EXT = ".jpg"
ic = ctypes.cdll.LoadLibrary("./tisgrabber_x64.dll")
tis.declareFunctions(ic)

ic.IC_InitLibrary(0)

hGrabber = tis.openDevice(ic)

if ic.IC_IsDevValid(hGrabber):

    ic.IC_StartLive(hGrabber, 0)
    img_counter = 0
    while True:

        if ic.IC_SnapImage(hGrabber, 2000) == tis.IC_SUCCESS:

            Width = ctypes.c_long()
            Height = ctypes.c_long()
            BitsPerPixel = ctypes.c_int()
            colorformat = ctypes.c_int()

            ic.IC_GetImageDescription(hGrabber, Width, Height,
                                      BitsPerPixel, colorformat)

            bpp = BitsPerPixel.value // 8
            buffer_size = Width.value * Height.value * bpp

            imagePtr = ic.IC_GetImagePtr(hGrabber)

            imagedata = ctypes.cast(
                imagePtr,
                ctypes.POINTER(ctypes.c_ubyte * buffer_size)
            )

            image = np.ndarray(buffer=imagedata.contents,
                               dtype=np.uint8,
                               shape=(Height.value,
                                      Width.value,
                                      bpp))
            image = np.ascontiguousarray(image)
            image = cv2.flip(image, 0)
            # Convert RGB to BGR for OpenCV
            """
            if bpp == 3:
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            elif bpp == 4:
                image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            """
            cv2.namedWindow("Window", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Window", 960, 540) # just for visulization, not output image resolution
            cv2.imshow("Window", image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                # Save image
                img_name = f"{IMAGE_PREFIX}_{img_counter:03d}{IMG_EXT}"
                save_path = os.path.join(SAVE_DIR, img_name)
                cv2.imwrite(save_path, image)
                print(f"💾 Saved {save_path}")
                img_counter += 1
            if key == ord('q'):
                break

        else:
            print("No frame received in 2 seconds.")

    ic.IC_StopLive(hGrabber)
    cv2.destroyAllWindows()

else:
    ic.IC_MsgBox(tis.T("No device opened"), tis.T("Simple Live Video"))

ic.IC_ReleaseGrabber(hGrabber)

