import cv2
import numpy as np
import PIL.Image


def patch_pillow_exif_tags():
    if hasattr(PIL.Image, "ExifTags") and hasattr(PIL.Image.ExifTags, "Base"):
        return

    class _ExifBase:
        Orientation = 274

    class _ExifTags:
        Base = _ExifBase

    PIL.Image.ExifTags = _ExifTags


patch_pillow_exif_tags()

from lerobot.datasets.lerobot_dataset import LeRobotDataset


OUTPUT_VIDEO = "dataset_replay.mp4"
FPS = 30.0


dataset = LeRobotDataset(
    repo_id="local/piper_teleoperation",
    root="/home/yumin/lerobot_datasets/piper_teleoperation_3cam",
)


def to_bgr(img):
    if hasattr(img, "numpy"):
        img = img.numpy()
    img = np.asarray(img)

    if img.ndim == 3 and img.shape[0] in (1, 3):
        img = np.transpose(img, (1, 2, 0))

    if img.dtype != np.uint8:
        img = np.clip(img * 255.0, 0, 255).astype(np.uint8)

    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


def opencv_has_gui():
    build_info = cv2.getBuildInformation()
    for line in build_info.splitlines():
        if line.strip().startswith("GUI:"):
            return "NONE" not in line
    return True


use_gui = opencv_has_gui()
writer = None

for i in range(len(dataset)):
    sample = dataset[i]

    main = to_bgr(sample["observation.images.main"])
    left = to_bgr(sample["observation.images.left_wrist"])
    right = to_bgr(sample["observation.images.right_wrist"])

    h, w = main.shape[:2]
    left = cv2.resize(left, (w, h))
    right = cv2.resize(right, (w, h))

    top = np.zeros((h, w * 2, 3), dtype=np.uint8)
    top[:, w // 2 : w // 2 + w] = main
    bottom = np.hstack([left, right])
    view = np.vstack([top, bottom])

    if use_gui:
        cv2.imshow("LeRobotDataset replay", view)

        key = cv2.waitKey(33) & 0xFF
        if key in (27, ord("q")):
            break
    else:
        if writer is None:
            height, width = view.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(OUTPUT_VIDEO, fourcc, FPS, (width, height))
            if not writer.isOpened():
                raise RuntimeError("Failed to open video writer: %s" % OUTPUT_VIDEO)
            print("OpenCV GUI is unavailable. Saving replay to %s" % OUTPUT_VIDEO)
        writer.write(view)

if writer is not None:
    writer.release()
    print("Saved %s" % OUTPUT_VIDEO)

if use_gui:
    cv2.destroyAllWindows()
