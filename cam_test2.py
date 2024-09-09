import PySpin # type:ignore
import time

def acquire_images_with_delay(camera, num_images, delay):
    """
    Acquire images from the camera with a delay between captures.

    :param camera: Camera object from PySpin
    :param num_images: Number of images to acquire
    :param delay: Delay in seconds between captures
    """
    start = time.time()
    try:
        camera.BeginAcquisition()
        print("Acquiring images...")

        for i in range(num_images):
            image_result = camera.GetNextImage()
            if image_result.IsIncomplete():
                print(f"Image incomplete with image status {image_result.GetImageStatus()}")
            else:
                # print(f"Acquired image {i}")
                print("image acquired: ", time.time()-start)
                # Process the image here (save, display, etc.)
                image_result.Release()
            time.sleep(delay)  # Delay between captures

        camera.EndAcquisition()
    except PySpin.SpinnakerException as ex:
        print(f"Error: {ex}")

def main():
    system = PySpin.System.GetInstance()

    # Get the camera list
    cam_list = system.GetCameras()

    if cam_list.GetSize() == 0:
        print("No cameras detected.")
        cam_list.Clear()
        system.ReleaseInstance()
        return

    camera = cam_list[0]

    try:
        camera.Init()

        # Acquire images with a delay
        num_images_to_acquire = 10
        delay_between_captures = 0.01  # Delay in seconds
        acquire_images_with_delay(camera, num_images_to_acquire, delay_between_captures)

        camera.DeInit()
    except PySpin.SpinnakerException as ex:
        print(f"Error: {ex}")
    finally:
        # Release the camera
        del camera
        cam_list.Clear()
        system.ReleaseInstance()

if __name__ == "__main__":
    main()