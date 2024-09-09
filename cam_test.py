import PySpin # type:ignore

def set_acquisition_frame_rate(camera, frame_rate):
    """
    Set the acquisition frame rate of the camera.

    :param camera: Camera object from PySpin
    :param frame_rate: Desired frame rate
    """
    try:
        # Ensure the camera supports frame rate control
        if PySpin.IsAvailable(camera.AcquisitionFrameRate) and PySpin.IsWritable(camera.AcquisitionFrameRate):
            camera.AcquisitionFrameRateEnabled.SetValue(True)
            camera.AcquisitionFrameRate.SetValue(frame_rate)
            print(f"Frame rate set to: {frame_rate}")
        else:
            print("Frame rate control not available on this camera.")
    except PySpin.SpinnakerException as ex:
        print(f"Error: {ex}")

def acquire_images(camera, num_images):
    """
    Acquire images from the camera.

    :param camera: Camera object from PySpin
    :param num_images: Number of images to acquire
    """
    try:
        camera.BeginAcquisition()
        print("Acquiring images...")

        for i in range(num_images):
            image_result = camera.GetNextImage()
            if image_result.IsIncomplete():
                print(f"Image incomplete with image status {image_result.GetImageStatus()}")
            else:
                print(f"Acquired image {i}")
                # Process the image here (save, display, etc.)
            image_result.Release()

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

        # Set the frame rate
        desired_frame_rate = 5.0  # Frames per second
        set_acquisition_frame_rate(camera, desired_frame_rate)

        # Acquire images
        num_images_to_acquire = 10
        acquire_images(camera, num_images_to_acquire)

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