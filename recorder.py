# coding=utf-8
# =============================================================================
# Brennan Hoppa
# based off of SaveToAvi example from PySpin god

import PySpin # type: ignore
import sys
import numpy as np
import matplotlib.pyplot as plt # type: ignore
import cv2 # type: ignore
import time

class AviType:
    """'Enum' to select AVI video type to be created and saved"""
    UNCOMPRESSED = 0 # way more memory than mjpg - 3 sec - 186 MB
    MJPG = 1 # still avi file, much smaller - 3 sec - 2.5 MB
chosenAviType = AviType.MJPG

############
# right now this sets length of vid - 10 images per sec of the video
# not sure over how long the images are actually taken
# will mod
NUM_IMAGES = 30  # number of images to use in AVI file
# FPS / vid length is not working properly at all - fix later
VIDEO_NUMBER = "two"
THRESHOLD = 70
ALPHA = 100
############

def find_dark_spots(image, threshold):
    _,thresh = cv2.threshold(image,threshold,255,cv2.THRESH_BINARY_INV) # adjust the first value to get lighter/smaller spots - higher is more sensitive
    contours, _ = cv2.findContours(thresh,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    dark_spots = [cv2.boundingRect(contour) for contour in contours]
    return dark_spots

def group_spots(dark_spots, alpha):
    groups = []
    for x,y,w,h in dark_spots:
        added_to_group = False
        for group in groups:
            gx, gy = np.mean([x + w / 2 for x, _, w, _ in group]), np.mean([y + h / 2 for _, y, _, h in group])
            dist = np.sqrt((gx - (x + w/2))**2 + (gy - (y+h/2))**2)
            if dist <= alpha:
                group.append((x,y,w,h))
                added_to_group = True
        if not added_to_group:
            groups.append([(x,y,w,h)])
    return groups

def find_jelly_group(groups):
    # only works if we actually get all 5 and may still need to refine
    # returns list of groups rn, we just want 1
    jg = [g for g in groups if len(g) == 5]
    return jg

def save_list_to_avi(nodemap, nodemap_tldevice, images):
    """
    This function prepares, saves, and cleans up an AVI video from a vector of images.

    :param nodemap: Device nodemap.
    :param nodemap_tldevice: Transport layer device nodemap.
    :param images: List of images to save to an AVI video.
    :type nodemap: INodeMap
    :type nodemap_tldevice: INodeMap
    :type images: list of ImagePtr
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    print('*** CREATING VIDEO ***')
    print("can't currently save the video on the pi")
    return True
    try:
        result = True

        # Get the current frame rate; acquisition frame rate recorded in hertz
        #
        # *** NOTES ***
        # The video frame rate can be set to anything; however, in order to
        # have videos play in real-time, the acquisition frame rate can be
        # retrieved from the camera.

        node_acquisition_framerate = PySpin.CFloatPtr(nodemap.GetNode('AcquisitionFrameRate'))

        if not PySpin.IsReadable(node_acquisition_framerate):
            print('Unable to retrieve frame rate. Aborting...')
            return False

        framerate_to_set = node_acquisition_framerate.GetValue()

        print('*** Frame rate to be set to %d FPS ***' % framerate_to_set)

        # Select option and open AVI filetype with unique filename
        #
        # *** NOTES ***
        # Depending on the filetype, a number of settings need to be set in
        # an object called an option. An uncompressed option only needs to
        # have the video frame rate set whereas videos with MJPG or H264
        # compressions should have more values set.
        #
        # Once the desired option object is configured, open the AVI file
        # with the option in order to create the image file.
        #
        # Note that the filename does not need to be appended to the
        # name of the file. This is because the AVI recorder object takes care
        # of the file extension automatically.
        #
        # *** LATER ***
        # Once all images have been added, it is important to close the file -
        # this is similar to many other standard file streams.

        avi_recorder = PySpin.SpinVideo()

        if chosenAviType == AviType.UNCOMPRESSED:
            avi_filename = 'Uncompressed-video-%s' % VIDEO_NUMBER 
            option = PySpin.AVIOption()
            option.frameRate = framerate_to_set
            option.height = images[0].GetHeight()
            option.width = images[0].GetWidth()

        elif chosenAviType == AviType.MJPG:
            avi_filename = 'MJPG-video-%s' % VIDEO_NUMBER
            option = PySpin.MJPGOption()
            option.frameRate = framerate_to_set
            option.quality = 75
            option.height = images[0].GetHeight()
            option.width = images[0].GetWidth()

        else:
            print('Error: Unknown AviType. Aborting...')
            return False


        avi_recorder.Open(avi_filename, option)

        # Construct and save AVI video
        # *** NOTES ***
        # Although the video file has been opened, images must be individually
        # appended in order to construct the video.
        print('Appending %d images to AVI file: %s.avi...' % (len(images), avi_filename))

        for i in range(len(images)):
            avi_recorder.Append(images[i])
            # print('Appended image %d...' % i)

        # Close AVI file
        #
        # *** NOTES ***
        # Once all images have been appended, it is important to close the
        # AVI file. Notice that once an AVI file has been closed, no more
        # images can be added.

        avi_recorder.Close()
        print('Video saved at %s.avi' % avi_filename)

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return False

    return result


def print_device_info(nodemap):
    """
    This function prints the device information of the camera from the transport
    layer; please see NodeMapInfo example for more in-depth comments on printing
    device information from the nodemap.

    :param nodemap: Transport layer device nodemap.
    :type nodemap: INodeMap
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    print('\n*** DEVICE INFORMATION (uncomment if desired) ***\n')

    try:
        result = True
        node_device_information = PySpin.CCategoryPtr(nodemap.GetNode('DeviceInformation'))

        if PySpin.IsReadable(node_device_information):
            features = node_device_information.GetFeatures()
            for feature in features:
                node_feature = PySpin.CValuePtr(feature)
                # print('%s: %s' % (node_feature.GetName(),
                #                   node_feature.ToString() if PySpin.IsReadable(node_feature) else 'Node not readable'))

        else:
            print('Device control information not readable.')

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        print("Try again to run the script, or change return statement below to True and try")
        return False

    return result

########### adjustments here though --------------------
def acquire_images(cam, nodemap):
    """
    This function acquires 30 images from a device, stores them in a list, and returns the list.
    please see the Acquisition example for more in-depth comments on acquiring images.

    :param cam: Camera to acquire images from.
    :param nodemap: Device nodemap.
    :type cam: CameraPtr
    :type nodemap: INodeMap
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    print('*** IMAGE ACQUISITION ***\n')
    try:
        result = True

        # Set acquisition mode to continuous
        node_acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode('AcquisitionMode'))
        if not PySpin.IsReadable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
            print('Unable to set acquisition mode to continuous (enum retrieval). Aborting...')
            return False

        # Retrieve entry node from enumeration node
        node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')
        if not PySpin.IsReadable(node_acquisition_mode_continuous):
            print('Unable to set acquisition mode to continuous (entry retrieval). Aborting...')
            return False

        acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()

        node_acquisition_mode.SetIntValue(acquisition_mode_continuous)

        print('Acquisition mode set to continuous...')

        #  Begin acquiring images
        cam.BeginAcquisition()

        print('Acquiring images...')

        # Retrieve, convert, and save images
        images = list()

        # Create ImageProcessor instance for post processing images
        processor = PySpin.ImageProcessor()

        # Set default image processor color processing method
        #
        # *** NOTES ***
        # By default, if no specific color processing algorithm is set, the image
        # processor will default to NEAREST_NEIGHBOR method.
        processor.SetColorProcessing(PySpin.SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR)
        for i in range(NUM_IMAGES):
            try:
                #  Retrieve next received image
                image_result = cam.GetNextImage(1000)

                #  Ensure image completion
                if image_result.IsIncomplete():
                    print('Image incomplete with image status %d...' % image_result.GetImageStatus())

                else:
                    #  Convert image to mono 8 and append to list
                    images.append(processor.Convert(image_result, PySpin.PixelFormat_Mono8))



                    # **************** TRACKING *************
                    
                    image_array = image_result.GetNDArray()
                    # image = cv2.cvtColor(image_array, cv2.IMREAD_GRAYSCALE) #might have to change it to COLOR_BGR2RGB
                    # image = cv2.cvtColor(image_array, cv2.COLOR_BGR2GRAY)
                    # cv2_image = cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB)  # Convert to RGB first (OpenCV default is BGR)
                    # image = cv2.cvtColor(cv2_image, cv2.COLOR_RGB2BGR)  # Convert back to BGR for correct display
                    image = cv2.cvtColor(image_array, cv2.IMREAD_GRAYSCALE)
                    image = cv2.cvtColor(image, cv2.COLOR_RGBA2GRAY)

                    height, width = image.shape
                    center = (round(width/2),round(height/2))
                    cv2.drawMarker(image,center,(0,0,255),markerType=cv2.MARKER_CROSS)
                    
                    # below might only work w/ jelly fish pics
                    dark_spots = find_dark_spots(image,THRESHOLD)
                    groups = group_spots(dark_spots,ALPHA)
                    jgs = find_jelly_group(groups)

                    if len(jgs) == 1:
                        jg = jgs[0]
                        for idx, (x, y, w, h) in enumerate(jg):
                                # print(f"Dark spot {idx+1}: Position (x: {x}, y: {y}), Width: {w}, Height: {h}")
                                cv2.drawMarker(i,(x,y),(0,0,0),markerType=cv2.MARKER_SQUARE)
                        jx = round(np.mean([x + w/2 for x,_,w,_ in jg]))
                        jy = round(np.mean([y + h/2 for _,y,_,h in jg]))
                        cv2.drawMarker(i,(jx,jy),(0,0,0),markerType=cv2.MARKER_DIAMOND)
                        err = [g - c for g,c in zip([jx,jy],center)]
                        # *********** MOTORS *****************
                        # will have to tune this a ton for speed / duration / frame rate etc
                        if abs(err[0]) > 10:
                            if err[0] < 10:
                                # move motor L (or R) depends
                                pass
                            else:
                                # move motor R (or L) depends on mapping
                                pass
                        if abs(err[1]) > 10:
                            if err[1] < 10:
                                # move motor U (or D) depends
                                pass
                            else:
                                # move motor D (or U) depends on mapping
                                pass
                    else: 
                        print("incorrect tracking - didn't find the jelly")

                    #***Fix below for a live stream - use pyagame? *********
                    # display image feed
                    # don't actually need this to work so we are chilling just helpful for debugging
                    # cv2.imshow('Live Feed', image)
                    # cv2.waitKey(1)

                    print("updated")
                    
                    # use plt to have axis / see coords if desired
                    # plt.title('Live Feed')
                    # plt.imshow(image,cmap='gray')
                    # plt.pause(0.001)
                    # # plt.show(block=False)
                    # plt.draw()
                    # print('Updated')

                    #  Release image
                    image_result.Release()

            except PySpin.SpinnakerException as ex:
                print('Error: %s' % ex)
                result = False

        # End acquisition
        cam.EndAcquisition()

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        result = False

    return result, images
#######

def run_single_camera(cam):
    """
    This function acts as the body of the example; please see NodeMapInfo example
    for more in-depth comments on setting up cameras.

    :param cam: Camera to run example on.
    :type cam: CameraPtr
    :return: True if successful, False otherwise.
    :rtype: bool
    """

    try:
        result = True

        # Retrieve TL device nodemap and print device information
        nodemap_tldevice = cam.GetTLDeviceNodeMap()

        result &= print_device_info(nodemap_tldevice)

        # Initialize camera
        cam.Init()

        # Retrieve GenICam nodemap
        nodemap = cam.GetNodeMap()

        # Acquire list of images
        err, images = acquire_images(cam, nodemap)
        if err < 0:
            return err

        result &= save_list_to_avi(nodemap, nodemap_tldevice, images)

        # Deinitialize camera
        cam.DeInit()

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        result = False

    return result


def main():
    """
    Example entry point; please see Enumeration example for more in-depth
    comments on preparing and cleaning up the system.

    :return: True if successful, False otherwise.
    :rtype: bool
    """
    result = True

    # Retrieve singleton reference to system object
    system = PySpin.System.GetInstance()

    # Get current library version
    version = system.GetLibraryVersion()
    print('Library version: %d.%d.%d.%d' % (version.major, version.minor, version.type, version.build))

    # Retrieve list of cameras from the system
    cam_list = system.GetCameras()

    num_cameras = cam_list.GetSize()

    print('Number of cameras detected:', num_cameras)

    # Finish if there are no cameras
    if num_cameras == 0:
        # Clear camera list before releasing system
        cam_list.Clear()

        # Release system instance
        system.ReleaseInstance()

        print('Not enough cameras!')
        input('Done! Press Enter to exit...')
        return False

    # Run example on each camera
    for i, cam in enumerate(cam_list):

        print('Running example for camera %d...' % i)

        result &= run_single_camera(cam)
        print('Camera %d example complete... \n' % i)

    # Release reference to camera
    # NOTE: Unlike the C++ examples, we cannot rely on pointer objects being automatically
    # cleaned up when going out of scope.
    # The usage of del is preferred to assigning the variable to None.
    del cam

    # Clear camera list before releasing system
    cam_list.Clear()

    # Release instance
    system.ReleaseInstance()

    input('Done! Press Enter to exit...')
    return result

if __name__ == '__main__':
    if main():
        sys.exit(0)
    else:
        sys.exit(1)
