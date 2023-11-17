import cv2
import time
from pypylon import pylon
import snap7
from snap7.util import *
import struct
import plccomm
from video import Video
from enum import Enum
import numpy as np
import math

####### User Variables ######
### Input Video Source Settings ###
use_camera = True
input_video_filename = "input_video.mp4" 

### Camera Settings ###
camera_image_width = 600
camera_image_height = 600
camera_framerate = 60
camera_exposure_time_ms = 200
camera_gain = 10
camera_gamma = 1
camera_centerROI_x = False
camera_centerROI_y = True
camera_offsetROI_x = 1003
camera_offsetROI_y = 600

### Video Display Settings ###
use_video_display = True
window_width = 800
window_height = 800

### Output Video Settings ###
record_video = False
output_video_filename = "output_video.mp4"
# FFMPEG Video Capture Settings
# TODO: if image rotation is applied later, height and width should be switched up
video = Video(output_video_filename, camera_image_width, camera_image_height, 'gray', camera_framerate)

### PLC Settings ###
use_PLC = False
plc_ip = "192.168.0.30"

### Image analysis settings ###
min_blob_area = 1000
threshold_low = 60
threshold_high = 240

imageIndex = 0
prev_plc_commbyte = 0b00000000
cumulative_brightness = 0
connected_to_plc = False
cummulative_volume = 0
feeder_state = 1
plc_data_frequency = 1000


def setupCamera():
    # Set camera parameters
    camera.AcquisitionFrameRateEnable.SetValue(True)
    camera.AcquisitionFrameRate.SetValue(camera_framerate)
    camera.ExposureTime.SetValue(camera_exposure_time_ms)
    camera.Gain.SetValue(camera_gain)
    camera.Gamma.SetValue(camera_gamma)

    #  Set ROI size; (image width - min image width) has to be divisible by min increment
    nearest_accepted_width = nearestAcceptedImgSize(camera_image_width, camera.Width.GetMin(), 
                                                            camera.Width.GetMax(), camera.Width.GetInc())
    camera.Width = nearest_accepted_width
    nearest_accepted_height = nearestAcceptedImgSize(camera_image_height, camera.Height.GetMin(), 
                                                            camera.Height.GetMax(), camera.Height.GetInc())
    camera.Height = nearest_accepted_height

    # Set ROI Offset; Either Center or by Fix value
    if camera_centerROI_x:
        camera.OffsetX.SetValue((camera.SensorWidth.GetValue() // 2 - nearest_accepted_width // 2) 
                                // camera.Width.GetInc() * camera.Width.GetInc())
    elif nearest_accepted_width + camera_offsetROI_x <= camera.Width.GetMax():
        camera.OffsetX.SetValue(camera_offsetROI_x // camera.Width.GetInc() * camera.Width.GetInc())
    else:
        camera.OffsetX.SetValue(camera.Width.GetMax() - nearest_accepted_width)

    if camera_centerROI_y:
        camera.OffsetY.SetValue(((camera.SensorHeight.GetValue() // 2 - nearest_accepted_height // 2) 
                                // camera.Height.GetInc() * camera.Height.GetInc()) // 2 * 2) 
    elif nearest_accepted_height + camera_offsetROI_y <= camera.Height.GetMax():
        camera.OffsetY.SetValue(camera_offsetROI_y // camera.Height.GetInc() * camera.Height.GetInc())
    else:
        camera.OffsetY.SetValue(camera.Height.GetMax() - nearest_accepted_height)
    
    # Start image aquisition
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

def nearestAcceptedImgSize(user_size, min_size, max_size, min_increment):
    if(user_size < min_size):
        nearest_accepted_size = min_size
    elif(user_size > max_size):
        nearest_accepted_size = max_size
    else:
        diff = user_size - min_size
        nearest_accepted_size = min_size + diff // min_increment * min_increment
    return nearest_accepted_size

def getCameraImage():
    grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
    if grab_result.GrabSucceeded():
        image = pylon.PylonImage()
        image.AttachGrabResultBuffer(grab_result)
        image.PixelFormat = "Mono8"
        converted_image = converter.Convert(image)
        frame = converted_image.GetArray()
    grab_result.Release()

    return frame

def analyseImage(frame_mono8):
    contours_filtered = []
    frame_blob_volume = 0
    
    _, frame_thresh = cv2.threshold(frame_mono8, threshold_low, threshold_high, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(frame_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    height, width = frame_thresh.shape[:2]
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area >= min_blob_area:
            contours_filtered.append(contour)
            
    # Exclude particles touching the edge of the image. Slows down analysis.
    #    for point in contour:
    #        x, y = point[0]
    #        if x == 0 or y == 0 or x == width - 1 or y == height - 1:
    #            break
    #    else:
    #        contours_filtered.append(contour)

    contours_image = cv2.cvtColor(frame_mono8, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(contours_image, contours, -1, (0, 255, 0), 2)
    particle_count = len(contours_filtered)
    
    average_brightness = round(cv2.mean(frame_thresh)[0], 3)

    #TODO: Volume-based mass calculation.
    # Working principle:
    # 1. Get the particle size (by user setting)
    # 2. Summarize particle volume
    # 3. Convert to mass
 
    for contour in contours_filtered:
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        width = rect[1][0]
        height = rect[1][1]
        frame_blob_volume += pow((width + height)/ 2,3) * math.pi / 6 / pow(10, 6)
        # Draw the contours and the fitted rectangle
        cv2.drawContours(contours_image, [box], 0, (0, 255, 0), 2)
        cv2.drawContours(contours_image, [contour], 0, (0, 0, 255), 2)
    
    current_time = time.time() * 1000
    time_diff = current_time - global_prev_time

    if use_video_display:
        cv2.putText(contours_image, f'Blob Count: {particle_count}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(contours_image, f'Brightness: {round((average_brightness), 3)}', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(contours_image, f'Blob Volume: {round((frame_blob_volume), 3)}', (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(contours_image, f'Cumm Blob Volume: {round((cummulative_volume), 3)}', (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(contours_image, f'Frametime: {int(time_diff)}ms', (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow('Camera', contours_image)

    return frame_blob_volume


# Find and open the camera
# Create an ImageFormatConverter object
camera = None
if use_camera:
    try:
        camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        #TODO:Add Failed to open camera message
        camera.Open()
        converter = pylon.ImageFormatConverter()
        setupCamera()
    except Exception as e:
        print("Failed to open camera:", str(e))

# Connect to the PLC
if use_PLC:
    try:
        plc, connected_to_plc = plccomm.connectToPLC(plc_ip)
        if connected_to_plc:
            print("Successfully connected to the PLC!")
    except Exception as e:
        print("Failed to connect to PLC:", str(e))

# Create a VideoCapture object for reading from offline video
if not use_camera:
    cap = cv2.VideoCapture(input_video_filename)
    if not cap.isOpened():
        print("Error: Cannot open input video file.")
    else:
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

# Create the Live View Window
if use_video_display:
    cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Camera', window_width, window_height)

# Initial time for speed calculation
global_prev_time = time.time() * 1000
last_data_sent_time = global_prev_time

while ((camera is not None and camera.IsGrabbing()) or (not use_camera and ret)):
    if connected_to_plc:
        feeder_state, plc_commbyte, plc_data_frequency = plccomm.PollPLC(plc)
        #TODO: plc_commbyte Handling

        prev_plc_commbyte = plc_commbyte



    if use_camera and feeder_state == 1:
        frame = getCameraImage()
        if record_video:
            video.write(frame)

        #frame_mono8 = cv2.rotate(frame_mono8, cv2.ROTATE_90_COUNTERCLOCKWISE)
        volume_from_frame = analyseImage(frame)
    
    # Offline video analysis for testing
    if not use_camera:
        ret, frame = cap.read()
        if not ret:
            # End of video file, break the loop
            break

        # Convert the frame to grayscale if necessary
        if frame.shape[2] == 3:  # Check if the frame is in color
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        volume_from_frame = analyseImage(frame)
    
    imageIndex += 1
    cummulative_volume += volume_from_frame
    global_prev_time = time.time() * 1000
    
    if (global_prev_time - last_data_sent_time) > plc_data_frequency:
        if connected_to_plc:
            plccomm.SendBlobVolume(plc, cummulative_volume)
        last_data_sent_time = global_prev_time
        cummulative_volume = 0
    
    # Check for 'q' key press to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q') or cv2.getWindowProperty('Camera', cv2.WND_PROP_AUTOSIZE) == 1.0:
        if connected_to_plc:
            plc.disconnect()

        if record_video:
            video.release()

        if use_camera:
            camera.StopGrabbing()
            camera.Close()
            cv2.destroyAllWindows()

        break
