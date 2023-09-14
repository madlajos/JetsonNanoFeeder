import cv2
import time
from pypylon import pylon
import snap7
from snap7.util import *
import struct
import plcconnect
from video import Video


# User Variables
camera_image_width = 300
camera_image_height = 500
camera_framerate = 60
camera_exposure_time_ms = 300
camera_gain = 15
camera_gamma = 1

# Output Video parameters
recordVideo = True
output_video_filename = "output_video.mp4"
# FFMPEG Video Capture Settings
# TODO: if image rotation is applied later, height and width should be switched up
video = Video(output_video_filename, camera_image_height, camera_image_width, 'gray', camera_framerate)

plc_ip = "192.168.0.30"

imageIndex = 0
baseline_brightness = 0
cumulative_brightness = 0
starting_plc_mass = 0
fed_plc_mass = 0


def setupCamera():
    # Set camera parameters
    camera.AcquisitionFrameRateEnable.SetValue(True)
    camera.AcquisitionFrameRate.SetValue(camera_framerate)
    camera.ExposureTime.SetValue(camera_exposure_time_ms)
    camera.Gain.SetValue(camera_gain)
    camera.Gamma.SetValue(camera_gamma)

    #  Set ROI size; (image width - min image width) has to be divisible by min increment
    nearest_accepted_width = calculateNearestAcceptedImageSize(camera_image_width, camera.Width.GetMin(), 
                                                            camera.Width.GetMax(), camera.Width.GetInc())
    camera.Width = nearest_accepted_width
    nearest_accepted_height = calculateNearestAcceptedImageSize(camera_image_height, camera.Height.GetMin(), 
                                                            camera.Height.GetMax(), camera.Height.GetInc())
    camera.Height = nearest_accepted_height

    # Center ROI
    camera.OffsetX.SetValue((camera.SensorWidth.GetValue() // 2 - nearest_accepted_width // 2) 
                            // camera.Width.GetInc() * camera.Width.GetInc())
    camera.OffsetY.SetValue(((camera.SensorHeight.GetValue() // 2 - nearest_accepted_height // 2) 
                            // camera.Height.GetInc() * camera.Height.GetInc()) // 2 * 2) 
    
   
    #camera.PixelFormat.SetValue(pylon.PixelFormat_Mono8)
    # Start grabbing frames
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

def calculateNearestAcceptedImageSize(user_size, min_size, max_size, min_increment):
    if(user_size < min_size):
        nearest_accepted_size = min_size
    elif(user_size > max_size):
        nearest_accepted_size = max_size
    else:
        diff = user_size - min_size
        nearest_accepted_size = min_size + (diff // min_increment) * min_increment
    return nearest_accepted_size

def analyseImage(frame_mono8):
    _, frame_thresh = cv2.threshold(frame_mono8, 170, 200, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(frame_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    contours_filtered = []
    height, width = frame_thresh.shape[:2]
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area >= 1000:
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

    average_brightness = round(cv2.mean(frame_mono8)[0], 3)
    
    current_time = time.time() * 1000
    time_diff = current_time - global_prev_time


     # Display the particle count on the image
    cv2.putText(contours_image, f'Blob Count: {particle_count}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    # Display the average brightness on the image
    cv2.putText(contours_image, f'Corr. Brightness: {round((average_brightness - baseline_brightness), 3)}', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    # Display the time difference on the image
    cv2.putText(contours_image, f'Frametime: {int(time_diff)}ms', (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    # Display the cumulative brightness on the image
    cv2.putText(contours_image, f'Total Brightness: {cumulative_brightness}', (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    # Display the fed mass on the image
    cv2.putText(contours_image, f'Mass Fed (PLC): {round(fed_plc_mass, 3)}g', (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)


    cv2.imshow('Camera', contours_image)

    return average_brightness   

def connectToPLC(plc_ip):
    plc = snap7.client.Client()
    connected_to_plc = False

    try:
        plc.connect(plc_ip, 0, 1)
        connected_to_plc = True
    except Exception as e:
        print("Failed to connect to PLC:", str(e))
        plc = None
    
    return plc, connected_to_plc


# Define the initial size of the display window
window_width = 800
window_height = 800
cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Camera', window_width, window_height)


# Find and open the camera # Create an ImageFormatConverter object
camera = None
try:
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.Open()
    converter = pylon.ImageFormatConverter()
    setupCamera()
except Exception as e:
    print("Failed to open camera:", str(e))

global_prev_time = time.time() * 1000

try:
    plc, connected_to_plc = connectToPLC(plc_ip)
    if connected_to_plc:
        print("Successfully connected to the PLC!")
except Exception as e:
    print("Failed to connect to PLC:", str(e))

while camera is not None and camera.IsGrabbing():
    grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    if grab_result.GrabSucceeded():
        # Convert the frame to BGR using ImageFormatConverter
        image = pylon.PylonImage()
        image.AttachGrabResultBuffer(grab_result)

        image.PixelFormat = "Mono8"
        converted_image = converter.Convert(image)
        frame_mono8 = converted_image.GetArray()
        #frame_mono8 = cv2.rotate(frame_mono8, cv2.ROTATE_90_COUNTERCLOCKWISE)
        average_brightness = analyseImage(frame_mono8)

        # Write frame to output video
        video.write(frame_mono8)

        global_prev_time = time.time() * 1000
        
    grab_result.Release()
    imageIndex += 1

    # Look for the highest brightness for baseline and get starting mass from PLC
    if(imageIndex <= 49):
        if(baseline_brightness < average_brightness):
            baseline_brightness = average_brightness
            starting_plc_mass = plcconnect.GetFeederMass(plc)
    # Subtract baseline brightness from further images
    else:
        average_brightness -= baseline_brightness
        if(average_brightness > 0):
            cumulative_brightness += average_brightness
            cumulative_brightness = round(cumulative_brightness, 3)
    if connected_to_plc:
        plc_version = plcconnect.GetFeederVersion(plc)
        plc_mass = plcconnect.GetFeederMass(plc)
        fed_plc_mass = starting_plc_mass - plc_mass

    # Check for 'q' key press to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q') or cv2.getWindowProperty('Camera', cv2.WND_PROP_AUTOSIZE) == 1.0:
        if connected_to_plc:
            plc.disconnect()

        # if recordvideo:
        video.release()

        camera.StopGrabbing()
        camera.Close()
        cv2.destroyAllWindows()

        break
