import cv2
from subprocess import Popen, PIPE
import atexit

class Video:
    def __init__(self, videofilename, width, height, pixelformat, videofps):
        dimension = '{}x{}'.format(int(width), int(height))

        __qvstr = ['-crf', '23']  # Set the CRF (Constant Rate Factor)

        ffmpegcommand = [
            'ffmpeg',
            '-y',
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-s', dimension,
            '-pix_fmt', pixelformat,
            '-r', str(videofps),
            '-i', '-',
            '-an',
            *__qvstr,
            '-s', dimension,
            '-preset', 'ultrafast',
            '-vcodec', 'h264',
            videofilename
        ]

        try:
            self.p = Popen(ffmpegcommand, stdin=PIPE)
            atexit.register(self.release)  # Register release() to be called when the program exits
        except Exception as e:
            print(f'Video error: {e}')

    def write(self, img):
        try:
            self.p.stdin.write(img)
        except BrokenPipeError:
            print("Broken pipe error. Make sure to release the video object when done.")

    def release(self):
        try:
            if self.p.poll() is None:  # Check if the process is still running
                self.p.stdin.close()
                self.p.wait()
        except Exception as e:
            print(f'Error while releasing video: {e}')