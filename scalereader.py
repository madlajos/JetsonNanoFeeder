import os
import time
import serial
import re

class ScaleReader:
    def __init__(self, serial_port, baud_rate):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.program_start_time = time.time()
        self.program_startup_time = time.strftime("%Y%m%d_%H%M%S")
        self.data_out_file = None
        self.enabled = False
        self.port = None

    def init(self, data_out_dir="Data", prefix=""):
        if not prefix.endswith("_"):
            prefix += "_"
        if not os.path.exists(data_out_dir):
            os.makedirs(data_out_dir)
        txt_out_file_name = os.path.join(data_out_dir, prefix + self.program_startup_time + ".txt")
        self.data_out_file = open(txt_out_file_name, "w")
        self.data_out_file.write("Date\tTime\tElapsedMicrosecs\tWeightFromScale [g]\n")

        # Set up serial communication
        self.port = serial.Serial(
            port=self.serial_port,
            baudrate=self.baud_rate,
            bytesize=serial.SEVENBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE
        )
        self.port.reset_input_buffer()

        self.enabled = True

    def read_scale(self):
        if not self.enabled:
            return

        try:
            raw_data = self.port.readline().decode().strip()
            weight_str = re.search(r"\d+\.\d+", raw_data)
            if weight_str:
                weight_g = float(weight_str.group())
                print(f"Weight from scale: {weight_g:.4f} g")
                self.log_data(weight_g)
            else:
                print(f"Could not find weight in the raw data: {raw_data}")

        except ValueError as e:
            print(f"Error while reading scale: {e}")

    def log_data(self, weight_g):
        if self.data_out_file:
            current_time = time.strftime("%Y-%m-%d\t%H:%M:%S")
            elapsed_microsecs = int((time.time() - self.program_start_time) * 1e6)
            log_line = f"{current_time}\t{elapsed_microsecs}\t{weight_g:.4f}\n"
            self.data_out_file.write(log_line)

    def stop(self):
        self.enabled = False
        if self.port and self.port.is_open:
            self.port.close()
        if self.data_out_file:
            self.data_out_file.close()


if __name__ == "__main__":
    # Replace '/dev/ttyUSB0' with the correct serial port for your scale
    scale_reader = ScaleReader(serial_port='/dev/ttyUSB0', baud_rate=9600)

    try:
        scale_reader.init()

        while True:
            scale_reader.read_scale()

    except KeyboardInterrupt:
        # Close the serial port on Ctrl+C
        scale_reader.stop()