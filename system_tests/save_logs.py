#!/usr/bin/env python3

# Opens a serial connection to the sensor module and OBC subsystems.
# Logs their serial output contents.
#
# python -m pip install pyserial
#
# Note: Arduino uses "\r\n" as the append value for Serial.println()
# but this may be inconsistent with the use of only "\n" in some of the
# Arduino code. This program will remove any lingering "\r" automatically.
# https://www.arduino.cc/reference/en/language/functions/communication/serial/println/

import serial
import threading
import queue
import datetime
import os
import time

# Modify these parameters for your setup.
BAUD = 9600
SENSOR_PORT = 'COM11'
OBC_PORT = 'COM12'
LOG_DIR = 'terminal_logs'

class SerialReader(threading.Thread):
    def __init__(self, port, baud, filepath, output_q, label):
        super().__init__()
        self.ser = serial.Serial(port, baud)
        self.filepath = filepath
        self.output_q = output_q
        self.label = label
        self.is_stopped = False

    def run(self):
        with open(self.filepath, 'w') as fp:
            while not self.is_stopped:
                line = self.ser.readline()
                try:
                    line = line.decode('utf-8').replace('\r','')
                    fp.write(line)
                    self.output_q.put( (self.label, line) )
                except UnicodeDecodeError as e:
                    self.output_q.put( (self.label, str(e) + os.linesep))


def main():
    # create filepaths
    fn_timestamp = datetime.datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
    fp_sensor = os.path.join(LOG_DIR, '{}_sensor.txt'.format(fn_timestamp))
    fp_obc = os.path.join(LOG_DIR, '{}_obc.txt'.format(fn_timestamp))
    print("Creating files: {}, {}".format(fp_sensor, fp_obc))

    # start threads
    print_q = queue.Queue() # stores tuples (device_label, UART_line)
    t_sensor = SerialReader(SENSOR_PORT, BAUD, fp_sensor, print_q, 'Sensor')
    t_obc    = SerialReader(OBC_PORT,    BAUD, fp_obc, print_q, 'OBC')
    tag_len = len('Sensor')

    #time.sleep(3) # wait for Arduino bootloaders to initialise
    t_sensor.start()
    t_obc.start()
    print("Started threads.")

    try:
        while True:
            label, line = print_q.get()
            print('[{:<{tag_len}}]: {}'.format(label, line, tag_len=tag_len), end='')
    except KeyboardInterrupt:
        # stop threads
        t_sensor.is_stopped = True
        t_obc.is_stopped = True
    print("Program ended.")


if __name__ == '__main__':
    main()