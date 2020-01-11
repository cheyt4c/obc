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

STREAM_OUTPUT = False # set True to stream all output coming from any enabled serial port

# Note: the analysis of the OBC output assumes specific string match conditions. If these
# strings do not match then the analysis will fail.
CHECK_OBC_OUTPUT = True # set True to match OBC output to connected modules

SENSOR_TAG = 'Sensor'
OBC_TAG = 'OBC'
SUCCESS_TAG = 'SUCCESS'
FAIL_TAG = 'FAILED'
TAG_LEN = max(len(SENSOR_TAG), len(OBC_TAG), len(SUCCESS_TAG), len(FAIL_TAG))

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
                except UnicodeDecodeError:# as e:
                    pass
                    #self.output_q.put( (self.label, str(e) + os.linesep))


def main():
    # create filepaths
    fn_timestamp = datetime.datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
    fp_sensor = os.path.join(LOG_DIR, '{}_sensor.txt'.format(fn_timestamp))
    fp_obc = os.path.join(LOG_DIR, '{}_obc.txt'.format(fn_timestamp))
    print("Creating files: {}, {}".format(fp_sensor, fp_obc))

    # start threads
    print_q = queue.Queue() # stores tuples (device_label, UART_line)
    t_sensor = SerialReader(SENSOR_PORT, BAUD, fp_sensor, print_q, SENSOR_TAG)
    t_obc    = SerialReader(OBC_PORT,    BAUD, fp_obc, print_q, OBC_TAG)

    time.sleep(4) # wait for Arduino bootloaders to initialise
    t_sensor.start()
    t_obc.start()
    print("Started threads.")

    try:
        realtime_analysis(print_q, TAG_LEN)
    except KeyboardInterrupt:
        # stop threads
        t_sensor.is_stopped = True
        t_obc.is_stopped = True
    print("Program ended.")


def realtime_analysis(print_q, tag_len):
    sensor_lines = queue.Queue()
    obc_crc_fails = 0
    obc_success = 0
    obc_fail = 0

    while True:
        label, line = print_q.get(print_q)
        if STREAM_OUTPUT: # print serial output without modification
            print('[{:<{tag_len}}]: {}'.format(label, line, tag_len=tag_len), end='')

        try:
            if CHECK_OBC_OUTPUT: # find matching OBC output "err, rtcTime, gpsTime, millisTime" to sensor output
                if label.lower() == SENSOR_TAG.lower():
                    # sensor output
                    sensor_lines.put(line)

                elif label.lower() == OBC_TAG.lower():
                    # obc output
                    if '[' not in line: # ignore any read partial serial lines on test startup
                        continue

                    if 'CRC failed' in line:
                        obc_crc_fails += 1
                        #print("CRC failed. (Total so far: {})".format(obc_crc_fails))
                        continue
    
                    obc_cur_dict = split_obc_line(line)
                    obc_csv = obc_cur_dict['line'].split(',')
                    if obc_cur_dict['line'].rstrip() == '0,0,0,0,0.00,0.00,0.00,0.00,0.00,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000':
                        continue

                    print("OBC testing (len: {}): {}".format(len(obc_csv), obc_cur_dict['line'].rstrip() ))

                    if obc_cur_dict['tag'].lower() == SENSOR_TAG.lower():
                        # echo of: sensor output
                        if len(obc_csv) != 18:
                            continue
                        found = False
                        
                        while not sensor_lines.empty(): # compare to sensor output
                            sensor_line = sensor_lines.get()
                            sensor_csv = sensor_line.split(',')
                            if len(sensor_csv) != 18: # expected num csv items in sensor output
                                continue
                            if obc_csv == sensor_csv:
                                found = True
                                obc_success += 1
                                print("[{:<{tag_len}}]: OBC and sensor module matched! (#success: {}, #fail: {})".format(SUCCESS_TAG, obc_success, obc_fail, tag_len=tag_len))
                                break
                            if int(sensor_csv[1]) > int(obc_csv[1]):
                                break
                        if not found:
                            obc_fail += 1
                            print("[{:<{tag_len}}]: OBC and sensor module mismatched (#success: {}, #fail: {})".format(FAIL_TAG, obc_success, obc_fail, tag_len=tag_len))
                            print("  OBC: {}  Sensor: {}".format(obc_cur_dict['line'], sensor_line))
                        print("")
            
        except Exception as e:
            print(line, e) # ignore errors from analysis failure

# LINE REFERENCE:
# [Sensor,166541]: 17,632070735,0,155292,nan,nan,25.10,25.20,0.00,1.006256,0.000000,0.000000,0.028732,-0.100562,-10.252494,0.000019,-0.000886,-0.000150
def split_obc_line(line):
    prefix, serial_line = line.split(' ')
    tag, print_time = prefix.split(',')
    tag = tag[1:]
    print_time = print_time[:-2]
    return {'tag':tag, 'print_time':print_time, 'line':serial_line}


if __name__ == '__main__':
    main()