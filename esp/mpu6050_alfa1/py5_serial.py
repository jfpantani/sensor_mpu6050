import math
import py5
import serial
import time
import sys
import os
import stat

# Serial port
serialPort = '/dev/ttyUSB0'

#Test
sensor_error = 1
sensor_disconnected = 1
buffer_data = ['0.00', '0.00', '0.00']

# This Function verify if device exists.
def device_exists(path):
    """
    Checks if a given path corresponds to a block or character device.
    """
    try:
        mode = os.stat(path).st_mode;
        return stat.S_ISBLK(mode) or stat.S_ISCHR(mode);
    except OSError:
        return False;
    
# This funcition initialize device. 
if device_exists(serialPort):
    try:
        ser = serial.Serial(
        port= serialPort, 
        baudrate = 115200,
        timeout = None # Timeout in seconds for read operations
        );
    except serial.SerialException as e:
        print(f"Error open serial port: {e}");
        sys.exit(2);
    finally:
        print(f"Serial Port Successfully open.");
        sensor_disconnected = 0;
        time.sleep(2);
else:
    print(f"{serialPort} does not exist or is not a device.");
    sys.exit(1);
         
# Function to read data from device
def read_data():
    global sensor_disconnected;

    if (device_exists(serialPort) and (ser.is_open == True)):
        try:
            if ser.in_waiting > 0:
                data = ser.readline(); # Read until newline, decode, and remove trailing whitespace
                return data;
        except serial.SerialException as e:
            print(f"Error reading data: {e}");
    else:
        serError = True;

        while(serError):
            if device_exists(serialPort):
                serError = False
                print("Sensor RECONNECTED");
                time.sleep(1);
                if(ser.is_open == False):
                    print("Serial is close, open it!");
                    ser.open();
                sensor_disconnected = 0
                time.sleep(2);
                #return None
            else:
                print("Sensor DISCONNECTED!");
                time.sleep(1);
                serError = True;
                if(ser.is_open != False):
                    print("Serial is open, close it");
                    ser.close();
                    sensor_disconnected = 1;
                time.sleep(2);
                #return None

# Process data read from device
def get_even():
    global sensor_error;
    
    axis_data = read_data();
    if axis_data != None:
       str_decoded=axis_data.decode('utf_8', errors='ignore');
       axis_dat = str_decoded.strip();
       index = axis_dat.find(':');
       result = axis_dat[(index + 3):];
       axis_dat = result.strip();
       axis_dat = axis_dat.split('/');
       try:
          if len(axis_dat) == 3:
            float(axis_dat[0]);
            float(axis_dat[1]);
            float(axis_dat[2]);
          else: 
            sensor_error = 1;
            return ['ERROR1', 'ERROR1', "ERROR1"];
       except ValueError:
          sensor_error = 1;
          return ['ERROR2', 'ERROR2', "ERROR2"];
       else:
          if axis_dat != None:
             sensor_error = 0;
             return axis_dat;
          else:
              sensor_error = 1;
              return ['ERROR3', 'ERROR3', "ERROR3"];
    sensor_error = 1;
    return ['ERROR4', 'ERROR4', "ERROR4"];  
             
# setup py5
def setup():
    py5.size(800, 500, py5.P3D);
    py5.background(000);

# Run draw loop
def draw():
    global sensor_error;
    global buffer_data;
    global sensor_disconnected;

    py5.translate(py5.width/2, py5.height/2, 0);
    py5.background(000);
    py5.text_size(22);
    
    xyz_data = get_even();
    
    # Rotate the object
    if sensor_error == 0:
        print(f"Received: {xyz_data} ");
        py5.text(" Roll: " + str(xyz_data[0]) , -400, -200);
        py5.text(" Pitch: " + str(xyz_data[1]), -400, -180);
        py5.text(" Yaw: " + str(xyz_data[2]), -400, -162);
        py5.rotate_x(math.radians(float(xyz_data[0])));
        py5.rotate_z(math.radians(float(xyz_data[1])));
        py5.rotate_y(math.radians(float(xyz_data[2])));
        buffer_data = xyz_data;
        sensor_error = 1;
    else:
        print(f"Received: {buffer_data} ");
        py5.text(" Roll: " + str(buffer_data[0]) , -400, -200);
        py5.text(" Pitch: " + str(buffer_data[1]), -400, -180);
        py5.text(" Yaw: " + str(buffer_data[2]), -400, -162);
        py5.rotate_x(math.radians(float(buffer_data[0])));
        py5.rotate_z(math.radians(float(buffer_data[1])));
        py5.rotate_y(math.radians(float(buffer_data[2])));
    
    # 3D 0bject
    py5.text_size(30); 
    py5.fill(0, 76, 153);
    py5.box (386, 40, 200); # Draw box
    py5.text_size(25);
    py5.fill(255, 255, 255);
    py5.text("******* ESP32 MPU6050 TEST *******", -183, 10, 101);

# Run Sketch
py5.run_sketch();
