import math
import py5
import serial
import sys

#Test
sensor_error = 1
buffer_data = ['0.00', '0.00', '0.00']

# string
#data = ""

# float
#roll = pitch = yaw = 0.000

ser = serial.Serial(
    port='/dev/ttyACM0', 
    baudrate=19200,
    timeout=2 # Timeout in seconds for read operations
)

# Function to read data
def read_data():
    try:
        if ser.in_waiting > 0:
            data = ser.readline() # Read until newline, decode, and remove trailing whitespace
            #print(f"Received: {data}")
            return data
    except serial.SerialException as e:
        print(f"Error reading data: {e}")
    return None

def setup():
    py5.size(800, 500, py5.P3D)
    py5.background(000)  

def draw():
    global sensor_error
    global buffer_data
    py5.translate(py5.width/2, py5.height/2, 0);
    py5.background(000);
    py5.text_size(22);
    py5.text("Roll: Pitch: ", 0, 0);
    # Rotate the object
    xyz_data = get_even();
    print(f"Received: {xyz_data} ");
    if sensor_error == 0:
        py5.rotate_x(math.radians(float(xyz_data[0])));
        py5.rotate_z(math.radians(float(xyz_data[1])));
        py5.rotate_y(math.radians(float(xyz_data[2])));
        buffer_data = xyz_data
        sensor_error = 1
    else:
        py5.rotate_x(math.radians(float(buffer_data[0])));
        py5.rotate_z(math.radians(float(buffer_data[1])));
        py5.rotate_y(math.radians(float(buffer_data[2])));
           
      
    #py5.rotate_x(math.radians(get_even()));
    #py5.rotate_z(math.radians(10));
    #py5.rotate_y(math.radians(10));
    
    
    # 3D 0bject
    py5.text_size(30);  
    py5.fill(0, 76, 153);
    py5.box (386, 40, 200); # Draw box
    py5.text_size(25);
    py5.fill(255, 255, 255);
    py5.text("      SECURE APP DEV", -183, 10, 101);
    #delay(10);
    #println("ypr:\t" + angleX + "\t" + angleY); // Print the values to check whether we are getting proper values



    #py5.fill(py5.random(255), py5.random(255), py5.random(255))
    #py5.ellipse(py5.mouse_x, py5.mouse_y, 50, 50)

def get_even():
    global sensor_error
    
    axis_data = read_data()
    if axis_data != None:
       str_decoded=axis_data.decode('utf_8', errors='ignore')
       #axis_dat = str_decoded.split('/')
       axis_dat = str_decoded.strip()
       axis_dat = axis_dat.split('/')
       string_length = len(axis_dat);
       print(string_length);
      #axis_x = axis_datx[0]
       #print(f"Received: {axis_x}")
       
       try:
          if len(axis_dat) == 3:
            float(axis_dat[0])
            float(axis_dat[1])
            float(axis_dat[2])
          else: 
            sensor_error = 1
            return ['ERROR', 'ERROR', "ERROR"]
       except ValueError:
          sensor_error = 1
          return ['ERROR', 'ERROR', "ERROR"]
       else:
          if axis_dat != None:
             sensor_error = 0
             return axis_dat
          else:
              sensor_error = 1
              return ['ERROR', 'ERROR', "ERROR"]  
    sensor_error = 1
    return ['ERROR', 'ERROR', "ERROR"]     
     
    
    
    
    #float_number = float(itens[0])
    #print(f"Received: {float_number}")
    
py5.run_sketch()
