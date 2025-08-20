import math
import py5

# string
data = ""

# float
roll = pitch = yaw = 0.000

def setup():
    py5.size(800, 500, py5.P3D)
    py5.background(000)  

def draw():

    py5.translate(py5.width/2, py5.height/2, 0);
    py5.background(000);
    py5.text_size(22);
    py5.text("Roll: Pitch: ", 0, 0);
    # Rotate the object
    py5.rotate_x(math.radians(get_even()));
    py5.rotate_z(math.radians(get_even()));
    py5.rotate_y(math.radians(10));
    
    
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
    return 50

py5.run_sketch()
