
"""my_controller_zone_1 controller """

from controller import Robot, GPS, Compass, Motor, Lidar
import math 
import numpy as np 
import matplotlib.pyplot as plt
import sys

# Create the Robot instance
robot = Robot()
timestep = int(robot.getBasicTimeStep())
# Constants
MAX_SPEED = 6.28  # rad/s (e-puck max speed)
ACCELERATION = 0.1  # speed increase per timestep when holding key
TURN_RATIO = 0.7  # ratio of turning speed compared to forward speed
DISPLAY_SCALE = 50  # pixels per meter for lidar visualization
dt = timestep / 1000.0


map_points = np.zeros((0, 2))   # Holds all map points across time
robot_path = np.zeros((0, 2))   # Holds robot path
plt.ion()  # For live plotting

def mapping(gps_values, current_heading, ranges, fov):
    global map_points, robot_path   # Use global so arrays are updated across timesteps

    x, y, _ = gps_values
    total_beams = len(ranges)
    angles = np.linspace(fov / 2, -fov / 2, total_beams)
    z_points = np.zeros((0, 2))  # This scan's points

    for i, angle in enumerate(angles):
        dist = ranges[i]
        if not np.isfinite(dist):
            continue  # Skip bad readings

        global_angle = current_heading + angle
        px = dist * np.cos(global_angle) + x
        py = dist * np.sin(global_angle) + y
        z_points = np.vstack((z_points, [px, py]))  # Add this point

    # After the loop, add all current scan points at once:
    map_points = np.vstack((map_points, z_points))
    robot_path = np.vstack((robot_path, [x, y]))

    # Optional: plot map (not inside the for-loop!)
    plt.clf()
    plt.plot(map_points[:, 0], map_points[:, 1], ".k", label="Lidar Map")
    # plt.plot(robot_path[:, 0], robot_path[:, 1], "-r", label="Robot Path")
    plt.plot(x, y, "ro", label="Current Position")
    plt.title("Dot Map - Lidar Walls")
    plt.axis("equal")
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.legend()
    # plt.pause(0.005)

class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt if self.dt>0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


graph = {}
for i in range(12):
    for j in range(16):
        for z in range(4):
            neighbors = []
            node = ((16 * i + j) * 4 )+ z + 1
            
            
            if i != 0 and j == 0 and z == 0 and i != 11:
                neighbors.append(node - 64)
                neighbors.append(node + 64)            
                neighbors.append(node + 3)
                neighbors.append(node + 2)
                neighbors.append(node + 1)
            elif i != 0 and j == 0 and z == 1 and i != 11:
                neighbors.append(node + 4)
                neighbors.append(node + 69)
                neighbors.append(node + 61)
                
                neighbors.append(node + 1)
                neighbors.append(node + 3)
                neighbors.append(node - 1)
                neighbors.append(node + 65)
            elif i != 0 and j == 0 and z == 2 and i != 11:
                neighbors.append(node + 4)
            
                neighbors.append(node - 1)
                neighbors.append(node +1)
                neighbors.append(node - 2)
                neighbors.append(node - 65)
            elif i != 0 and i != 11 and j == 0 and z == 3:
                neighbors.append(node - 64)
                neighbors.append(node + 64)
            
                neighbors.append(node - 3)
                neighbors.append(node -1)
                neighbors.append(node - 2)
                neighbors.append(node + 1)
                
            elif i == 0 and j != 0 and j != 15 and z == 0:
                # neighbors.append(node + 64)
                # neighbors.append(node + 63)
                # neighbors.append(node + 66)
                # neighbors.append(node + 67)
                # neighbors.append(node + 66)
            
                neighbors.append(node + 3)
                neighbors.append(node + 1)
                neighbors.append(node + 2)
                neighbors.append(node - 1)
            elif i == 0 and j != 0 and j != 15 and z == 1:
                neighbors.append(node + 4)
                neighbors.append(node - 4)
                neighbors.append(node + 69)
                neighbors.append(node + 61)
                neighbors.append(node + 63)
                neighbors.append(node + 66)
                                        
                neighbors.append(node + 1)
                neighbors.append(node + 2)
                neighbors.append(node - 1)
                neighbors.append(node + 65)
            elif i == 0 and j != 0 and j != 15 and z == 2:
                # neighbors.append(node + 64)
                # neighbors.append(node - 64)
            
                neighbors.append(node - 1)
                neighbors.append(node - 2)
                neighbors.append(node + 1)
            elif i == 0 and j != 0 and j != 15 and z == 3:
                # neighbors.append(node + 64)
            
                neighbors.append(node - 3)
                neighbors.append(node + 1)
                neighbors.append(node - 2)
                neighbors.append(node - 1)
                
                
            elif i != 0 and j == 15  and i != 11 and z == 0:
                neighbors.append(node + 64)
                neighbors.append(node - 64)
                
                neighbors.append(node + 3)
                neighbors.append(node + 1)
                neighbors.append(node + 2)
                neighbors.append(node - 1)
            elif i != 0 and j == 15 and i != 11 and z == 1:
                neighbors.append(node - 4)
            
                neighbors.append(node + 1)
                neighbors.append(node + 2)
                neighbors.append(node - 1)
                neighbors.append(node + 65)
            elif i != 0 and j == 15 and i != 11 and z == 2:
                neighbors.append(node - 64)
            
                neighbors.append(node - 1)
                neighbors.append(node - 2)
                neighbors.append(node + 1)
                neighbors.append(node - 65)
            elif i != 0 and j == 15 and i != 11 and z == 3:
                neighbors.append(node + 64)
                neighbors.append(node - 64)
            
                neighbors.append(node - 3)
                neighbors.append(node - 2)
                neighbors.append(node - 1)
                
            elif i == 11 and j != 15  and j != 0 and z == 0:
                neighbors.append(node - 64)
                neighbors.append(node - 63)
                neighbors.append(node - 67)
                neighbors.append(node - 3)
                neighbors.append(node - 2)
            
                neighbors.append(node + 3)
                neighbors.append(node + 1)
                neighbors.append(node + 2)
                neighbors.append(node - 1)
            elif i == 11 and j != 15 and j != 0 and z == 1:
                neighbors.append(node + 4)
                neighbors.append(node - 4)
                neighbors.append(node - 2)
                neighbors.append(node + 3)
            
                neighbors.append(node + 1)
                neighbors.append(node + 2)
                neighbors.append(node - 1)
            elif i == 11 and j != 15 and j != 0 and z == 2:
                neighbors.append(node - 4)
                neighbors.append(node + 4)
                neighbors.append(node - 63)
                neighbors.append(node - 66)
                neighbors.append(node - 67)
            
                neighbors.append(node - 1)
                neighbors.append(node - 2)
                neighbors.append(node + 1)
                neighbors.append(node - 65)
            elif i == 11 and j != 15 and j != 0 and z == 3:
                neighbors.append(node - 64)
                neighbors.append(node - 66)
                neighbors.append(node - 62)
                neighbors.append(node - 63)
            
                neighbors.append(node - 3)
                neighbors.append(node - 1)
                neighbors.append(node - 2)
                neighbors.append(node + 1)
                
            elif i != 0 and i != 11  and j != 0 and j != 15 and z == 0:
                neighbors.append(node + 64)
                neighbors.append(node - 64)                
                neighbors.append(node + 65)                
                neighbors.append(node - 65)                
                neighbors.append(node - 67) 
                neighbors.append(node + 67)     
                
                neighbors.append(node - 2)            
                neighbors.append(node - 3)
                
                neighbors.append(node + 63)
                neighbors.append(node - 63)
                           
                neighbors.append(node + 3)
                neighbors.append(node + 1)
                neighbors.append(node + 2)
                neighbors.append(node - 1)
            elif i != 0 and i != 11  and j != 0 and j != 15 and z == 1:
                neighbors.append(node + 4)
                neighbors.append(node - 4)
                neighbors.append(node + 69)
                neighbors.append(node + 61)
                neighbors.append(node + 62)
                neighbors.append(node + 67)
                neighbors.append(node - 2)
                neighbors.append(node + 3)
                            
                neighbors.append(node + 1)
                neighbors.append(node + 2)
                neighbors.append(node - 1)
                neighbors.append(node + 65)
            elif i != 0 and i != 11  and j != 0 and j != 15 and z == 2:
                neighbors.append(node + 4)
                neighbors.append(node - 4)
                neighbors.append(node - 63)
                neighbors.append(node - 66)
                neighbors.append(node - 67)
                neighbors.append(node - 62)
                neighbors.append(node - 61)
                neighbors.append(node - 69)
                
                neighbors.append(node - 1)
                neighbors.append(node - 2)
                neighbors.append(node + 1)
                neighbors.append(node - 65)
            elif i != 0 and i != 11  and j != 0 and j != 15 and z == 3:
                neighbors.append(node + 64)
                neighbors.append(node - 64)
                neighbors.append(node - 63)
                neighbors.append(node + 66)
                neighbors.append(node + 2)
                neighbors.append(node + 3)
                neighbors.append(node + 63)
            
                neighbors.append(node - 3)
                neighbors.append(node - 1)
                neighbors.append(node - 2)
                neighbors.append(node + 1)
                
            elif i == 0 and j == 0 and z == 0 :
                neighbors.append(node + 64)
            
                neighbors.append(node + 3)
                neighbors.append(node + 1)
                neighbors.append(node + 2)
            elif i == 0 and j == 0 and z == 1 :
                neighbors.append(node + 4)
            
                neighbors.append(node + 1)
                neighbors.append(node + 2)
                neighbors.append(node - 1)
                neighbors.append(node + 65)
            elif i == 0 and j == 0 and z == 2 :
                neighbors.append(node + 4)
            
                neighbors.append(node - 1)
                neighbors.append(node - 2)
                neighbors.append(node + 1)
            elif i == 0 and j == 0 and z == 3 :
                neighbors.append(node + 64)
            
                neighbors.append(node - 3)
                neighbors.append(node - 1)
                neighbors.append(node - 2)
                neighbors.append(node + 1)
                
                
            elif i == 0 and j == 15 and z == 0 :
                neighbors.append(node + 64)
            
                neighbors.append(node + 3)
                neighbors.append(node + 1)
                neighbors.append(node + 2)
                neighbors.append(node - 1)    
            elif i == 0 and j == 15 and z == 1 :
                neighbors.append(node - 4)
            
                neighbors.append(node + 1)
                neighbors.append(node + 2)
                neighbors.append(node - 1)
                neighbors.append(node + 65)
            elif i == 0 and j == 15 and z == 2 :
                neighbors.append(node - 4)
            
                neighbors.append(node - 1)
                neighbors.append(node - 2)
                neighbors.append(node + 1)
            elif i == 0 and j == 15 and z == 3 :
                neighbors.append(node + 64)
            
                neighbors.append(node - 3)
                neighbors.append(node - 1)
                neighbors.append(node - 2)
                
            elif i == 11 and j == 0 and z == 0 :
                neighbors.append(node - 64)
            
                neighbors.append(node + 3)
                neighbors.append(node + 1)
                neighbors.append(node + 2)    
            elif i == 11 and j == 0 and z == 1 :
                neighbors.append(node + 4)
            
                neighbors.append(node + 1)
                neighbors.append(node + 2)
                neighbors.append(node - 1)
            elif i == 11 and j == 0 and z == 2 :
                neighbors.append(node + 4)
            
                neighbors.append(node - 1)
                neighbors.append(node + 1)
                neighbors.append(node - 2)
                neighbors.append(node - 65)
            elif i == 11 and j == 0 and z == 3 :
                neighbors.append(node - 64)
            
                neighbors.append(node - 3)
                neighbors.append(node - 1)
                neighbors.append(node - 2)
                neighbors.append(node + 1)
                
            elif i == 11 and j == 15 and z == 0 :
                neighbors.append(node - 64)
            
                neighbors.append(node + 3)
                neighbors.append(node + 1)
                neighbors.append(node + 2) 
                neighbors.append(node - 1)                       
            elif i == 11 and j == 15 and z == 1 :
                neighbors.append(node - 4)
            
                neighbors.append(node + 1)
                neighbors.append(node + 2)
                neighbors.append(node - 1)
            elif i == 11 and j == 15 and z == 2 :
                neighbors.append(node - 4)
            
                neighbors.append(node - 1)
                neighbors.append(node + 1)
                neighbors.append(node - 2)
                neighbors.append(node - 65)
            elif i == 11 and j == 15 and z == 3 :
                neighbors.append(node - 64)
            
                neighbors.append(node - 3)
                neighbors.append(node - 1)
                neighbors.append(node - 2)
                
            graph[node] = neighbors
# print(graph)

pid_dist = PID(2.0, 0.0, 0.5, dt)
pid_ang  = PID(5.0, 0.0, 1.0, dt)
pos_tolerance = 0.085
ang_tolerance = 0.07
WHEEL_RADIUS = 0.0205
AXLE_LENGTH  = 0.053


def draw_lidar_data(ranges, fov, display, display_width, display_height, scale):
    """
    Visualize lidar data on the display device for Zone 1
    Args:
        ranges: List of lidar distance measurements
        fov: Field of view of the lidar in radians
        display: Webots display device reference
        display_width: Width of display in pixels
        display_height: Height of display in pixels
        scale: Scaling factor for visualization
    """
    center_x = display_width // 2
    center_y = display_height // 2
    
    # Clear the display with white background
    display.setColor(0xFFFFFF)
    display.fillRectangle(0, 0, display_width, display_height)
    
    # Draw zone identifier
    display.setColor(0x000000)  # Black text
    display.setFont("Arial", 6, True)
    display.drawText('Zone 1', 0, 0)
    
    # Draw lidar points (blue for Zone 1)
    display.setColor(0x0000FF)
    num_points = len(ranges)
    
    for i in range(num_points):
        angle = -fov/2 + i * (fov/(num_points-1))
        distance = ranges[i]
        
        if math.isfinite(distance):
            # Convert polar to Cartesian coordinates
            x = distance * math.sin(angle)
            y = distance * math.cos(angle)
            
            # Convert to display coordinates
            px = center_x + int(x * scale)
            py = center_y - int(y * scale)  # Y axis is inverted in display
            
            if 0 <= px < display_width and 0 <= py < display_height:
                display.drawPixel(px, py)

def clamp(value, min_val, max_val):
    """Clamp a value between min and max bounds"""
    return max(min(value, max_val), min_val)

def get_compass_heading(compass_values):
    """Calculate compass heading in radians from compass values"""
    rad = math.atan2(compass_values[0], compass_values[1])
    return rad if rad >= 0 else rad + 2*math.pi

def is_key_pressed(key, expected_keys):
    """Check if any of the expected keys is pressed"""
    return any(key == ord(k) for k in expected_keys)


def angle_diff(a, b):
    d = a - b
    while d > math.pi:   d -= 2*math.pi
    while d < -math.pi:  d += 2*math.pi
    return d

def is_on_blue_tile():
    values = [sensor.getValue() for sensor in gs]
    avg = sum(values) / len(values)
    print(f"📘 Ground Avg = {avg:.2f}")
    return 490 < avg < 530  # بازه تجربی برای رنگ آبی


def drive_to(target_x , target_y ):
    # second_gv = [10 , 10] 
    pid_ang.reset()   # اختیاری: ریست خطای زاویه
    pid_dist.reset() 
    gs_values = [sensor.getValue() for sensor in gs]
    # print(gs_values)
    gps_values = gps.getValues()
    first_gv = gps.getValues()
    # m = ((-gps_values[0])-(-target_x)) / (gps_values[1]-target_y)
    dy = target_y-gps_values[1]
    dx = target_x - gps_values[0]
    target_angle = math.atan2(dy , dx)
    # print(math.degrees(math.atan(m)))
    # print(math.degrees(target_angle))
    while robot.step(timestep) != -1 : 
        compass_values = compass.getValues()
        heading = get_compass_heading(compass_values)       
        diff = angle_diff(target_angle , heading)
        # print(math.degrees(diff))
        if abs(diff) < ang_tolerance:
            # وقتی زاویه تنظیم شد، ایست کن
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            break
        omega = pid_ang.compute(diff)
        v_l = -omega * (AXLE_LENGTH/(2*WHEEL_RADIUS))
        v_r =  omega * (AXLE_LENGTH/(2*WHEEL_RADIUS))
            # محدودسازی سرعت
        v_l = max(-MAX_SPEED, min(MAX_SPEED, v_l))
        v_r = max(-MAX_SPEED, min(MAX_SPEED, v_r))
        left_motor.setVelocity(v_l)
        right_motor.setVelocity(v_r)
    pid_ang.reset()   # اختیاری: ریست خطای زاویه
    pid_dist.reset()  # اختیاری: ریست خطای فاصله
    gps_values = gps.getValues()
    x3 = gps_values[0]
    y3 = gps_values[1]
    x = gps_values[0]
    y = gps_values[1]
    dx, dy = target_x - x, target_y - y
    dist = math.hypot(dx, dy)
    scan = lidar.getRangeImage()
    mid = len(scan) // 2
    min_dist = min(scan[(mid - 31) : (mid + 31)])  
    if not np.isfinite(min_dist):
        min_dist = 0
    print("فاصله تا مقصد و فاصله تا نزدیک ترین مانع")        
    print(dist , min_dist)  
    if min_dist < dist :
        return 1
    while robot.step(timestep) != -1:
        gs_values = [sensor.getValue() for sensor in gs]
        avg = sum(gs_values) / len(gs_values)    
        if avg > 901 :
            for led in leds:
                led.set(1)
            print("Red tile detected! 🚨")
        else :
            for led in leds:
                led.set(0) 
        gps_values = gps.getValues()
        # print("جایی که هستی")
        # print(gps_values)
        # print("اخرین جایی که بودی")
        # print(second_gv)
        x = gps_values[0]
        y = gps_values[1]
        # if abs(round(second_gv[0], 4) - round(x, 4)) < 0.0008 and abs(round(second_gv[1], 4) - round(y, 4)) < 0.0008:
            # drive_to(first_gv[0], first_gv[1])
            # return 1
        dx, dy = target_x - x, target_y - y
        dist = math.hypot(dx, dy) 
        if dist < pos_tolerance and abs(angle_diff(target_angle, heading)) < ang_tolerance :
            # رسیدیم
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            break
        # چون دیگر زاویه تنظیم است، فقط PID فاصله فعال
        v = pid_dist.compute(dist)
        # تبدیل سرعت خطی به سرعت زاویه‌ای چرخ
        v_l = v / WHEEL_RADIUS
        v_r = v / WHEEL_RADIUS
        # محدودسازی
        v_l = max(-MAX_SPEED, min(MAX_SPEED, v_l))
        v_r = max(-MAX_SPEED, min(MAX_SPEED, v_r))
        left_motor.setVelocity(v_l)
        right_motor.setVelocity(v_r)
        second_gv = gps.getValues()
def is_reachable(x3 , y3 , x_target , y_target):
    """بررسی وجود دیوار بین دو سلول با استفاده از زاویه صحیح LiDAR."""
    # print(cell , nxt)
    compass_values = compass.getValues()
    heading = get_compass_heading(compass_values)
    # row = math.floor((cell-0.1) / 16)
    # col = (cell - (row*16)) - 1
    # x0 = ((row+0.5)*0.25) - 1.5
    # y0 = ((col+0.5)*0.25) - 2
    # print(row , col)
    x0 = x3
    y0 = y3
    # row2 = math.floor((nxt-0.1) / 16)
    # col2 = (nxt - (row2*16)) - 1
    # print(row2 , col2)
    # x1 = ((row2+0.5)*0.25) - 1.5
    # y1 = ((col2+0.5)*0.25) - 2
    x1 = x_target
    y1 = y_target
    print(x0 , y0 , x1 , y1)
    dx = x1 - x0
    dy = y1 - y0
    print(dx)
    print(dy)
    angle = math.atan2(dy, dx)  # زاویه دقیق بین دو سلول
    print(math.degrees(angle))
    print(math.degrees(heading))
    angle = angle_diff(heading , angle)
    print(math.degrees(angle))
    # اطلاعات LiDAR
    scan = lidar.getRangeImage()
    fov = lidar.getFov()  # مثلاً 240° = 4.18879
    res = len(scan)       # 667

    # محدود کردن زاویه به FOV
    angle = max(-fov/2, min(fov/2, angle))

    # نگاشت زاویه به ایندکس در scan[]
    ratio = (angle + fov / 2) / fov
    idx = int(ratio * (res - 1))

    distance = [scan[idx-3] , scan[idx-2] , scan[idx-1] ,  scan[idx] ,  scan[idx+1] ,  scan[idx+2] , scan[idx+3]]
    count = 0 
    # print(f"زاویه: {math.degrees(angle):.1f}° | ایندکس: {idx} | فاصله: {distance:.2f} متر")
    for i in distance :
        print("فاصله ها")
        print(i)
        if i > 0.2 :
            count += 1
    if count == len(distance):
        return True
    # return distance > 0.1  # اگر فاصله بیشتر از ۲۰ سانتی‌متر باشد، راه باز است


    

visited = []
def dfs(current_node):
    gs_values = [sensor.getValue() for sensor in gs]
    gps_values = gps.getValues()
    x3 = gps_values[0]
    y3 = gps_values[1]
    # print(x3 , y3)
    # row2 = ((current_node - 0.1) // 4) // 16
    # col2 = ( (current_node - 0.1) - ((row2*16)*4) ) // 4
    # place2 = current_node - ((col2+(row2 * 16))*4) - 1
    visited.append(current_node)
    # if place2 == 0 :
        # visited.append(current_node+1)
        # visited.append(current_node+2)
        # visited.append(current_node+3)      
    # if place2 == 1 :
        # visited.append(current_node-1)
        # visited.append(current_node+1)
        # visited.append(current_node+2)
    # if place2 == 2 :
        # visited.append(current_node-2)
        # visited.append(current_node-1)
        # visited.append(current_node+1)
    # if place2 == 3 :
        # visited.append(current_node-3) 
        # visited.append(current_node-2)
        # visited.append(current_node-1)

    print(visited)
    for nbr in graph[current_node]:
        print("نودی که توشیم")
        print(current_node)
        print("همسایه های نود")        
        print(graph[current_node])
        row = ((nbr - 0.1) // 4) // 16
        col = ( (nbr - 0.1) - ((row*16)*4) ) // 4
        place = nbr - ((col+(row * 16))*4) - 1
        print("سطر و ستون نود همسایه")        
        print(row , col , place)
        # row2 = math.floor((node - 0.1) / 16)
        # col2 = (node - (row*16)) - 1
        x0 = -1.5 + (row * 0.25)
        x1 = x0 + 0.25
        # print(x0 , x1)
        y0 = -2 + (col * 0.25)
        y1 = y0 + 0.25
        # print(y0 , y1)
        if place == 0 or place == 3 :
            x_target = (x0 + x1) /2
        elif place == 1 :
            temp = (x0 + x1) /2
            x_target = (x1 + temp) / 2
        elif place == 2 :
            temp = (x0 + x1) /2
            x_target = (x0 + temp) / 2
            
        if place == 1 or place == 2 :
            y_target =(y0 + y1) / 2
        elif place == 0 :
            temp = (y0 + y1) / 2
            y_target = (temp + y0)/2
        elif place == 3 :
            temp = (y0 + y1) / 2
            y_target = (temp + y1)/2
          
        # print(x_target , y_target)
        
            
        # target_x = ((row+0.5)*0.25) - 1.5
        # target_y = ((col+0.5)*0.25) - 2
        # print(target_x , target_y)
        # print(x3 , y3)
        if nbr in visited:
            continue
        # drive_to(target_x , target_y)
        # if drive_to(target_x , target_y) == 1 :
            # print("دیوار")
            # continue
        # distance , distance1 = is_reachable(current_node, nbr)
        # print("فاصله ها")
        # print(distance , distance1)
        # if distance > 0.2:
            # if abs(distance - 0.25) < 0.04:
                # dx = target_x - x3
                # dy = target_y - y3
                # if dy == 0 and dx > 0:
                    # if (distance1 - distance) > 0 :
                        # drive_to(x + 0.1875 , target_y- 0.1875)
                    # elif (distance1 - distance) < 0 : 
                        # drive_to(x + 0.1875 , target_y + 0.1875)
                # if dy == 0 and dx < 0:
                    # if (distance1 - distance) > 0 :
                        # drive_to(x + 0.1875 , target_y- 0.1875)
                    # elif (distance1 - distance) < 0 : 
                        # drive_to(x + 0.1875 , target_y + 0.1875)
                # if is_on_blue_tile():
                    # print("🔵 ربات روی کاشی آبی قرار گرفت!")
                    # drive_to(x3,y3)
                    # continue
            # dfs(nbr)
            # drive_to(x3,y3)
            # continue
            # print("دیوار")
            # continue
        # if not is_reachable(x3 , y3 , x_target , y_target):
            # print("دیوار")
            # continue
        if drive_to(x_target , y_target) == 1 :
            print("دیوار")
            continue
        ranges = lidar.getRangeImage()
        fov = lidar.getFov()
        compass_values = compass.getValues()
        heading = get_compass_heading(compass_values)
        gps_values = gps.getValues()
        mapping(gps_values, heading, ranges, fov) 
        if is_on_blue_tile():
            print("🔵 ربات روی کاشی آبی قرار گرفت!")
            drive_to(x3,y3)
            continue
        dfs(nbr)
        drive_to(x3,y3)
    # temp = visited[len(visited)-2]
    # print(temp)
    # row = math.floor((temp-0.1) / 16)
    # col = (temp - (row*16)) - 1
    # x_f = ((row+0.5)*0.25) - 1.5
    # y_f = ((col+0.5)*0.25) - 2
    # drive_to(x_f,y_f)

    


# Initialize keyboard control
# keyboard = Keyboard()
# keyboard.enable(timestep)

# Initialize LEDs
leds = [robot.getDevice(f'led{i}') for i in range(8)]

# Initialize motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Initialize lidar
lidar = robot.getDevice('Hokuyo URG-04LX')
lidar.enable(timestep)
lidar.enablePointCloud()

# Initialize display
# display = robot.getDevice('display_zone_1')
# display_width = display.getWidth()
# display_height = display.getHeight()

# Initialize ground sensors
gs = [robot.getDevice(f'gs{i}') for i in range(3)]
for sensor in gs:
    sensor.enable(timestep)

# Initialize GPS
gps = robot.getDevice('gps')
gps.enable(timestep)

# Initialize Compass
compass = robot.getDevice('compass')
compass.enable(timestep)

# Movement control variables
current_forward_speed = 0.0
current_turn_speed = 0.0
forward_active = False
turn_active = False

# Main control loop
while robot.step(timestep) != -1:
    # Process lidar data and visualization
    ranges = lidar.getRangeImage()
    fov = lidar.getFov()
    mid = len(ranges) // 2
    dis = min(ranges[mid-40 : mid +40])
    print("faseleeeeee")
    print(dis)
    if dis < 0.3 :
    # print(math.degrees(fov))
    # draw_lidar_data(ranges, fov, display, display_width, display_height, DISPLAY_SCALE)
    
        # Read all sensors
        gs_values = [sensor.getValue() for sensor in gs]
        gps_values = gps.getValues()
        # print(gps_values)
        compass_values = compass.getValues()
        heading = get_compass_heading(compass_values)
        # print(math.degrees(heading))
        pos = gps_values
        current_node = math.floor((gps_values[0]+1.5)/0.25)*16 + math.floor((gps_values[1]+2)/0.25) + 1
        # print(current_node)
        row = math.floor((current_node - 0.1) / 16)
        col = (current_node - (row*16)) - 1
        x0 = -1.5 + (row * 0.25)
        x1 = x0 + 0.25
        # print(x0 , x1)
        
        y0 = -2 + (col * 0.25)
        y1 = y0 + 0.25
        
        # print(y0 , y1)
        if pos[1] < (pos[0] + (y1-x1)) :
            t = [0,1]
        else :
            t = [2,3]
        # print(t)
        if pos[1] < (-pos[0] - ((-x0)+(-y1))):
            t2 = [0,2]
        else :
            t2 = [1,3] 
        # print(t2)   
        current_node = list(set(t) & set(t2))
        cn = (row * 16 * 4) + (col * 4 ) + current_node[0] + 1
        
        # print(cn)
        # for i in graph[current_node]:
        #     row = math.floor(i / 16)
        #     col = (i - (row*16)) - 1
        #     target_x = ((row+0.5)*0.25) - 1.5
        #     target_y = ((col+0.5)*0.25) - 2
        #     m = ((-gps_values[0])-(-target_x)) / (gps_values[1]-target_y)
        #     target_angle = math.atan(m)
        #     pid_dist.reset()
        #     pid_ang.reset()
        #     while robot.step(timestep) != -1 :        
        #         if math.degrees(target_angle) - math.degrees(heading) < 5:
        #             left_motor.setVelocity(3)
        #             right_motor.setVelocity(3)
        #             break 
        #         diff = angle_diff(target_angle , heading)
        #         omega = pid_ang.compute(diff)
        #         v_l = -omega * (AXLE_LENGTH/(2*WHEEL_RADIUS))
        #         v_r =  omega * (AXLE_LENGTH/(2*WHEEL_RADIUS))
        #             # محدودسازی سرعت
        #         v_l = max(-MAX_SPEED, min(MAX_SPEED, v_l))
        #         v_r = max(-MAX_SPEED, min(MAX_SPEED, v_r))
        #         left_motor.setVelocity(v_l)
        #         right_motor.setVelocity(v_r)
        #     pid_ang.reset()   # اختیاری: ریست خطای زاویه
        #     pid_dist.reset()  # اختیاری: ریست خطای فاصله
        #     while robot.step(timestep) != -1:
        #         x = gps_values[0]
        #         y = gps_values[1]
        #         dx, dy = target_x - x, target_y - y
        #         dist = math.hypot(dx, dy)
        #         if dist < pos_tolerance:
        #             # رسیدیم
        #             left_motor.setVelocity(0)
        #             right_motor.setVelocity(0)
        #             break
        #         # چون دیگر زاویه تنظیم است، فقط PID فاصله فعال
        #         v = pid_dist.compute(dist)
        #         # تبدیل سرعت خطی به سرعت زاویه‌ای چرخ
        #         v_l = v / WHEEL_RADIUS
        #         v_r = v / WHEEL_RADIUS
        #         # محدودسازی
        #         v_l = max(-MAX_SPEED, min(MAX_SPEED, v_l))
        #         v_r = max(-MAX_SPEED, min(MAX_SPEED, v_r))
        #         left_motor.setVelocity(v_l)
        #         right_motor.setVelocity(v_r)
        dfs(cn)
        map1 = np.load("/Users/hosseinmac/Downloads/mini_project/controllers/my_controller_zone_1/zone1_map_points.npy")
        path1 = np.load("/Users/hosseinmac/Downloads/mini_project/controllers/my_controller_zone_1/zone1_robot_path.npy")
            # رسم نقشه نهایی
            
        map2 = map_points
        path2 = robot_path
          
        plt.figure(figsize=(10, 8))
            
            # نقاط Lidar
        plt.plot(map1[:, 0], map1[:, 1], ".", color="blue", label="Zone 1 Lidar")
        plt.plot(map2[:, 0], map2[:, 1], ".", color="green", label="Zone 2 Lidar")
            
            # مسیر حرکت ربات‌ها
            # plt.plot(path1[:, 0], path1[:, 1], "-", color="red", label="Robot 1 Path")
            # plt.plot(path2[:, 0], path2[:, 1], "-", color="orange", label="Robot 2 Path")
            
            # مرز بین دو ناحیه (اختیاری)
        plt.axhline(y=-1.0, color='blue', linestyle='--', label="Zone Border")
            
        plt.axis("equal")
        plt.grid(True, linestyle="--", alpha=0.5)
        plt.title("🗺️ Final Combined Maze Map")
        plt.legend()
        plt.tight_layout()
            
            # ذخیره تصویر
        plt.savefig("full_maze_map.png")
        plt.close()
        print("✅ full_maze_map.png ذخیره شد.")
        break
        # Print sensor readings
        # print(f'--- Zone 1 Sensor Readings ---')
        # print(f'Ground Sensors: {[f"{v:.2f}" for v in gs_values]}')
        # print(f'GPS Position: X={gps_values[0]:.2f}, Y={gps_values[1]:.2f}, Z={gps_values[2]:.2f}')
        # print(f'Compass Heading: {math.degrees(heading):.1f}°')
        # print('-----------------------------')
        
    
        # # Handle keyboard input (arrow key controls)
        # key = keyboard.getKey()
        
        # # Forward/backward movement control
        # if key == Keyboard.UP:
        #     current_forward_speed = min(current_forward_speed + ACCELERATION, MAX_SPEED)
        #     forward_active = True
        # elif key == Keyboard.DOWN:
        #     current_forward_speed = max(current_forward_speed - ACCELERATION, -MAX_SPEED)
        #     forward_active = True
        # elif forward_active:  # Key released
        #     current_forward_speed = 0.0
        #     forward_active = False
        
        # # Turning control
        # if key == Keyboard.RIGHT:
        #     current_turn_speed = min(current_turn_speed + ACCELERATION, MAX_SPEED * TURN_RATIO)
        #     turn_active = True
        # elif key == Keyboard.LEFT:
        #     current_turn_speed = max(current_turn_speed - ACCELERATION, -MAX_SPEED * TURN_RATIO)
        #     turn_active = True
        # elif turn_active:  # Key released
        #     current_turn_speed = 0.0
        #     turn_active = False
        
        # # Calculate and set motor speeds
        # left_speed = clamp(current_forward_speed + current_turn_speed, -MAX_SPEED, MAX_SPEED)
        # right_speed = clamp(current_forward_speed - current_turn_speed, -MAX_SPEED, MAX_SPEED)
        
        # left_motor.setVelocity(left_speed)
        # right_motor.setVelocity(right_speed)
    
    