
import logging
import time
import os
import threading
import queue
import math
import asyncio

from freenove.Server.car import Car
from freenove.Server.led import Led

from rplidarc1 import RPLidar

from inputs import get_gamepad

LIDAR_PORT_NAME = "/dev/ttyUSB0"
MIN_TANK_SERVO_SPEED = 800
STICK_FILTER_MIN = 5000
ROTATION_SPEED = 5000
AUTOMOVE_SPEED = 3000

class TankControl:
    def __init__(self):
        self.led = Led()                               # Initialize the LED object
        self.car = Car()                               # Initialize the car object
        self.car_thread = None                         # Initialize the car thread
        self.led_process = None                        # Initialize the LED process
        self.action_process = None                     # Initialize the action process
        self.cmd_thread_is_running = False             # Initialize the command thread running state
        self.video_thread_is_running = False           # Initialize the video thread running state
        self.car_thread_is_running = False             # Initialize the car thread running state
        self.led_process_is_running = False            # Initialize the LED process running state
        self.action_process_is_running = False         # Initialize the action process running state
        self.car_mode = 1                              # Initialize the car mode
        self.car_last_mode = 1                         # Initialize the last car mode
        self.left_wheel_speed = 0                      # Initialize the left wheel speed
        self.right_wheel_speed = 0                     # Initialize the right wheel speed

        # self.set_threading_car_task(True)  # Start the car task thread
        # self.set_process_led_running(True)  # Start the LED process

        self.left_wheel_speed = 0  # Set the left wheel speed to 0
        self.right_wheel_speed = 0  # Set the right wheel speed to 0
        self.car.motor.setMotorModel(self.left_wheel_speed, self.right_wheel_speed)  # Set the motor model

    def run(self, left_speed: int, right_speed: int):
        self.left_wheel_speed = left_speed  # Set the left wheel speed to 0
        self.right_wheel_speed = right_speed  # Set the right wheel speed to 0
        self.car.motor.setMotorModel(self.left_wheel_speed, self.right_wheel_speed)  # Set the motor model

    def stop(self):
        self.led.colorWipe([0, 0, 0])                  # Turn off the LEDs
        # self.camera.stop_stream()                      # Stop the camera stream
        # self.camera.close()                            # Close the camera
        self.car.close()                               # Close the car

def gamepad_reader(event_queue):
    while True:
        events = get_gamepad()  # This blocks in the thread
        for event in events:
            event_queue.put(event)

async def wait_and_stop(seconds, event):
    await asyncio.sleep(seconds)
    event.set()
    
async def process_queue(q: asyncio.Queue, event: asyncio.Event):
    while not event.is_set():
        if q.qsize() < 10:
            print("Printer sleeping for more data")
            await asyncio.sleep(0.1)
        out: dict = await q.get()
        print(out)

async def main(lidar: RPLidar):

    tank = TankControl()
    tank.run(0, 0)

    # Create a thread-safe queue to pass events
    event_queue = queue.Queue()

    # Start the gamepad reading thread
    reader_thread = threading.Thread(target=gamepad_reader, args=(event_queue,), daemon=True)
    reader_thread.start()

    speed_left = 0
    speed_right = 0
    value_y = 0
    x_millimeter = .0
    x_automove_distance = 1000.0
    x_automove_timer = 0
    mode_steering = False
    mode_automove = False
    scan_button_last_state = False
    new_file = True
    current_filename = get_available_filename("scan.ply")
    last_milli_time = current_milli_time()

    while True:

        #print("Cycle")

        #events = get_gamepad()

        events_to_process = []

        while not event_queue.empty():
            try:
                event = event_queue.get_nowait()
                events_to_process.append(event)
            except queue.Empty:
                break

        for event in events_to_process:
            print(event.ev_type, event.code, event.state)
                        
            if event.code == "ABS_RY" and not mode_steering and not mode_automove:
                value_y = int(event.state) / 4
                if value_y >= STICK_FILTER_MIN or value_y <= -STICK_FILTER_MIN:
                    speed_left = value_y
                    speed_right = speed_left
                else:
                    speed_left = 0
                    speed_right = speed_left
            elif event.code == "ABS_RX" and value_y > -STICK_FILTER_MIN and value_y < STICK_FILTER_MIN and not mode_automove:
                value = int(event.state) / 4
                mode_steering = True if value > STICK_FILTER_MIN or value < STICK_FILTER_MIN else False
                if value >= STICK_FILTER_MIN:
                    speed_left = -ROTATION_SPEED
                    speed_right = ROTATION_SPEED
                elif value <= -STICK_FILTER_MIN:
                    speed_right = -ROTATION_SPEED
                    speed_left = ROTATION_SPEED
                else:
                    speed_left = 0
                    speed_right = speed_left
                    mode_steering = False
            elif event.code == "BTN_SOUTH":
                if event.state == 1 and scan_button_last_state == False:
                    try:
                        lidar.healthcheck()
    
                        async with asyncio.TaskGroup() as tg:
                            tg.create_task(wait_and_stop(5, lidar.stop_event))
                            tg.create_task(process_queue(lidar.output_queue, lidar.stop_event))
                            tg.create_task(lidar.simple_scan(make_return_dict=True))
                       
                        print(lidar.output_dict)
                        
                        if new_file:
                            current_filename = get_available_filename(current_filename)

                        write_ply_from_vertical_dict(lidar.output_dict, new_file, filename=current_filename, x=x_millimeter)

                        if new_file:
                            new_file = False

                        x_millimeter += x_automove_distance
                        
                    except Exception as e:
                        print(e)
                    finally:
                        await asyncio.sleep(1)
                        lidar.shutdown()
                        lidar = RPLidar(LIDAR_PORT_NAME, 460800)
                        
                if event.state == 0:
                    scan_button_last_state = False
                        
                scan_button_last_state = event.state == 1

            elif event.code == "BTN_EAST":
                new_file = True 
                x_millimeter = .0

            elif event.code == "BTN_NORTH": # Reset
                x_automove_timer = .0
                speed_left = 0
                speed_right = speed_left
                mode_automove = False

            elif event.code == "BTN_TRIGGER_HAPPY3" and event.state == 1: # LEFT HAT UP distance 100 mm
                print("Automove 100 mm")
                x_automove_distance = 100.0
                x_automove_timer = x_automove_distance * 1.25
                speed_left = -AUTOMOVE_SPEED
                speed_right = speed_left
                mode_automove = True

            elif event.code == "BTN_TR" and event.state == 1: # RB BUTTON distance 1000 mm
                print("Automove 1000 mm")
                x_automove_distance = 1000.0
                x_automove_timer = x_automove_distance
                speed_left = -AUTOMOVE_SPEED
                speed_right = speed_left
                mode_automove = True

        #print(x_automove_timer)

        if mode_automove and x_automove_timer > 0:
            x_automove_timer -= (current_milli_time() - last_milli_time)
            #print("Automove Timer dec: ", x_automove_timer)

        if mode_automove and x_automove_timer <= 0:
            x_automove_timer = 0
            speed_left = 0
            speed_right = speed_left
            mode_automove = False

        #print(f"Speed: {speed_right}, {speed_left}")
            
        tank.run(speed_left if speed_left >= MIN_TANK_SERVO_SPEED or speed_left <= -MIN_TANK_SERVO_SPEED else 0, speed_right if speed_right >= MIN_TANK_SERVO_SPEED or speed_right <= -MIN_TANK_SERVO_SPEED else 0)

        last_milli_time = current_milli_time()

        #print(last_milli_time)

        time.sleep(0.01) # 10 ms sleep
            
    tank.stop()


import math

def write_ply_from_vertical_dict(data_dict: dict, new_file: bool = False, filename="scan.ply", x=0.0):
    """
    data_dict: {winkel_in_grad: entfernung_in_mm oder None}
    filename: .ply-Dateiname
    x: fester X-Wert bei vertikaler Ausrichtung
    """

    points = []

    for angle_deg in sorted(data_dict.keys()):
        distance_mm = data_dict[angle_deg]
        if distance_mm is None or distance_mm == 0:
            continue  # ungültiger Punkt
        angle_rad = math.radians(angle_deg)
        y = distance_mm * math.cos(angle_rad)
        z = distance_mm * math.sin(angle_rad)

        # Achsen-Anpassung für Blender (default import Y, Z, Scale 0.001:
        bx = x           # bleibt gleich
        by = z           # wird zu Y (oben)
        bz = y           # Y wird zu Z (vorne)

        points.append((bx, by, bz))

        #points.append((x, y, z))

    with open(filename, "w" if new_file else "a") as f:
        if new_file:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")
        for x, y, z in points:
            f.write(f"{x:.2f} {y:.2f} {z:.2f}\n")

    print(f".ply-Datei erfolgreich exportiert: {filename} mit {len(points)} Punkten")

def current_milli_time():
    return round(time.time() * 1000)

def get_available_filename(filename):
    if not os.path.exists(filename):
        return filename
        
    base, ext = os.path.splitext(filename)
    counter = 1
    while True:
        new_filename = f"{base}{counter}{ext}"
        if not os.path.exists(new_filename):
            return new_filename
        counter += 1


if __name__ == '__main__':
    
    logging.basicConfig(level=0)
    lidar = RPLidar(LIDAR_PORT_NAME, 460800)
    
    try:
        asyncio.run(main(lidar))
    except KeyboardInterrupt:
        time.sleep(1)
        lidar.reset()
        lidar.shutdown()

