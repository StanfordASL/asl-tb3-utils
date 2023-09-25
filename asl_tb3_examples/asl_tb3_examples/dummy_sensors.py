from termcolor import colored

def get_imu_data() -> None:
    print(colored("[IMU]: feeling dizzy", "green"))

def get_lidar_data() -> None:
    print(colored("[LiDAR]: getting new scans", "cyan"))

def get_motor_encoder_data() -> None:
    print(colored("[Motor Encoder]: keeping up with the spins", "magenta"))

def get_heart_beat() -> None:
    print(colored("[Heart Beat]: alive!", "red"))

def get_joystick_data() -> None:
    print(colored("[Joystick]: new input", "cyan"))
