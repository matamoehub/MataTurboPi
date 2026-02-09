from common.lib.robot import drive, stop

# GIVEN
def forward(speed=40, secs=0.4):
    drive(speed, speed, speed, speed, secs)

def move_left(speed=30, secs=0.3):
    # strafe left (mecanum)
    drive(-speed, speed, speed, -speed, secs)

def turn_left(speed=30, secs=0.25):
    drive(-speed, speed, -speed, speed, secs)

def drift_left(speed=40, secs=0.3):
    # gentle curve left
    drive(20, speed, 20, speed, secs)

# STUDENTS COMPLETE (TODO)
def back(speed=40, secs=0.4):
    pass

def move_right(speed=30, secs=0.3):
    pass

def turn_right(speed=30, secs=0.25):
    pass

def drift_right(speed=40, secs=0.3):
    pass

def stop_now():
    stop()

