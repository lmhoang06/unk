from controller import Robot
from controller import Motor
from controller import DistanceSensor
from controller import Camera
from controller import LED
from controller import Supervisor
import math

robot = Robot()

# get the time step of the current world
timestep = 8
robot.step(timestep)

# camera
cam = robot.getDevice("camera")
cam.enable(64)
# Leff motor, Right motor
lm = robot.getDevice("left wheel motor")
lm.setPosition(float("inf"))
lm.setVelocity(0)

rm = robot.getDevice("right wheel motor")
rm.setPosition(float("inf"))
rm.setVelocity(0)

# Sensors
NB_GROUND_SENS = 8
gs = []
gsNames = ["gs0", "gs1", "gs2", "gs3", "gs4", "gs5", "gs6", "gs7"]
for i in range(NB_GROUND_SENS):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

# LEDs
NB_LEDS = 5
leds = []
led_Names = ["led0", "led1", "led2", "led3", "led4"]
for i in range(NB_LEDS):
    leds.append(robot.getDevice(led_Names[i]))


### Private Functions ###
# Function to control LEDs
def LED_Alert():
    if (robot.getTime() - initTime) * 1000 % 3000 >= 2000:
        # leds[1].set(not(leds[1].get()))
        leds[1].set(1)
        # for i in range(NB_LEDS):
        # leds[i].set(not(leds[i].get()))
    return


# Waiting for completing initialization
### thí sinh không được bỏ phần này
initTime = robot.getTime()
while robot.step(timestep) != -1:
    if (robot.getTime() - initTime) * 1000.0 > 200:
        break


### Phần code cần chỉnh sửa cho phù hợp ##
# Định nghĩa các tín hiệu của xe
NOP = -1
MID = 0
LEFT = 1
RIGHT = -1
FULL_SIGNAL = 2
BLANK_SIGNAL = -2

HALT = 0x3F3F3F3F

MAX_SPEED = 7
threshold = [330, 330, 330, 330, 330, 330, 330, 330]
preFilted = 0b00000000

# Biến lưu giá trị tỉ lệ tốc độ của động cơ
left_ratio = 0.0
right_ratio = 0.0


# Hàm đọc giá trị của sensors
def ReadSensors():
    gsValues = []
    filted = 0x00
    for i in range(NB_GROUND_SENS):
        gsValues.append(gs[i].getValue())
        if gsValues[i] > threshold[i]:
            filted |= 0x01 << (NB_GROUND_SENS - i - 1)
    # print(*gsValues, sep = '\t')
    return filted


# Phần code điều khiển xe


class LineFollowingCar:

    NORMAL = 0
    SQUARE_LEFT = 1
    SQUARE_RIGHT = 2
    PRE_SQUARE_LEFT = 3
    PRE_SQUARE_RIGHT = 4
    ROUNDABOUT = 5
    PRE_SQUARE_LEFT_2 = 6
    PRE_SQUARE_RIGHT_2 = 7
    PRE_ROUNDABOUT = 8
    PRE_EXT_ROUNDABOUT = 10
    EXT_ROUNDABOUT = 11
    IN_ROUNDABOUT = 12

    def __init__(self):
        self.NUM_SENSORS = NB_GROUND_SENS
        self.MAX_SPEED = 46
        self.BASE_SPEED = 36.5

        # PID constants
        self.KP = 4.41
        self.KI = 0
        self.KD = 1.78

        # Sensor weights (outer sensors have more weight)
        self.WEIGHTS = [-9875, -7200, -3600, -1750, 1750, 3600, 7200, 9875]

        self.last_error = 0
        self.integral = 0

        self.state = self.NORMAL
        self.special = False

        self.count_white = 0

    def calculate_position(self, sensor_byte):
        weighted_sum = 0
        for i in range(self.NUM_SENSORS):
            if (sensor_byte & (1 << i)) == 0:  # If sensor detects white
                weighted_sum += self.WEIGHTS[i]
        return weighted_sum / 1000

    def turn_90_degrees(self, sensor_byte, direction="LEFT"):
        if direction == "LEFT":
            return -38, 37.5
        elif direction == "RIGHT":
            return 37.5, -38
        else:
            return 0, 0

    def pid_control(self, error):
        self.integral += error
        derivative = error - self.last_error
        output = self.KP * error + self.KI * self.integral + self.KD * derivative
        self.last_error = error
        return output

    def get_motor_speeds(self, sensor_byte):
        position = self.calculate_position(sensor_byte)
        correction = self.pid_control(position)

        left_speed = self.BASE_SPEED - correction
        right_speed = self.BASE_SPEED + correction

        # Ensure motor speeds are within the -MAX_SPEED to MAX_SPEED range
        left_speed = max(-self.MAX_SPEED, min(self.MAX_SPEED, left_speed))
        right_speed = max(-self.MAX_SPEED, min(self.MAX_SPEED, right_speed))

        print(position, correction, left_speed, right_speed)

        return left_speed, right_speed

    def state_decide(self, sensor_byte):
        if self.special:
            if self.state == self.SQUARE_LEFT:
                if (
                    sensor_byte == 0b00011111
                    or sensor_byte == 0b00111111
                    or sensor_byte == 0b01111111
                ):
                    self.state = self.NORMAL
                    self.special = False
                    return 0b11110000

            if self.state == self.SQUARE_RIGHT:
                if (
                    sensor_byte == 0b11111000
                    or sensor_byte == 0b11111100
                    or sensor_byte == 0b11111110
                ):
                    self.state = self.NORMAL
                    self.special = False
                    return 0b00001111

        if (
            self.state == self.SQUARE_LEFT
            and (sensor_byte ^ 0b11111111) & 0b00011000 != 0
            and not self.special
        ):
            self.state = self.NORMAL
            return 0b11110000

        if (
            self.state == self.SQUARE_RIGHT
            and (sensor_byte ^ 0b11111111) & 0b00011000 != 0
            and not self.special
        ):
            self.state = self.NORMAL
            return 0b00001111

        if self.state == self.PRE_SQUARE_LEFT or self.state == self.PRE_SQUARE_RIGHT:
            if self.special:
                if sensor_byte != 0b00000000:
                    if self.state == self.PRE_SQUARE_LEFT:
                        self.state = self.SQUARE_LEFT
                    else:
                        self.state = self.SQUARE_RIGHT
            else:
                if sensor_byte == 0b11111111:
                    if self.state == self.PRE_SQUARE_LEFT:
                        self.state = self.SQUARE_LEFT
                    else:
                        self.state = self.SQUARE_RIGHT
                elif (
                    sensor_byte == 0b11100111
                    or sensor_byte == 0b11110111
                    or sensor_byte == 0b11101111
                ):
                    self.state = (
                        self.PRE_SQUARE_LEFT_2
                        if self.state == self.PRE_SQUARE_LEFT
                        else self.PRE_SQUARE_RIGHT_2
                    )

        if (
            self.state == self.PRE_SQUARE_LEFT_2
            or self.state == self.PRE_SQUARE_RIGHT_2
        ):
            if sensor_byte == 0b00000000:
                if self.state == self.PRE_SQUARE_LEFT_2:
                    self.state = self.PRE_SQUARE_LEFT
                    self.special = True
                else:
                    self.state = self.PRE_SQUARE_RIGHT
                    self.special = True
            elif sensor_byte == 0b11111111:
                if self.state == self.PRE_SQUARE_LEFT_2:
                    self.state = self.SQUARE_LEFT
                else:
                    self.state = self.SQUARE_RIGHT

        if self.state == self.NORMAL:
            if sensor_byte == 0b11100000 or sensor_byte == 0b11110000:
                self.state = self.PRE_SQUARE_RIGHT
                return 0b11100111

            if sensor_byte == 0b0000111 or sensor_byte == 0b00001111:
                self.state = self.PRE_SQUARE_LEFT
                return 0b11100111

            if sensor_byte == 0b00000000:
                self.state = self.PRE_ROUNDABOUT
                return 0b11100111

        if self.state == self.PRE_ROUNDABOUT:
            if sensor_byte == 0b11111111:
                self.state = self.ROUNDABOUT
                return sensor_byte
                
        if self.state == self.IN_ROUNDABOUT:
            if (sensor_byte ^ 0b11111111) & 0b00000001:
                self.state = self.PRE_EXT_ROUNDABOUT
                return 0b11101111
            
        if self.state == self.EXT_ROUNDABOUT:
            if (sensor_byte ^ 0b11111111) & 0b11100000 == 0 and (sensor_byte ^ 0b11111111) & 0b00001111 != 0:
                self.state = self.NORMAL
            
        if self.state == self.PRE_EXT_ROUNDABOUT:
            if (sensor_byte ^ 0b11111111) & 0b00000011:
                self.state = self.EXT_ROUNDABOUT

        return sensor_byte

    def fuzzy_pid(self, sensor_byte):
        if sensor_byte != 0b00000000:
            self.count_white = 0
        self.count_white += sensor_byte == 0b00000000

        if self.count_white >= 42:
            return HALT, HALT

        sensor_byte = self.state_decide(sensor_byte)

        if self.state == self.SQUARE_LEFT:
            left_speed, right_speed = self.turn_90_degrees(sensor_byte, "LEFT")
            return left_speed, right_speed
        elif self.state == self.SQUARE_RIGHT:
            left_speed, right_speed = self.turn_90_degrees(sensor_byte, "RIGHT")
            return left_speed, right_speed
        elif self.state == self.EXT_ROUNDABOUT:
            return self.MAX_SPEED, -self.BASE_SPEED
        elif self.state == self.ROUNDABOUT:
            if sensor_byte == 0b11111111 or sensor_byte not in [
                0b11100111, 0b11101111, 0b11110111
            ]:
                return 36, -32.5
            self.state = self.IN_ROUNDABOUT
            return -30, 60
        else:
            if self.state in [
                self.PRE_SQUARE_LEFT,
                self.PRE_SQUARE_LEFT_2,
                self.PRE_SQUARE_RIGHT,
                self.PRE_SQUARE_RIGHT_2,
                self.PRE_ROUNDABOUT,
            ]:
                sensor_byte = 0b11100111
            left_speed, right_speed = self.get_motor_speeds(sensor_byte)
            return left_speed, right_speed


ir_car = LineFollowingCar()

# Main loop:
# Chương trình sẽ được lặp lại vô tận
while robot.step(timestep) != -1:

    filted = ReadSensors()
    # In ra màn hình giá trị của filted ở dạng nhị phân
    print("Position: " + str(format(filted, "08b")), sep="\t")

    lspeed, rspeed = ir_car.fuzzy_pid(sensor_byte=filted)
    if lspeed == HALT or rspeed == HALT:
        lm.setVelocity(0)
        rm.setVelocity(0)
        break
    lm.setVelocity(lspeed)
    rm.setVelocity(rspeed)
    print(lm.getVelocity(), rm.getVelocity())
    preFilted = filted

# # Enter here exit cleanup code.D
del ir_car
