from bluedot.btcomm import BluetoothClient
from time import sleep
from .Kinematics import Kinematics


class PiClient:
    def __init__(self, link1, link2, range1, range2, publisher):
        self.link1, self.link2 = link1, link2
        self.range1, self.range2 = range1, range2
        self.max_reach = link1 + link2
        self.km = Kinematics(link1, link2, range1, range2)
        self.pos_x, self.pos_y = 372, 0
        self.pos_z = 400 
        self.pos_gripper = 300
        self.ax12_range = 300
        self.offset_range1 = ((self.ax12_range - (self.range1*2)) / 2)
        self.offset_range2 = ((self.ax12_range - (self.range2*2)) / 2)
        self.client = None
        self.publisher = publisher
        self.gripper_servo_id = 41
        self.gripper_open = True
        self.max_pos_multiplier = 1.5
        

    def command(self, angle1, angle2):
        # Arm bottom id 1, arm top id 2
        print("sending1")
        self.publisher.send_command(1, self.convert_unit(angle1))
        print("sending2")
        self.publisher.send_command(2, self.convert_unit(angle2))

    def convert_unit(self, degrees):
        return round((1023 / self.ax12_range) * degrees)

    def handle_input(self, input):
        target_x = self.pos_x
        target_y = self.pos_y
        target_z = self.pos_z
        target_gripper = self.pos_gripper
        gripper_open = self.gripper_open
        move = False
        move_gripper = False
        move_z = False
        

        match input:
            case "forward":
                if target_x < (self.ax12_range*self.max_pos_multiplier):
                    target_x += 10
                    move = True
            case "backward":
                if target_x > -(self.ax12_range*self.max_pos_multiplier):
                    target_x -= 10
                    move = True
            case "left":
                if target_y < (self.ax12_range*self.max_pos_multiplier):
                    target_y += 10
                    move = True
            case "right":
                if target_y > -(self.ax12_range*self.max_pos_multiplier):
                    target_y -= 10
                    move = True
            case "up":
                if target_z < 1000:
                    target_z += 20 
                    move_z = True
                #print(input)
            case "down":
                if target_z > 20:
                    target_z -= 20
                    move_z = True

                #print(input)
            case "Grijpen":
                move_gripper = True
                print(input)
            #case "open":
                #while True:
                #    try:
                #        load = self.publisher.get_data(gripper_servo_id,40,2)
                #        if load > 1400 and load < 2048:
                #            break
                #        else:
                #           target_gripper += 10
                #           self.publisher.send_command(gripper_servo_id,target_gripper)
                #    except Exception as e:
                #        print("Error open: ", e)

            #case "close":
                #while True:
                #    try:
                #        load = self.publisher.get_data(gripper_servo_id,40,2)
                #        if load > 400 and load < 1023:
                #            break
                #        else:
                #            target_gripper -= 10
                #            self.publisher.send_command(gripper_servo_id,target_gripper)
                #    except Exception as e:
                #        print("Error close: ", e)

            case "init":  # move to start position
                move = True
                move_z = True
                #move_gripper = True
            case _:
                print("Invalid input: ", input)


        if move_z == True:
            self.publisher.send_command(5,target_z)
            self.pos_z = target_z
            move_z = False

        if move_gripper == True:

                if gripper_open == False:
                    print("gripper is opening")
                    while 0 <= target_gripper <= 420:
                        try:
                            load = self.publisher.get_data(self.gripper_servo_id,40,2)
                            if load > 1600 and load < 2048:
                                print("gripper is open")
                                gripper_open = True 
                                self.gripper_open = gripper_open
                                break
                            elif target_gripper >= 400:
                                print("gripper is open")
                                gripper_open = True
                                self.gripper_open = gripper_open
                                break
                            else:
                                target_gripper += 20
                                self.publisher.send_command(self.gripper_servo_id,target_gripper)
                        except Exception as e:
                            print("Error open: ", e)

                elif gripper_open == True:
                    print("gripper is closing")
                    while 0 <= target_gripper <= 420:
                        try:
                            load = self.publisher.get_data(self.gripper_servo_id,40,2)
                            if load > 400 and load < 1023:
                                print("gripper closed")
                                gripper_open = False
                                self.gripper_open = gripper_open
                                break
                            else:
                                target_gripper -= 20
                                self.publisher.send_command(self.gripper_servo_id,target_gripper)
                        except Exception as e:
                            print("Error close: ", e)

                move_gripper=False

        if self.is_within_reach(target_x, target_y) and move is True:
            self.pos_x, self.pos_y = target_x, target_y
            angle1, angle2 = self.km.inverse_kinematics(target_x, target_y)
            #print("angle1:",angle1,"range1", self.range1,"offset range1", self.offset_range1,"angle2",angle2,"range2", self.range2,"offset range2", self.offset_range2)
            #angle1 += self.range1 + self.offset_range1
            #angle2 += self.range2 + self.offset_range2
            #print("angles1,2 after ofsett and range",angle1, angle2)
            self.command(angle1, angle2)
        elif move is True:
            #print("we cant reach the coordinate but will try to get as close as possible")
            self.pos_x, self.pos_y = target_x, target_y
            angle1, angle2 = self.km.inverse_kinematics(target_x, target_y)
            #print("angle1:",angle1,"range1", self.range1,"offset range1", self.offset_range1,"angle1",angle1,"range2", self.range2,"offset range2", self.offset_range2)
            #angle1 += self.range1 + self.offset_range1
            #angle2 += self.range2 + self.offset_range2
            #print("angles1,2",angle1, angle2)
            self.command(angle1, angle2)


    def data_received(self, data):
        if data is not None and isinstance(data, str):
            self.handle_input(data)

    def is_within_reach(self, target_x, target_y):
        distance = (target_x**2 + target_y**2)**0.5
        
        return 50 < distance <= self.max_reach

    def start(self):
        while True:
            if self.client is None:
                print("Connecting...")
                mac_address = "D4:8A:FC:A4:AF:06"
                self.client = BluetoothClient(mac_address, self.data_received)
                #self.client = BluetoothClient("D4:8A:FC:A4:AF:06", self.data_received)
                if self.client.connected:
                    print("Connected")
                    self.handle_input("init")
                    break
                else:
                    sleep(1000)