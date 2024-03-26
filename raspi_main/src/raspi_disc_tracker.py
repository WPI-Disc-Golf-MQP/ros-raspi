# the purpose of this file is to store the information about where discs are within the system 

from disc_record import disc_record

class raspi_disc_tracker():
    def __init__(self, top_conveyor_publisher, main_conveyor_publisher):

        # these functions should be called any time a value is updated, so that it can be viewed by other nodes (mainly the UI node)
        self.top_conveyor_publisher = top_conveyor_publisher
        self.main_conveyor_publisher = main_conveyor_publisher

        self.top_conveyor = [0,0] # this represents the number of discs between each of the blocks. For now, there is one value because there is one block 
        self.measure_modules = {
            "Intake":False,
            "Camera":False,
            "CV":False,
            "Flex":False,
            "Scale":False,
            "Outtake":False,
        }

    # this function updates where discs are within the machine currently 
    def update_measure_modules(self, key:str, value:bool):
        if key in self.measure_modules.keys():
            self.measure_modules[key] = value
            self.main_conveyor_publisher.publish(self.measure_modules_stringified())
        else: 
            print("pass correct key to the raspi_disc_counts class")

    # this function is used to show a value about each disc 
    def measure_modules_stringified(self):
        new = self.measure_modules
        for key in self.measure_modules.keys():
            if type(self.measure_modules[key]) == disc_record:
                new[key] = self.measure_modules[key].sku
        return str(new)

    # this function shifts the values "down the machine" and appends a value (0 or 1) to the "front" of the machine  
    def move_all_measures_over(self, add_new_value:disc_record):
        previous_value = add_new_value
        for key in self.measure_modules.keys():
            current_value = self.measure_modules[key]

            self.update_measure_modules(key, previous_value)
            # self.measure_modules[key] = previous_value ## this does the same thing #TODO: determine if I should use this line instead, and then call the update publisher in this function rather than update_measure_modules

            previous_value = current_value 



    # these are CRUD functions for the top conveyor (Create, Read, Update, Delete)
    def update_top_conveyor(self,value): 
        self.top_conveyor[0] = value
        self.top_conveyor_publisher.publish(str(self.top_conveyor))
    
    def increase_top_conveyor(self):
        self.update_top_conveyor(self.top_conveyor[0]+1)

    def decrease_top_conveyor(self):
        self.update_top_conveyor(self.top_conveyor[0]-1)



if __name__ == "__main__": 
    class dummy_publisher(): 
        def __init__(self):
            pass
        def publish(self):
            return -1

    rdt = raspi_disc_tracker(dummy_publisher(), dummy_publisher())

    # rdt.update_measure_modules("Intake",1)
    # print(rdt.measure_modules)

    # print(adisc := disc_record())
    # adisc.diameter = 10
    # print(adisc)

    rdt.move_all_measures_over(disc_record(sku="PLACEHOLDER SKU"))
    print(str(rdt.measure_modules_stringified()))

    # rdt.increase_top_conveyor()
    # rdt.increase_top_conveyor()
    # print(rdt.top_conveyor)


# # http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
# pub = rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10)
# pub.publish(std_msgs.msg.String("foo"))
