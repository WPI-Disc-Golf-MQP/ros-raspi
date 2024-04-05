# the purpose of this file is to store the information about where discs are within the system 

from disc_record import disc_record, location
import random

class raspi_disc_tracker():
    def __init__(self, top_conveyor_publisher, main_conveyor_publisher):

        # these functions should be called any time a value is updated, so that it can be viewed by other nodes (mainly the UI node)
        self.top_conveyor_publisher = top_conveyor_publisher
        self.main_conveyor_publisher = main_conveyor_publisher

        self.discs = []

    # Function to filter disc records with loc = Location.TOP_CONVEYOR
    def _filter_top_conveyor_records(self):
        return [record for record in self.discs if record.loc == location.TOP_CONVEYOR]

    def _filter_non_top_conveyor_records(self):
        discs_excluding_top_conveyor = [obj for obj in self.discs if obj.loc != location.TOP_CONVEYOR]
        discs_excluding_top_conveyor.sort(key=lambda obj: obj.loc.value)
        # sorted_discs = sorted(discs_excluding_top_conveyor, key=attrgetter('loc'))
        return discs_excluding_top_conveyor
        # TODO: UNTESTED
    
    def count_top_conveyor_discs(self): 
        # count = sum(1 for obj in self.discs if obj.loc == location.TOP_CONVEYOR)
        # return count
        return len(self._filter_top_conveyor_records())

    # --- get the column names for the table of discs 
    def get_field_names(self):
        record = disc_record() # dummy disc to get the field names 
        columns = [str(field.name) for field in record.__dataclass_fields__.values()]
        return '|'.join(columns)

    # --- format each disc, and get stringified version to be displayed on the GUI 
    def _format_disc_record(self, record):
        attributes = [str(getattr(record, field.name)) for field in record.__dataclass_fields__.values()]
        return '|'.join(attributes)

    def _format_disc_records(self, records):
        return '\n'.join(self._format_disc_record(record) for record in records)
    
    def get_stringified_non_top_conveyor_records(self): 
        return self._format_disc_records(self._filter_non_top_conveyor_records())

    # --- IO functions for adding and removing discs, and moving them over modules 

    # this adds a disc to the back of the queue (farthest from being dropped to intake)
    def new_disc(self, manufacturer_sku = "(No SKU)"):
        disc = disc_record(manufacturer_sku, random.randint(10000,99999), top_conveyor__sub_loc=max(self.highest_top_conveyor_sub_loc()+1,0))
        self.discs.append(disc) 
        # TODO: make the handle an actual unique number based on MAPLE HILL Inventory System, rather than a random number 
        # TODO: write a function that writes to the front of the queue and let them choose with 2 buttons
        # TODO: add a manufacturer_sku input field in the GUI 

    def remove_last_disc(self): 
        # this function removes the disc that is furthest from being dropped
    
        for item in self.discs:
            if item.top_conveyor__sub_loc == self.highest_top_conveyor_sub_loc():
                a = item

        # print("A IS HERE")
        # print(a)
        self.discs.remove(a)

        # TODO: allow user to choose removing the closest or furthest 

    # Function to get the highest number of the top_conveyor_sub_loc
    def highest_top_conveyor_sub_loc(self):
        top_conveyor__sub_locs = [record.top_conveyor__sub_loc for record in self.discs]
        if top_conveyor__sub_locs:
            return max(top_conveyor__sub_locs)
        else:
            return -1  # Or any default value indicating no records found


    def move_all_measures_over(self): 
        for disc in self.discs: 
            if disc.loc.value == location.TOP_CONVEYOR.value: 
                
                if disc.top_conveyor__sub_loc == 0: 
                    # print(disc.loc)
                    disc.loc = location.INTAKE # moved into the intake 
                    # print(disc.loc)
                    disc.top_conveyor__sub_loc = - 1 # reset this value to null value
                else: 
                    # print(disc.top_conveyor__sub_loc)
                    disc.top_conveyor__sub_loc = disc.top_conveyor__sub_loc - 1 # moving closer to the intake to drop

            elif disc.loc.value in [location.INTAKE.value,
                            location.MAIN_CONVAYOR__TURNTABLE.value,
                            location.MAIN_CONVAYOR__CV.value,
                            location.MAIN_CONVAYOR__FLEXIBILITY.value,
                            location.MAIN_CONVAYOR__SCALE.value,
                            location.OUTTAKE.value]:
                disc.loc = location(disc.loc.value + 1) # incriment by 1, so that it is in the next state
                
            elif disc.loc.value == location.BOXES.value: 
                pass # still in the boxes, unless you press a button to store it 
                # TODO: store some kind of sub location, like the slot in the boxes, so that it can be found later much easier 
        
        # after all discs are moved, we can publish some new states
        self.top_conveyor_publisher.publish(str(self.count_top_conveyor_discs()))
        column_names_included = self.get_field_names() + "\n" + self.get_stringified_non_top_conveyor_records()
        self.main_conveyor_publisher.publish(column_names_included)


if __name__ == "__main__": 
    class dummy_publisher(): 
        def __init__(self):
            pass
        def publish(self,value):
            print(value)
            return -1

    rdt = raspi_disc_tracker(dummy_publisher(), dummy_publisher())

    rdt.new_disc()
    rdt.new_disc()
    rdt.new_disc()
    rdt.new_disc()
    # print(len(rdt._filter_top_conveyor_records()))
    rdt.move_all_measures_over()
    # print(len(rdt._filter_top_conveyor_records()))
    # rdt.move_all_measures_over()
    # print(len(rdt._filter_top_conveyor_records()))
    # rdt.move_all_measures_over()
    # print(len(rdt._filter_top_conveyor_records()))
    # rdt.move_all_measures_over()
    # print(len(rdt._filter_top_conveyor_records()))

    # stringified = rdt.get_field_names()
    # print(stringified)

    # stringified = rdt.get_stringified_non_top_conveyor_records()
    # print(stringified)


    # print(rdt.highest_top_conveyor_sub_loc())
    # print(len(rdt.discs))
    # print(rdt.remove_last_disc())
    # print(len(rdt.discs))
