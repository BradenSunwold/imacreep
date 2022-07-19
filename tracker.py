import math


class EuclideanDistTracker:
    def __init__(self):
        # Store the center positions of the objects
        self.center_points = {}
        # Keep the count of the IDs
        # each time a new object id detected, the count will increase by one
        self.id_count = 0


    def update(self, objects_rect):
        # hysteresis counter init (hystCnt = hystCnt + 2
        hystCnt = 1
        
        # Objects boxes and ids
        objects_bbs_ids = []

        # Get center point of new object
        for rect in objects_rect:
            x, y, w, h = rect
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2

            # Find out if that object was detected already
            same_object_detected = False
            for id, data in self.center_points.items():
                dist = math.hypot(cx - data[0], cy - data[1])
                currHystCnt = data[2]
                
                validObject = False
                if dist < 25:
                    # Check if object has been detected at least three times before "valid"
                    if currHystCnt > hystCnt:
                        validObject = True
                        currHystCnt = hystCnt
                
                    objects_bbs_ids.append([x, y, w, h, id])
                    same_object_detected = True
                    currHystCnt += 1
                    self.center_points[id] = (cx, cy, currHystCnt)
                    break

            # New object is detected we assign the ID to that object
            if same_object_detected is False:
                objects_bbs_ids.append([x, y, w, h, self.id_count])
                self.center_points[self.id_count] = (cx, cy, 1)
                self.id_count += 1

#        # Clean the dictionary by center points to remove IDS not used anymore
        new_center_points = {}
#        for obj_bb_id in objects_bbs_ids:
#            _, _, _, _, object_id = obj_bb_id
#            center = self.center_points[object_id]
#            new_center_points[object_id] = center

        # Loop through center points
        # Check if there are any center_point entries that are not in the new bbs_ids array
        # If an entry is not in, decrement hysCnt
        # Save any center_point with a hystCnt greater than 0
        # Return lowest center_Point ID with valid hystCnt (< 1)
        for id, currHystCnt in self.center_points.items():
            found = False
            center = []
            for obj_bb_id in objects_bbs_ids:
                _, _, _, _, object_id = obj_bb_id
                if object_id == id:
                    found = True
                    center = self.center_points[id]
                    break
            if found == False:
                self.center_points[id][2] -= 1
                center = self.center_points[id]

            if self.center_points[id][2] > 0:
                new_center_points[id] = [center[0], center[1], self.center_points[id][2]]

        # Update dictionary with IDs not used removed
        self.center_points = new_center_points.copy()

        # Find the lowest ID that is currently "active" in frame
        for id, currHystCnt in self.center_points.items():
            if self.center_points[id][2] > hystCnt:
                print(self.center_points[id])
                return(self.center_points[id])
                break

        return [0, 0, 0]



