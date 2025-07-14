import unittest

from shapely import box
from PyPOCL.Ground_Compiler_Library.pathPlanner import check_is_connected

class TestPathPlanner(unittest.TestCase):
    def test_path_exists(self):
        reach_area = box(0, 0, 5, 1)
        object_length = 0.4
        object_width = 0.2
        start_area = box(0, 0, 0.2, 0.4)
        goal_area = box(4.5, 0.4, 4.7, 0.8)
        obst_area1 = box(1, 0, 2, 0.5)
        obst_area2 = box(3, 0.5, 4, 1)
        obst_areas = [obst_area1, obst_area2]

        self.assertTrue(check_is_connected(start_area, goal_area, reach_area, obst_areas, object_width, object_length), "connected path is found to not be connected")

    def test_path_does_not_exist(self):
        reach_area = box(0, 0, 5, 1)
        object_length = 0.4
        object_width = 0.2
        start_area = box(0, 0, 0.2, 0.4)
        goal_area = box(4.5, 0.4, 4.7, 0.8)
        obst_area1 = box(1, 0, 2, 0.5)
        obst_area2 = box(2, 0.5, 3, 1)
        obst_areas = [obst_area1, obst_area2]

        self.assertFalse(check_is_connected(start_area, goal_area, reach_area, obst_areas, object_width, object_length), "unconnected path is found to be connected")

    def test_path_does_not_exist_small_gap(self):
        reach_area = box(0, 0, 5, 1)
        object_length = 0.4
        object_width = 0.2
        start_area = box(0, 0, 0.2, 0.4)
        goal_area = box(4.5, 0.4, 4.7, 0.8)
        obst_area1 = box(2, 0, 3, 0.45) # obstacles leave a gap of 0.1m
        obst_area2 = box(2, 0.55, 3, 1)
        obst_areas = [obst_area1, obst_area2]

        self.assertFalse(check_is_connected(start_area, goal_area, reach_area, obst_areas, object_width, object_length), "Path connected by a too narrow gap is found to be connected")

    def test_path_does_exist_small_gap(self):
        reach_area = box(0, 0, 5, 1)
        object_length = 0.4
        object_width = 0.2
        start_area = box(0, 0, 0.2, 0.4)
        goal_area = box(4.5, 0.4, 4.7, 0.8)
        obst_area1 = box(2, 0, 3, 0.35) # obstacles leave a gap of 0.3m
        obst_area2 = box(2, 0.65, 3, 1)
        obst_areas = [obst_area1, obst_area2]

        self.assertTrue(check_is_connected(start_area, goal_area, reach_area, obst_areas, object_width, object_length), "Path connected by a sufficiently large gap is found to not be connected")


        
if __name__ == '__main__':
    unittest.main()