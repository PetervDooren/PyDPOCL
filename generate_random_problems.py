import sys
import random
import json
from shapely import box, overlaps

def generate_problem(name):
    n_robots = 2
    n_objects = 5
    n_goals = 3
    min_obj_size = 0.1
    max_obj_size = 0.4
    buffer = 0.1 # goal areas should be this much larger than the object
    max_goal_size = 1
    table_width = 2.5
    table_length = 1.5

    # generate the reaches of the different robots
    robot_reach = []
    if n_robots == 2:
        width_overlap = random.uniform(max_obj_size+buffer, 1.0)
        x_overlap = (table_width-width_overlap)/2 # overlap in the middle of the table

        robot_reach = []
        # robot 1
        robot_reach.append([[0, 0],
                            [0, table_length],
                            [x_overlap+width_overlap, table_length],
                            [x_overlap+width_overlap, 0]])
        # robot 2
        robot_reach.append([[x_overlap, 0],
                            [x_overlap, table_length],
                            [table_width, table_length],
                            [table_width, 0]])
    else:
        raise ValueError(f"unsupported number of robots {n_robots}")
    
    # generate objects and their initial positions
    obj_geometry = []
    obj_list = []
    goal_list = []
    for i in range(n_objects):
        redo = True
        while redo:
            width = random.uniform(min_obj_size, max_obj_size)
            length = random.uniform(min_obj_size, max_obj_size)
            x_pos = random.uniform(0, table_width-width)
            y_pos = random.uniform(0, table_length-length)

            obj_box = box(x_pos, y_pos, x_pos+width, y_pos+length)
            for other_obj in obj_geometry:
                if overlaps(obj_box, other_obj):
                    continue
            # object does not overlap with another
            redo = False
        obj_geometry.append(obj_box)
        obj_list.append({
                "name": f"box_{i}",
                "initial_pose": [x_pos+0.5*width, y_pos+0.5*length],
                "width": width,
                "length": length
            })
        if i < n_goals:
            goal_width = random.uniform(width+buffer, max_goal_size)
            goal_length = random.uniform(length+buffer, max_goal_size)
            x_goal = random.uniform(0, table_width-goal_width)
            y_goal = random.uniform(0, table_length-goal_length)

            goal_list.append({"name": f"goal_{i}",
                              "coords": [
                                        [x_goal, y_goal],
                                        [x_goal, y_goal+goal_length],
                                        [x_goal+goal_width, y_goal+goal_length],
                                        [x_goal+goal_width, y_goal]
                                        ]
                            })

    # create pddl file
    robots_string = ""
    reach_string = ""
    for i in range(n_robots):
        robots_string = robots_string + f"robot_{i} "
        reach_string = reach_string + f"reach_robot_{i} "
    
    box_string = ""
    for i in range(n_objects):
        box_string = box_string + f"box_{i} "
    
    goal_string = ""    
    goal_area_string = ""
    for i in range(n_goals):
        goal_area_string = goal_area_string + f"goal_{i} "
        goal_string = goal_string + f"              (within box_{i} goal_{i})\n"

    pddl_content = f"""
(define (problem test-{i}-problem)
  (:domain manipulation)
  (:objects {robots_string} - robot
            {box_string} - physical_item
            table - area
            {goal_area_string} - area
            {reach_string} - area
            )
  (:init )
  (:goal (and
{goal_string}              )))
    """
    filepath = name + "_problem.pddl"
    with open(filepath, "w") as f:
        f.write(pddl_content)

    # create worldmodel file
    worldmodel_dict = {}
    worldmodel_dict["domain"] = "manipulation-domain"
    worldmodel_dict["robots"] = []
    for i in range(n_robots):
        worldmodel_dict["robots"].append({"name": f"robot_{i}",
                                          "reach": f"reach_robot_{i}"})
    worldmodel_dict["base_area"] = "table"
    worldmodel_dict["areas"] = []
    worldmodel_dict["areas"].append({"name": "table",
                                     "coords": [
                                            [0, 0],
                                            [0, table_length],
                                            [table_width, table_length],
                                            [table_width, 0]
                                        ]
                                    })
    for i in range(n_robots):
        worldmodel_dict["areas"].append({"name": f"reach_robot_{i}",
                                     "coords": robot_reach[i]
                                    })
    for i in range(n_goals):
        worldmodel_dict["areas"].append(goal_list[i])

    worldmodel_dict["objects"] = obj_list

    filepath = name+"_worldmodel.json"
    with open(filepath, "w") as f:
        json.dump(worldmodel_dict, f, indent=2)


if __name__ == '__main__':
    num_args = len(sys.argv)
    if num_args >1:
        n = sys.argv[1]
    else:
        n = 10
    for i in range(n):
        problem_path = "domains/manipulation-domain-batch/"
        problem_name = problem_path + "test_" + str(i)
        generate_problem(problem_name)