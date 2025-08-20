import os
from collections import namedtuple
import csv
from PyPOCL.PyDPOCL import POCLPlanner
from PyPOCL.worldmodel import load_domain_and_problem
from PyPOCL.plan_utility import check_plan, plan_to_json, visualize_plan, plan_to_dot

Problem = namedtuple("Problem", ["problem", "worldmodel"])

LOG = 0

if __name__ == '__main__':
    batch_dir = "domains/manipulation-domain-batch/"

    problem_tuples = {}
    for file in os.scandir(batch_dir):
        if file.is_file():
            testname = file.name[0:7] # should be like "test_0"
            if testname[-1] == '_':
                testname = testname[0:6]
            if testname in problem_tuples: # problem is already found
                continue
            problem_file = os.path.join(batch_dir, testname+"_problem.pddl")
            worldmodel_file = os.path.join(batch_dir, testname+"_worldmodel.json")
            if not os.path.isfile(problem_file):
                raise ValueError(f"file {problem_file} does not exist")
            if not os.path.isfile(worldmodel_file):
                raise ValueError(f"file {worldmodel_file} does not exist")
            problem_tuples[testname] = Problem(problem_file, worldmodel_file)

    domain_file = 'domains/manipulation-domain/manipulation-domain.pddl'
   
    iteration = 0
    success_count = 0
    faulty_plan_count = 0
    plan_not_found_count = 0
    error_count = 0

    # Prepare CSV file
    csv_filename = "test_results.csv"
    with open(csv_filename, mode="w", newline="") as csvfile:
        fieldnames = ["iteration", "testname", "status", "plan_file", "planning_time", "expanded", "visited", "terminated", "plans_found", "nr_objects", "nr_goals"]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for testname, problem in problem_tuples.items():
            print(f"\n\niteration: {iteration}. running test: {testname}\n\n")

            domain, problem_obj = load_domain_and_problem(domain_file, problem.problem, problem.worldmodel)

            nr_objects = len([o for o in problem_obj.objects if o.typ=='physical_item'])
            nr_goals = len(problem_obj.goal.preconds)

            plangraph_name = f"plans/{domain.name}/{problem_obj.name}-plangraph"

            planner = POCLPlanner(domain, problem_obj, LOG, plangraph_name=plangraph_name)
            try:
                plans, planning_report = planner.solve(k=1, cutoff=180)
            except:
                print("error during execution")
                error_count += 1
                writer.writerow({
                    "iteration": iteration,
                    "testname": testname,
                    "status": "error",
                    "plan_file": "",
                    "planning_time": 0,
                    "expanded": 0,
                    "visited": 0,
                    "terminated": 0,
                    "plans_found": 0,
                    "nr_objects": nr_objects,
                    "nr_goals": nr_goals,
                })
                iteration += 1
                continue
        
            if len(plans) == 0:
                print("no plans could be found")
                plan_not_found_count += 1
                writer.writerow({
                    "iteration": iteration,
                    "testname": testname,
                    "status": "no_plan_found",
                    "plan_file": "",
                    "planning_time": planning_report.planning_time,
                    "expanded": planning_report.expanded,
                    "visited": planning_report.visited,
                    "terminated": planning_report.terminated,
                    "plans_found": planning_report.plans_found,
                    "nr_objects": nr_objects,
                    "nr_goals": nr_goals,
                })
            for i in range(len(plans)):
                plan = plans[i]
                if not check_plan(plan):
                    print("Error: plan is not valid")
                    faulty_plan_count += 1
                    status = "faulty_plan"
                else:
                    print("Plan is valid")
                    success_count += 1
                    status = "success"
                plan_path = f"plans/{domain.name}/{problem_obj.name}-plan_{i}"
                plan_to_json(plan, f"{plan_path}.json")
                plan_to_dot(plan, f"{plan_path}.dot", f"{plan_path}.svg", show=False)
                visualize_plan(plan, show=False, filepath=f"{plan_path}.png")
                writer.writerow({
                    "iteration": iteration,
                    "testname": testname,
                    "status": status,
                    "plan_file": f"{plan_path}.json",
                    "planning_time": planning_report.planning_time,
                    "expanded": planning_report.expanded,
                    "visited": planning_report.visited,
                    "terminated": planning_report.terminated,
                    "plans_found": planning_report.plans_found,
                    "nr_objects": nr_objects,
                    "nr_goals": nr_goals,
                })
                iteration += 1
    print(f"Done running. ran {iteration} iterations")
    print(f"{success_count} successfull.")
    print(f"{faulty_plan_count} faulty plans found.")
    print(f"{plan_not_found_count} plan not found.")
    print(f"{error_count} errors.")