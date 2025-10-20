import os
import sys
from collections import namedtuple
import csv
from PyPOCL.PyDPOCL import POCLPlanner
from PyPOCL.worldmodel import load_domain_and_problem
from PyPOCL.plan_utility import check_plan, plan_to_json, visualize_plan, plan_to_dot

Problem = namedtuple("Problem", ["problem", "worldmodel"])

LOG = 0

if __name__ == '__main__':
    num_args = len(sys.argv)
    if num_args == 2:
        test_i = sys.argv[1]
        domain_file = 'domains/manipulation-domain/manipulation-domain.pddl'
        problem_file = f'domains/manipulation-domain-batch/test_{test_i}_problem.pddl'
        worldmodel_file = f'domains/manipulation-domain-batch/test_{test_i}_worldmodel.json'
    elif num_args > 2:
        domain_file = sys.argv[1]
        problem_file = sys.argv[2]
        if num_args > 3:
            worldmodel_file = sys.argv[2]
    else:
        test_i = 84
        domain_file = 'domains/manipulation-domain/manipulation-domain.pddl'
        problem_file = f'domains/manipulation-domain-batch/test_{test_i}_problem.pddl'
        worldmodel_file = f'domains/manipulation-domain-batch/test_{test_i}_worldmodel.json'

        # benchmarks
        #domain_file = 'tests/benchmarks/manipulation-domain-geometric-threats/domain.pddl'
        #problem_file = 'tests/benchmarks/manipulation-domain-geometric-threats/problem.pddl'
        #worldmodel_file = 'tests/benchmarks/manipulation-domain-geometric-threats/worldmodel.json'

    nr_iterations = 10
    cutoff_time = 60 # in seconds

    iteration = 0
    success_count = 0
    faulty_plan_count = 0
    plan_not_found_count = 0
    error_count = 0

    domain, problem = load_domain_and_problem(domain_file, problem_file, worldmodel_file)
    testname = problem.name
    # Prepare CSV file
    csv_filename = f"test_results/repeat_{testname}.csv"
    with open(csv_filename, mode="w", newline="") as csvfile:
        fieldnames = ["iteration", "testname", "status", "plan_file", "planning_time", "expanded", "visited", "terminated", "plans_found", "assumption_failed", "nr_objects", "nr_goals"]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for iteration in range(nr_iterations):
            print(f"\n\niteration: {iteration}. running test: {testname}\n\n")

            domain, problem = load_domain_and_problem(domain_file, problem_file, worldmodel_file)

            nr_objects = len([o for o in problem.objects if o.typ=='physical_item'])
            nr_goals = len(problem.goal.preconds)

            planner = POCLPlanner(domain, problem, LOG)
            try:
                plans, planning_report = planner.solve(k=1, cutoff=cutoff_time)
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
                    "assumption_failed": 0,
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
                    "assumption_failed": planning_report.assumption_failed,
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
                plan_path = f"plans/{domain.name}/{problem.name}-plan_{i}"
                #plan_to_json(plan, f"{plan_path}.json")
                #plan_to_dot(plan, f"{plan_path}.dot", f"{plan_path}.svg", show=False)
                #visualize_plan(plan, show=False, filepath=f"{plan_path}.png")
                writer.writerow({
                    "iteration": iteration,
                    "testname": testname,
                    "status": status,
                    "plan_file": "",
                    "planning_time": planning_report.planning_time,
                    "expanded": planning_report.expanded,
                    "visited": planning_report.visited,
                    "terminated": planning_report.terminated,
                    "plans_found": planning_report.plans_found,
                    "assumption_failed": planning_report.assumption_failed,
                    "nr_objects": nr_objects,
                    "nr_goals": nr_goals,
                })
                iteration += 1
    print(f"Done running. ran {iteration} iterations")
    print(f"{success_count} successfull.")
    print(f"{faulty_plan_count} faulty plans found.")
    print(f"{plan_not_found_count} plan not found.")
    print(f"{error_count} errors.")