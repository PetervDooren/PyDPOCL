import sys
from PyPOCL.GPlan import GPlan

if __name__ == '__main__':
    num_args = len(sys.argv)
    if num_args >1:
        json_file = sys.argv[1]
    else:
        json_file = 'plan_1.json'

    # Load the plan from the JSON file
    plan = GPlan.from_json(json_file)
    plan.check_plan()
