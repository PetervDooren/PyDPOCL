import unittest
import json
from PyPOCL.GPlan import GPlan

class DummyStep:
    def __init__(self, ID, name):
        self.ID = ID
        self.name = name
        self.Args = []
        self.preconds = []
        self.effects = []

class TestGPlanJson(unittest.TestCase):
    def test_from_json(self):
        # Example JSON structure matching what to_json would produce
        plan_json = {
            "ID": "1234",
            "name": "testplan",
            "cost": 1,
            "heuristic": 0.5,
            "depth": 0,
            "init_state": {"ID": "init", "schema": "dummy_init", "Args": [], "preconds": [], "effects": [], "stepnum": 0, "height": 0},
            "goal_state": {"ID": "goal", "schema": "dummy_goal","Args": [], "preconds": [], "effects": [], "stepnum": 1, "height": 0},
            "steps": [
                {"ID": "init", "schema": "dummy_init", "Args": [], "preconds": [], "effects": [], "stepnum": 0, "height": 0},
                {"ID": "goal", "schema": "dummy_goal", "Args": [], "preconds": [], "effects": [], "stepnum": 1, "height": 0},
                {"ID": "1", "schema": "schema1", "Args": [], "preconds": [], "effects": [], "stepnum": 0, "height": 0},
                {"ID": "2", "schema": "schema2","Args": [], "preconds": [], "effects": [], "stepnum": 1, "height": 0}
            ],
            "orderings": [],
            "causal_links": [],
            "variableBindings": {}
        }
        # Write to a temp file
        with open("temp_plan.json", "w") as f:
            json.dump(plan_json, f)
        
        # Assume GPlan.from_json exists and loads from file
        plan = GPlan.from_json("temp_plan.json")
        self.assertEqual(plan.name, "testplan")
        self.assertEqual(plan.cost, 1)
        self.assertEqual(plan.heuristic, 0.5)
        self.assertEqual(plan.depth, 0)
        self.assertEqual(len(plan.steps), 4)
        self.assertEqual(plan.steps[0].schema, "dummy_init")
        self.assertEqual(plan.steps[1].schema, "dummy_goal")
        self.assertEqual(plan.steps[2].schema, "schema1")
        self.assertEqual(plan.steps[3].schema, "schema2")

if __name__ == '__main__':
    unittest.main()