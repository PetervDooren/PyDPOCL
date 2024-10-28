
from PyDPOCL import *

if __name__ == '__main__':

	# domain_file = 'Ground_Compiler_Library//domains/travel_domain_primitive_only.pddl'
	domain_file = 'Ground_Compiler_Library//domains/travel_domain.pddl'

	# Problem files
	# 1] 1 agent, 1 car, 1 airplane, 2 locations
	problem_file_1 = 'Ground_Compiler_Library//domains/travel-to-la.pddl'
	
	problems = [problem_file_1]
	# problems = [problem_file_8]

	d_name = domain_file.split('/')[-1].split('.')[0]

	# for each problem, solve in 1 of 4 ways... but need way to run in different ways

	for prob in problems:
		p_name = prob.split('/')[-1].split('.')[0]
		uploadable_ground_step_library_name = 'Ground_Compiler_Library//' + d_name + '.' + p_name

		RELOAD = 1
		if RELOAD:
			print('reloading')
			ground_steps = just_compile(domain_file, prob, uploadable_ground_step_library_name)

		ground_steps = []
		i = 0
		while True:
			try:
				# print(i)
				with open(uploadable_ground_step_library_name + str(i), 'rb') as ugly:
					ground_steps.append(pickle.load(ugly))
				i += 1
			except:
				break
		print('finished uploading')

		print(p_name)

		planner = GPlanner(ground_steps)
		planner.solve(k=40, cutoff=1200)
		# planner.solve(k=40, cutoff=10)