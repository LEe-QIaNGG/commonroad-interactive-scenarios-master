To create scenario with vehicles (generate_vehicle.py):

vehicles_config = {
    "CAR": 1,
    "TRUCK": 1,
}
random_seed = 3
TST= scenario_with_vehicles(scenario,random_seed=random_seed,vehicles_config=vehicles_config)
TST_scenario = TST.generate_vehicle()

To initialize blank scenario with planning problems and trajectories (batch_planning_problem_generator.py):

python generation/batch_planning_problem_generator.py --input interactive_scenarios/CHN_TST-2_1_T-1/CHN_TST-2_1_T-1.cr.xml --seed 23 --max-attempts 80 --overwrite
执行后将得到：
规划问题：.../CHN_TST-2_1_T-22.cr.xml
配套轨迹：.../CHN_TST-2_1_T-22_solution.xml