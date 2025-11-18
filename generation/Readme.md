To create scenario with vehicles (generate_vehicle.py):

vehicles_config = {
    "CAR": 1,
    "TRUCK": 1,
}
random_seed = 3
TST= scenario_with_vehicles(scenario,random_seed=random_seed,vehicles_config=vehicles_config)
TST_scenario = TST.generate_vehicle()

To initialize blank scenario with planning problems and trajectories (batch_planning_problem_generator.py):

python generation/initialize_scenario.py --input interactive_scenarios/CHN_TST-2_1_T-1/CHN_TST-2_1_T-1.cr.xml --seed 40 --num-vehicle 4 --num-ego 1 --max-attempts 80 --overwrite

执行后将得到：
规划问题：.../CHN_TST-2_1_T-23.cr.xml
配套轨迹：.../CHN_TST-2_1_T-23_solution.xml


To generate SUMO configuration files from CommonRoad scenario (scenario_config_generator.py):

python generation/sumo_config_generator.py --scenario_name CHN_TST-2_1_T-40 --simulation_steps 500 --step_length 0.1

执行后将得到：
SUMO配置文件：.../CHN_TST-2_1_T-23.sumo.cfg
SUMO网络文件：.../CHN_TST-2_1_T-23.net.xml
SUMO车辆路由文件：.../CHN_TST-2_1_T-23_solution.rou.xml


To run the whole pipeline once (initialize scenario + generate SUMO configs):

./generation/run_full_pipeline.sh \
    --input interactive_scenarios/CHN_TST-2_1_T-1/CHN_TST-2_1_T-1.cr.xml \
    --seed 440 \
    --num-vehicle 5 \
    --num-ego 1 \
    --max-attempts 200 \
    --simulation-steps 500 \
    --step-length 0.1 \
    --overwrite


