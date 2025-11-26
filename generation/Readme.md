To create scenario with vehicles (generate_vehicle.py):

vehicles_config = {
    "CAR": 1,
    "TRUCK": 1,
}
random_seed = 3
TST= scenario_with_vehicles(scenario,random_seed=random_seed,vehicles_config=vehicles_config)
TST_scenario = TST.generate_vehicle()

To initialize blank scenario with planning problems and trajectories (batch_planning_problem_generator.py):

```bash
python generation/initialize_scenario.py \
    --input interactive_scenarios/CHN_TST-2_1_T-1/CHN_TST-3_1_T-1.cr.xml \
    --seed 31 \
    --num-vehicle 5 \
    --num-ego 1 \
    --max-attempts 800 \
    --output-dir interactive_scenarios/CHN_TST-3_1_T-31 \
    --overwrite
```

执行后将得到：
规划问题：interactive_scenarios/CHN_TST-3_1_T-30/CHN_TST-3_1_T-1-30.cr.xml
配套轨迹：interactive_scenarios/CHN_TST-3_1_T-30/CHN_TST-3_1_T-1.xml_30_solution.xml


To generate SUMO configuration files from CommonRoad scenario (scenario_config_generator.py):

```bash
python generation/sumo_config_generator.py \
    --scenario_name CHN_TST-2_1_T-300 \
    --simulation_steps 500 \
    --step_length 0.1 \
    --num_pedestrians 0
```

> 说明：`--scenario_name` 会定位到 `interactive_scenarios/CHN_TST-3_1_T-30/`，脚本会自动匹配该目录下唯一的 `.cr.xml` 与 `*_solution.xml` 文件。如果存在多个候选文件，请显式提供 `--commonroad_file`（以及 `--solution_file`）参数。

执行后将得到：
SUMO配置文件：.../CHN_TST-2_1_T-23.sumo.cfg
SUMO网络文件：.../CHN_TST-2_1_T-23.net.xml
SUMO车辆路由文件：.../CHN_TST-2_1_T-23_solution.rou.xml


To run the whole pipeline once (initialize scenario + generate SUMO configs):

./generation/run_full_pipeline.sh \
    --input interactive_scenarios/CHN_TST-2_1_T-1/CHN_TST-2_1_T-1.cr.xml \
    --seed 20 \
    --num-vehicle 6 \
    --num-ego 1 \
    --max-attempts 300 \
    --simulation-steps 500 \
    --step-length 0.1 \
    --overwrite

python generation/sumo_from_graph.py \
    --scenario_name CHN_TST-4_1_T-1 

