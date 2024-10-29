# Intelligent LiDAR Navigation: Leveraging External Information and Semantic Maps with LLM as Copilot

https://github.com/user-attachments/assets/02c2fd67-2ba3-4d8e-94e5-8a001c9315dd

### Introduction
This is the open source repository for the paper "Intelligent LiDAR Navigation: Leveraging External Information and Semantic Maps with LLM as Copilot" by Fujing Xie , Jiajie Zhang and Sören Schwertfeger ([Paper link](https://arxiv.org/pdf/2409.08493)).

### Base Idea
Recent efforts to integrate large language models (LLMs) into robot navigation often focus on using vision-based approaches to extract semantic information of robot surroundings. However, traditional robotics primarily relies on maps and LiDAR for navigation, for example well-used ROS move_base package. In essence, robot navigation doesn’t require the identification of specific obstacles; it only needs to recognize that something is obstructing the path and find an alternative route. We argue that, much like the widely used ROS move_base package, vision information might not be essential for navigation. Instead, we propose leveraging LLMs to interpret textual maps and respond to external information, such as notifications about an intersection closure (for example, the image below shows a 3rd-party delivery robot on our University campus blocked by an intersection closure, accompanied by an e-mail sent by Office of General Services announcing this closure). However, typical robotics map like occupancy grid maps are hard for LLMs to understand, and topological maps can be formatted to be text, but they don't have geometric information for navigation. Therefore, we need a map that:
    1. Is in a textual format for LLMs to understand
    2. Contains geometric info for navigation
We propose using osmAG map to bridge this gap, as it is a hierarchical, topometric, and semantic textual map representation.
![email](https://github.com/user-attachments/assets/e549a4a2-7f00-49d1-b2a0-7ed32f7a9072)
### Abstract
Traditional robot navigation systems primarily utilize occupancy grid maps and laser-based sensing technologies, as demonstrated by the popular move_base package in ROS. Unlike robots, humans navigate not only through spatial awareness and physical distances but also by integrating external information, such as elevator maintenance updates from public notification boards and experiential knowledge, like the need for special access through certain doors. With the development of Large Language Models (LLMs), which posses text understanding and intelligence close to human performance, there is now an opportunity to infuse robot navigation systems with a level of understanding akin to human cognition. In this study, we propose using osmAG (Area Graph in OpensStreetMap textual format), an innovative semantic topometric hierarchical map representation, to bridge the gap between the capabilities of ROS move_base and the contextual understanding offered by LLMs. Our methodology employs LLMs as actual copilot in robot navigation, enabling the integration of a broader range of informational inputs while maintaining the robustness of traditional robotic navigation systems. 
### Folder & File Descriptions
- config: rviz configuration file
- external_info: external notification used in the paper
- launch: launch file for the project
- msg: custom message for the project
- osmAG: osmAG map used in the paper
  - occupancy_grid_map: generated when running, only contains chosed areas and passages
  - real: osmAG map used in this project.
    -  2F_ShanghaiTech_merge.osm can be visualized in JOSM
    - 2F_ShanghaiTech_merge_utm_path_0704.json is the path length file, which only depends on the map, please refer to Section. III-C of the paper for more details.
- worlds: gazebo world file used in the experiments.
- scripts: python scripts for the project
  - 'cases' folder: contains 4 cases used in the paper and their recorded result.
  - 'launch' folder: contains launch file that switch maps using map_server.
  - config.json: configuration file for the project. Please put your own API key in this file.
  - PassageCostEvaluator.py: used to evaluate the cost of passages in the map, please refer to 'PassageCostEvaluator' module in the paper for more details.
  - osmAGPathPlanning.py: used to plan path in the map, please refer to 'osmAGPathPlanning' module in the paper for more details.
  - NavigationEventMonitor.py: used to monitor the navigation event and approve path planned by osmAGPathPlanning module, please refer to 'NavigationEventMonitor' module in the paper for more details.
### How to Use the Code
Step 1: Setting Up the Environment

`conda env create -f ./intelligent_navigation.yml`

Step 2: put the package in a ros workspace folder and catkin_make the ros workspace folder.

`catkin_make`

Step 3: Source the workspace

`source devel/setup.bash`

Step 4: Run the launch file

`roslaunch osmAG_intelligent_navigation robot2.launch`

### Citation
If you find this work useful, please consider citing the paper:
```
@article{xie2024intelligent,
  title={Intelligent LiDAR Navigation: Leveraging External Information and Semantic Maps with LLM as Copilot},
  author={Xie, Fujing and Zhang, Jiajie and Schwertfeger, S{\"o}ren},
  journal={arXiv preprint arXiv:2409.08493},
  year={2024}
}

```
