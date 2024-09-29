# Intelligent LiDAR Navigation: Leveraging External Information and Semantic Maps with LLM as Copilot

https://github.com/user-attachments/assets/02c2fd67-2ba3-4d8e-94e5-8a001c9315dd

### Introduction
This is the open source repository for the paper "Intelligent LiDAR Navigation: Leveraging External Information and Semantic Maps with LLM as Copilot" by Fujing Xie , Jiajie Zhang and Sören Schwertfeger ([Paper link](https://arxiv.org/pdf/2409.08493)).

### Base Idea
Recent efforts to integrate large language models (LLMs) into robot navigation often emphasize using vision-based approaches to extract semantic information of robot surroundings. However, traditional robotics primarily relies on maps and LiDAR for navigation, for example well-used ROS move_base package. If you think about it, robot navigation doesn't need to identify specific obstacles; they simply need to recognize that something is blocking their path and find a way around it. We argue that, similar to the widely used ROS move_base package, vision information may not be essential for navigation. Instead, we propose leveraging LLMs to interpret textual maps and respond to external information, such as notifications about an intersection closure (for example the image below, a 3rd-party delivery robot on our University campus, where it is blocked by an intersection closure. Below the e-mail sent by Office of General Services announcing this closure is shown). For actual navigation, we can continue to utilize LiDAR and the ROS move_base framework. However, typical robotics map like occupancy grid maps are hard for LLMs to understand, and topological maps can be formatted to be text, but they don't have geometric information for navigation. Therefore, we need a map that:
    1. textual format for LLMs to understand
    2. contain geometric info for navigation
We propose using osmAG map to bridge this gap, which is a hierarchical, topometric, and semantic map representation.
![email](https://github.com/user-attachments/assets/e549a4a2-7f00-49d1-b2a0-7ed32f7a9072)
### Abstract
Traditional robot navigation systems primarily utilize occupancy grid maps and laser-based sensing technologies, as demonstrated by the popular move_base package in ROS. Unlike robots, humans navigate not only through spatial awareness and physical distances but also by integrating external information, such as elevator maintenance updates from public notification boards and experiential knowledge, like the need for special access through certain doors. With the development of Large Language Models (LLMs), which posses text understanding and intelligence close to human performance, there is now an opportunity to infuse robot navigation systems with a level of understanding akin to human cognition. In this study, we propose using osmAG (Area Graph in OpensStreetMap textual format), an innovative semantic topometric hierarchical map representation, to bridge the gap between the capabilities of ROS move_base and the contextual understanding offered by LLMs. Our methodology employs LLMs as actual copilot in robot navigation, enabling the integration of a broader range of informational inputs while maintaining the robustness of traditional robotic navigation systems. 

To be continued...
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
