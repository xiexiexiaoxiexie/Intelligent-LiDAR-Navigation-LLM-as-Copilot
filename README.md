# Intelligent LiDAR Navigation: Leveraging External Information and Semantic Maps with LLM as Copilot

https://github.com/user-attachments/assets/02c2fd67-2ba3-4d8e-94e5-8a001c9315dd

### Introduction
This is the open source repository for the paper "Intelligent LiDAR Navigation: Leveraging External Information and Semantic Maps with LLM as Copilot" by Fujing Xie , Jiajie Zhang and Sören Schwertfeger ([Paper link](https://arxiv.org/pdf/2409.08493)).

### Base Idea
Recent efforts to integrate large language models (LLMs) into robot navigation often emphasize using vision-based approaches to extract semantic information of robot surrendings. However, traditional robotics primarily relies on maps and LiDAR for navigation for example well-used ROS move_base package. But if you think about it, robots don't need to identify specific obstacles; they simply need to recognize that something is blocking their path and find a way around it. We argue that, similar to the widely used ROS move_base package, vision information may not be essential for navigation. Instead, we propose leveraging LLMs to interpret textual maps and respond to external information, such as notifications about an intersection closure (for example the image below, a 3rd-party delivery robot on our University campus, where it is blocked by an intersection closure. Below the e-mail sent by Office of General Services announcing this closure is shown). For actual navigation, we can continue to utilize LiDAR and the ROS move_base framework.

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