# KOBA (Keyboardless Object Bringing Assistant)

Tiago GPT-4 project for HRI course

[DEMO VIDEO](https://www.youtube.com/watch?v=W3SLLaTTGQ0)

## System setup

The following instructions are given for a system that uses a physical TIAGo robot running ROS Master on ROS Melodic, a second computer with ROS Noetic running teh ASR and GPT-4 parts and a third one with ROS Melodic running the object pickup part. The `ROS_MASTER_URI` and `ROS_IP` values have to be configured accordingly. It also assumes you have an OpenAI API key and Azure Speech API key. We recommend connecting the other two computers to TIAGo using ethernet cables. You may need a USB to ethernet converter.

Place the robot in front of a table with up to 5 sponges on it. The available AR markers to be used are 5, 26, 99, 182, 253 and markers should be placed on top of the sponges. The markers we used had side length 4.11 cm. When using different sizes, `gpt_pick_demo.launch` should be adjusted accordingly. The sponge size should be specified in `gpt_pick_demo.launch` file under `object_width`, `object_height` and `object_depth` parameters. Adjust the `tiago_gpt/src/pre_prompt.txt` file according to which objects you have available. Sit next to the robot, to the right and slighly behind of it.

## ASR and GPT-4

Speech recognition uses [Microsoft Azure speech-to-text](https://learn.microsoft.com/en-us/azure/cognitive-services/speech-service/how-to-recognize-speech?pivots=programming-language-python) and is working all the time. For the conversation logic and decision-making processes, we use the [GPT-4 API](https://platform.openai.com/docs/api-reference/completions) (8,192 tokens). Text intended for speech synthesis is always sent to Tiago following punctuation marks. The marker id is then shared on the `recognized` topic. The node uses ROS Noetic.

Before using install requirements:
```
cd tiago_gpt
pip install -r requirements.txt
```

Clone the `pal_interaction_msgs` package from [pal_msgs repository](https://github.com/pal-robotics/pal_msgs) into your workspace and build it. If it requires any dependencies then install those as well.

Then (after also cloning the present repository) build and source the workspace.

Insert your OpenAI API key and Azure Speech API key to the `tiago_gpt/src/robot_extra_dummy.py` file.

When ready, run the code with
```
rosrun tiago_gpt robot_extra_dummy.py
```

## Object pickup with MoveIt

This code runs on ROS Melodic.

[Follow the instructions to set up TIAGo packages.](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS)

Also clone the `tiago_gpt` package from the current repository to the same workspace. Build and source the workspace.

In one window, launch
```
roslaunch tiago_gpt gpt_pick_demo.launch
```

After waiting for the system to go through its startup, in another window trigger the preparation for object pickup with
```
rosservice call /pick_gui
```

Then give the robot oral instructions on which sponge to pick up (assuming the `robot_extra_dummy.py` node is running) and wait for it to drop it on top of you.
