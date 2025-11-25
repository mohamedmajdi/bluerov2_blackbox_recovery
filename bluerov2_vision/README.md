Here's your reformatted README:

# BlueROV2 Vision Package - README

Hello Mahmoud, Maghdy, or probably Elex! Hi Tasia! If you're reading this, I'm most likely not there and you want to know how this all works. Well, I'll write it down for y'all real simple.

---

## 1. Config Folder

There's a config folder with the old configuration file. I should've removed it, but it's nice to see the base confidence values on file. Note that **all parameters are now editable via ROS param**, so you don't need to modify the YAML file directly.

---

## 2. Launch Files

There are multiple launch files, but you only need **2**:

- **`detection_only.launch.py`**: Detection only with YOLO
- **`detection.launch.py`**: Full package (YOLO detection + orientation + ArUco)

***

## 3. Detection Node (YOLO)

**The main attraction: YOLO!** Change the values as you please via ROS parameters.

### Confession Time

The model detection **is not tested yet**. As a matter of fact, **this whole codebase is not tested!**

### Confidence Parameters

We have **2 types of confidence**:

- **Main confidence**: For the black box
- **Handle confidence**: For the... duh, the handle!

#### Why Differentiate?

Because the model is so dumb sometimes, you need to give it time and confidence, especially with handles. Just **lower the handle confidence** until it detects the handle. Still won't detect? Then I just wasted my time today since 4am... or maybe 5...

### IoU Threshold

**IoU (Intersection over Union)** controls how much overlap is allowed between detections. Increase it in case the handle is hard to detect, or decrease it... I kinda forgot which way. Either way, because our two classes are so close and normalized, the bounding boxes or segmentation masks will often overlap.

### Device Parameter

- **Set to `0`**: If you have an NVIDIA GPU
- **Set to `cpu`**: If you don't have a GPU (it's gonna lag a bit... maybe... I only use the YOLO nano model, heh. I know something else that is nano.)

***

## 4. ArUco Tag Detection

As the name suggests... **it detects ArUco markers!** According to Perplexity, we have IDs up to 249, or I might be wrong. So don't be holding your horses on this one. We can fix this tomorrow—oh wait, tomorrow is today. Huh, funny.

***

## 5. Orientation Node

This is what Mahmoud said was a **"2 lines of code" program**... ended up being **200 lines**. I guess LLMs do be that stupid, or me being stupid and overthinking it.

Anyway, **all the variables are changeable via ROS param** (I LLM'ed it! Like what he would do! Mahmoud will understand this... Tasya too...).

***

## 6. Human-Made Graphs

If you guys still don't understand what I did, or what the LLM did, here are some **human-made graphs** to make it more understandable.

![ROS System Diagram](./img/ros_diagram.png)
***


**Good luck! - Written at ungodly hours by someone who clearly needs sleep**
