# Betabot Obstacle avoider

- [Betabot Obstacle avoider](#betabot-obstacle-avoider)
  - [Project Description](#project-description)
  - [After you implement the pkg state your reflection below](#after-you-implement-the-pkg-state-your-reflection-below)
    - [How did you plan the task?](#how-did-you-plan-the-task)
    - [what is your logic?](#what-is-your-logic)
    - [What ROS REPs did you used?](#what-ros-reps-did-you-used)
    - [How we could increase the overall performance?](#how-we-could-increase-the-overall-performance)
    - [List the most time consuming problems you faced](#list-the-most-time-consuming-problems-you-faced)
    - [Demo](#demo)
    - [Screenshot](#screenshot)
      - [NAME:](#name)
      - [ID:](#id)

## Project Description 

Create a ROS package with custom nodes c++/python to move the
betabot randomly in gazebo, the movement should be triggered then the robot
moves randomly while avoid objects based on laser scans reading based on the laser
scans.


>NOTE: DON'T process one ray of the laser scans array or it will be considered ultrasonic/IR sensor.try to come up with approach thats use the laser full potential. 

>To make you project standout try not to visit any place twice.

---

## After you implement the pkg state your reflection below

### How did you plan the task?

This task requires having access to the scan data as well as a way to control the robot both angular and linear velocities. This is implemented through having a subscriber to /scan topic and a publisher to cmd_vel that have to be able to communicate through the logic. To avoid using global variables, I implemented a class that has both objects as members along with other members to allow for communication and control flow. Every time a new scan arrives, the data is inspected for obstacles to the front, left and right. 

### what is your logic?

The robot will then have to move to the left if right area is blocked and vice versa, however if both right and left are clear, the robot will move forward. To do that I count the number of rays that are sub-threshold on both sides. If either side is more than a certain number, the robot should move towards the one scoring less. 

### What ROS REPs did you used?

- REP 144 (ROS Package Naming)
- [CppStyleGuide](http://wiki.ros.org/action/fullsearch/CppStyleGuide?action=fullsearch&context=180&value=linkto%3A"CppStyleGuide")

### How we could increase the overall performance?

Tuning the thresholds and number of sub-threshold rays.

### List the most time consuming problems you faced

Understanding the /scan topic and figuring out that the link was shifted 90 degrees. 

---

### Demo
Add unlisted youtube/drive video

[Demo](https://youtu.be/efGobdHP3MU)

### Screenshot

[image](yourscreenshot)

---

#### NAME: Mohamed Kasem 	
#### ID: 201601144

---
