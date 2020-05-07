# Betabot Moving in Square

- [Betabot Moving in Square](#betabot-moving-in-square)
  - [Project Description](#project-description)
  - [GUIDE](#guide)
  - [After you implement the pkg state your reflection below](#after-you-implement-the-pkg-state-your-reflection-below)
    - [How did you plan the task?](#how-did-you-plan-the-task)
    - [What ROS REPs did you used?](#what-ros-reps-did-you-used)
    - [How we could increase the overall performance?](#how-we-could-increase-the-overall-performance)
    - [List the most time consuming problems you faced](#list-the-most-time-consuming-problems-you-faced)
  - [Demo](#demo)
  - [Screenshot](#screenshot)
      - [NAME:](#name)
      - [ID:](#id)

## Project Description 

Create a ROS package with custom service/action c++/python to move the betabot in gazebo, the movement
triggered will have to be a square, like in the image below:
Betabot movement Use the data passed to this topic /move_betabot_in_square_custom to change the way
betabot moves. Depending on the side value, the service must move the robot in a square
movement based on the side given. Also, the robot must repeat the shape as many times as
indicated in the repetitions variable of the message. Finally, it must return True if everything
went OK in the success variable.


## GUIDE

1. Create a new launch file, called start_betabot_move_custom_service_server.launch,
that launches the new betabot_move_custom_service_server.py file.
2. Test that when calling this new /move_betabot_in_square_custom service, betabot
moves accordingly. This means, the square is performed taking into account the side
and repetitions variables.
3. Create a new service client that calls the service /move_betabot_in_square_custom,
and makes betabot moves in a small square twice and in a bigger square once.
It will be called betabot_move_custom_service_client.py. The launch that starts it will
be called call_betabot_move_in_square_custom_service_server.launch.
4. Refer to this tutorial for more help.
The BetabotCustomServiceMessage.srv will be something like this:

```sh
float64 side # The distance of each side of the square
int32 repetitions # The number of times the betabot robot has to execute the square
movement when the service is called
---
bool success # Did it achieve it?
```

>NOTE: The side variable doesn’t represent the real distance along each size of the square.It’s just a variable that will be used to change the size of the square. The bigger the size variable is, the bigger the square performed by the robot will be. You can use any robot pkg.

>To make you project standout try another shape and to use filtered `odemtry` data.

---

## After you implement the pkg state your reflection below

### How did you plan the task?

I started with a review of the 2 possible approaches to the problem, namely service-server or action-server. Upon reviewing, I realized that an action server with a series of goals (coordinates) would not only be more intuitive, but also would provide more control over the robot through continuous feedback. Next, I laid down the basic structure of the package using catkin, and edited the xml and cmake list accordingly. After that, I started writing the package implementation in c++ while abiding by ROS REPs and coding best practices as much as possible, i.e. not using global variables or namespaces for example. At this point the package could read the robot's pos and decide whether or not it has reached the set goal. The publisher then publishes to cmd_vel topic, moving the robot in Gazebo. The next step was to tune the speed of the robot so it avoids the lashing due to inertia, and to re-factor the code to be more efficient and less verbose. Finally, I tested the package on multiple inputs and monitored the performance in Gazebo, which was satisfactory.

### What ROS REPs did you used?

- REP 144 (ROS Package Naming)
- [CppStyleGuide](http://wiki.ros.org/action/fullsearch/CppStyleGuide?action=fullsearch&context=180&value=linkto%3A"CppStyleGuide")

### How we could increase the overall performance?

- Making sure to sync the timing
- Provide a helper function to generate the series of goals automatically, given a shape name and side length

### List the most time consuming problems you faced

Tuning the speed and control logic based on robot pos.

---

## Demo
Add unlisted youtube/drive video

[Demo](https://youtu.be/VBQChNo0Gfk)

## Screenshot

![image](/home/kasem/.config/Typora/typora-user-images/image-20200506094910733.png)

---

#### NAME: Mohamed Kasem Saber
#### ID: 201601144

---
