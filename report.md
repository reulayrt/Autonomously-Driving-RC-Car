# Cruise Control
Authors: Ayrton Reulet, Keven DeOliveira, Brian Jung

Date: 2022-04-22
-----

## Summary

In this Quest we create a robust platform for autonomous driving by implementing "cruise control", what that means is that we need to maintain a constant velocity and avoid coliision to obstical. We also make sure that the car does not steer to far from the center and collide with the sides.

We were able to succesfully use PID to not only control the speed to keep it within the desired range, but we also utilize PID to adjust the angle of the servo controlling the front wheels to steer the car properly. Finally, we just the front facing sensor to come to a halt when there is an obstacle getting close to the car.

To read more about our solution specifically, please continue reading to the solution design.

To understand the code in more detail, please go to [CODE README](https://github.com/BU-EC444/Team13-DeOliveira-Jung-Reulet/blob/master/quest-5/code/README.md) in the code folder for more information. 

### Investigative Question
How would you change your solution if you were asked to provide 'adaptive' cruise control?
Adaptive cruise control is when the vehicle's acceleration and braking is automatically controlled by monitoring other vehicles and objects on the road. It keeps a safe distance from the vehicle infront of you while keeping the speed of the vehicle steady.
In order to implement this, we just have to tweak our PID a little while adding more sensors for the car to be more "aware" of the surroundings. For example, the front LIDAR sensor sesnes distance of what is in front of you. In case of a wall - where the car sees that the distance value decreases over time - it stops the car when it senses 30cm (in order to account for momemtum). Now we need to add logic to see if the distance sensor infront is dynamic. Keep PID for cruise control (keeping the car going at 0.1m/s - 0.4m/s) until they sense that it is coming close to something. Then slow down but don't come to a halt and see if the distance increases. If the distance increase, logic tells us its most likely a car in front, then we can keep a safe distance using PID and cruise control. If it continues to decrease, then the car should come to a halt. 
Another way to implement this (while less possible real life, but very possible for ESP controlled crawlers) is to have the microcontrollers that control the speed of the crawlers to communicate with each other, through UDP or other wireless communication to send out heartbeats of the speed. This way the crawlers can all have the same speed.

## Solution Design
Our design does everything the quest asks for and more! 

First we wanted to establish the sensors on the vehicle and make sure they are working. To begin, we worked with the optical encoder and a black white circle or skill 32 to make sure that the sensor was getting the proper speed. Then we hooked up LIDAR-lite sensor to the front of the car to get the front distance from obstacle. We had to learn to hook up two i2c components to one esp, as the alphanumeric display was connected. We manage to figure it out, and we finally connected 2 IR sensors to the side.

Once we have the sensors set up, we made xtasks that update global variables of speed, front, left, and right distance. These variables will be accessed to work our logic of PID and control approach. First we used the PID template we got from the skill to create the PID control for the speed. If the calculated speed was too fast we slow down and if it was too slow, we speed up. Next we added logic to say if the front sensor is less than 30, stop the car. This is because we were taking account of the momentum of the car. Next we also used PID to look at the distance of the right side of the car (facing the wall) and made sure that it is a constant 25cm away. We turn the wheels to steer the car to try to keep the constant 25.

Finally, we added UDP and used if statements and checks to turn the vehicle on off with out sever (frontend and backend js), please go to [CODE README](https://github.com/BU-EC444/Team13-DeOliveira-Jung-Reulet/blob/master/quest-5/code/README.md) in the code folder for more information. 

### How our solution went beyond in terms of quality.
First of all, we put a lot of effort to make the crawler not only functional in terms of meeting all the solution requirements but stylish. We have added a flag that read "Team 13" in the back as an added feature. On top of that we got a wooden plank that can host multiple breadboards as well as sensors. Instead of having just one sensor on the side (while we needed only one for this quest), we also added one on the other side so that the car was more "aware" of the surrounding.
As you can see from our stopping video (Linked in Supporting Artifacts), we also made our solution be able to stop within 10cm of the end of track without collision. This is a 2X improvement of the 20cm of end of track. 
Finally, you can also see from the starting at angle video that our PID control for the wall works with a different starting angle. Which shows the quality of our solution.


## Sketches and Photos
![Beauty](https://github.com/BU-EC444/Team13-DeOliveira-Jung-Reulet/blob/master/quest-5/images/20220422_145145.jpg)
![Circuit](https://github.com/BU-EC444/Team13-DeOliveira-Jung-Reulet/blob/master/quest-5/images/20220422_145121.jpg)
![Sensor](https://github.com/BU-EC444/Team13-DeOliveira-Jung-Reulet/blob/master/quest-5/images/20220422_145231.jpg)


## Supporting Artifacts
Please press on the hyperlinks below to watch the corresponding videos!

- [Explanation Video](https://drive.google.com/file/d/16FNCnZ4mt5uNj3LpY_JzY2UY0W4ui9-H/view?usp=sharing)

