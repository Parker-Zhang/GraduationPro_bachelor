# GraduationPro_bachelor
## main content：
A graduation project for bachelor degree.

Use dynamixel motors to control 3 DOF plant.

A distribute control method is applied.

## code guide

the main ros package I rewrite is in `my_dynamixel_workbench_test` folder.

when you put it into your catkin workspace, then compile it.

```shell
cd catkin_ws
catkin_make
```

you start the master to control motors.

```shell
roslaunch my_dynamixel_workbench_test my_test.launch
```

you publish trajectory message to master and the motor will rotate.

```shell
roslaunch my_dynamixel_workbench_test start_operator.launch 
```

**Mark 1.** the trajectory yaml files are defined in folder `cfg`

you can change the launch to load different yaml file.

## other thing

the source code include data, figure, matlab code either , which may have no help to you, so you can ignore immediately.

if you have any question , you can connect the author through email 1742356236@qq.com .

Thank you for your reading.