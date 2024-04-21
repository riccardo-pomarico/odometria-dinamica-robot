# Calcolo dell'odometria di un robot

Corso: Robotics.

Autori: Riccardo Pomarico, Claudio Galimberti, Alexandra Iuga.

L'applicazione è stata realizzata in linguaggio C++ e ha come obiettivo principale il calcolo dell'odometria utilizzando la cinematica adeguata. Ciò include la determinazione delle velocità lineari e angolari del robot tramite i dati provenienti dai codificatori delle ruote e l'applicazione di metodi di integrazione come Euler e Runge-Kutta per il calcolo dell'odometria. Inoltre, è prevista la calibrazione dei parametri del robot per garantire una corrispondenza accurata con la posizione reale.
L'applicazione prevede anche la capacità di calcolare le velocità di controllo delle ruote basate sulle velocità lineari e angolari del robot. È stato implementato un servizio per ripristinare l'odometria a una posizione specifica, definita da coordinate cartesiane e angolo di orientamento. Infine, viene utilizzata la riconfigurazione dinamica per selezionare il metodo di integrazione più adatto durante l'esecuzione dell'applicazione.

# Files description:

- robotVelocity.cpp compute velocities with mecanum wheels kinematics
- robotOdometry.cpp gets the twist from geometry_msgs/TwistStamped, computes and publishes the odometry in /odom
- parametersCalibration.cpp is used to calibrate robot parameters (r, l, w, N) to match GT pose

# Parameters description:

- xPrevious, yPrevious, thetaPrevious are params which indicate the previous position of the robot
- vx, vy, wz are params which indicate a rough estimate of v and ω
- gearRatio and encoderResolution are used in the formula to convert ticks to rpm
- the array wheelRpmFromTick is the speed of the wheels, which is expressed in radians per minute
- r, l, w, T and N are the wheel radius, wheel position along x, wheel position along y, gear ratio and the encoders resolution

# TF tree structure:

The TF tree’s father represents the reference system of the environment, and the TF tree’s child, base_link, represents the center of mass of the robot.

# Custom message structure:

- wheelSpeed.msg contains the velocity of the wheel expressed in rpm obtained inverting the formula of the omnidirectional wheel mobile robot
- to check that message use rostopic echo /wheels_rpm
- ros custom message

# More info:

The bags must be inside the workspace src.

Service call:
this service allows to change dynamically the position of the robot it sets the three coordinates x, y and theta
$ rosservice call /givePose x y theta

Dynamic reconfiguration:
it allows to change the integration method dynamically while odometry is computed

- if you want to select Euler or RK through the drop-down menu, type
$ rosrun rqt_reconfigure rqt_reconfigure

- if you want to select Euler through command line, you can choose one of these alternatives
$ rosrun dynamic_reconfigure dynparam set /robotOdometry integrationMethod 0
$ rosrun dynamic_reconfigure dynparam set /robotOdometry integrationMethod Euler

- if you want to select Runge-Kutta through command line, you can choose one of these alternatives
$ rosrun dynamic_reconfigure dynparam set /robotOdometry integrationMethod 1
$ rosrun dynamic_reconfigure dynparam set /robotOdometry integrationMethod RK

Parameters calibration:
It's used to compute the optimal parameters to match the pose of the robot inside the bags.
We compute them one at a time by changing the first one to find the optimal value that we then use to compute the next and so on.
$ rosrun homework1 parametersCalibration

# Launch files descriptions:
It allows you to start all the nodes of the project (robot velocity and robot odometry) excepts for the parameters calibration.
$ roslaunch homework1 launcher.launch
