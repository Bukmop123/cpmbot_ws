# README #

to make the robot work:
mkdir cpmbot_ws
mkdir cpmbot_ws/src
git clone https://bitbucket.org/cpmbot/
cd ..
catkin_make

edit .bashrc

To launch simple gazebo (to evaluate the model)

run roslaunch cpmbot cpmbot_world.launch

To launch gazebo with sensors (to simulate)

roslaunch cpmbot cpmbot_gazebo_world.launch

jstest /dev/input/js0
### What is this repository for? ###

* Quick summary
* Version
* [Learn Markdown](https://bitbucket.org/tutorials/markdowndemo)

### How do I get set up? ###

* Summary of set up
* Configuration
* Dependencies
* Database configuration
* How to run tests
* Deployment instructions

### Contribution guidelines ###

* Writing tests
* Code review
* Other guidelines

### Who do I talk to? ###

* Repo owner or admin
* Other community or team contact
