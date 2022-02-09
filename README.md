# PyRep [![Build Status](https://github.com/stepjam/PyRep/workflows/Build/badge.svg)](https://github.com/stepjam/PyRep/actions) [![Discord](https://img.shields.io/discord/694945313638842378.svg?label=&logo=discord&logoColor=ffffff&color=7389D8&labelColor=6A7EC2)](https://discord.gg/eTMsa5Y)


__PyRep is a toolkit for robot learning research, built on top of [CoppeliaSim](http://www.coppeliarobotics.com/) (previously called V-REP).__


- [Install](#install)
- [Running Headless](#running-headless)
- [Getting Started](#getting-started)
- [Usage](#usage)
- [Supported Robots](#supported-robots)
- [Adding Robots](#adding-robots)
- [Contributing](#contributing)
- [Projects Using PyRep](#projects-using-pyrep)
- [What Happened to V-REP?](#what-happened-to-v-rep)
- [Citation](#citation)


## Install

PyRep requires version **4.1** of CoppeliaSim. Download: 
- [Ubuntu 16.04](https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_1_0_Ubuntu16_04.tar.xz)
- [Ubuntu 18.04](https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_1_0_Ubuntu18_04.tar.xz)
- [Ubuntu 20.04](https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04.tar.xz)

Once you have downloaded CoppeliaSim, you can pull PyRep from git:

```bash
git clone https://github.com/stepjam/PyRep.git
cd PyRep
```

Add the following to your *~/.bashrc* file: (__NOTE__: the 'EDIT ME' in the first line)

```bash
export COPPELIASIM_ROOT=EDIT/ME/PATH/TO/COPPELIASIM/INSTALL/DIR
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT
export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT
```

__Remember to source your bashrc (`source ~/.bashrc`) or 
zshrc (`source ~/.zshrc`) after this.

Finally install the python library:

```bash
pip3 install -r requirements.txt
pip3 install .
```

You should be good to go!
Try running one of the examples in the *examples/* folder.

_Although you can use CoppeliaSim on any platform, communication via PyRep is currently only supported on Linux._

#### Troubleshooting

Below are some problems you may encounter during installation. If none of these solve your problem, please raise an issue.

- ModuleNotFoundError: No module named 'pyrep.backend._v_rep_cffi'
  - If you are getting this error, then please check that you are not running the interpreter from the project root. If you are, then your Python interpreter will try to import those files rather the installed files.
- error: command 'x86_64-linux-gnu-gcc' failed
  - You may be missing packages needed for building python extensions. Try: `sudo apt-get install python3-dev`, and then re-run the installation.

## Running Headless

You can run PyRep/CoppeliaSim headlessly with VirtualGL. VirtualGL is an open source toolkit that gives any Unix or Linux remote display software the ability to run OpenGL applications **with full 3D hardware acceleration**.
First insure that you have the nVidia proprietary driver installed. I.e. you should get an output when running `nvidia-smi`. Now run the following commands:
```bash
sudo apt-get install xorg libxcb-randr0-dev libxrender-dev libxkbcommon-dev libxkbcommon-x11-0 libavcodec-dev libavformat-dev libswscale-dev
sudo nvidia-xconfig -a --use-display-device=None --virtual=1280x1024
# Install VirtualGL
wget https://sourceforge.net/projects/virtualgl/files/2.5.2/virtualgl_2.5.2_amd64.deb/download -O virtualgl_2.5.2_amd64.deb
sudo dpkg -i virtualgl*.deb
rm virtualgl*.deb
```
You will now need to reboot, and then start the X server:
```bash
sudo reboot
nohup sudo X &
```
Now we are good to go! To render the application with the first GPU, you can do the following:
```bash
export DISPLAY=:0.0
python my_pyrep_app.py
```
To render with the second GPU, you will insetad set display as: `export DISPLAY=:0.1`, and so on.

**Acknowledgement**: Special thanks to Boyuan Chen (UC Berkeley) for bringing VirtualGL to my attention!

## Getting Started

1. First take a look at [Usage](#usage) and the examples in the *examples/* folder to see if PyRep might be able to accelerate your research.
2. Take a look at the CoppeliaSim [tutorials](http://www.coppeliarobotics.com/helpFiles/en/tutorials.htm).

## Usage

The best way to see how PyRep can help in your research is to look at the examples in the *examples/* folder!

#### Launching the simulation

```python
from pyrep import PyRep

pr = PyRep()
# Launch the application with a scene file in headless mode
pr.launch('scene.ttt', headless=True) 
pr.start()  # Start the simulation

# Do some stuff

pr.stop()  # Stop the simulation
pr.shutdown()  # Close the application
```


#### Modifying the Scene

```python
from pyrep.objects.shape import Shape
from pyrep.const import PrimitiveShape

object = Shape.create(type=PrimitiveShape.CYLINDER, 
                      color=[r,g,b], size=[w, h, d],
                      position=[x, y, z])
object.set_color([r, g, b])
object.set_position([x, y, z])
```

#### Using Robots

Robots are designed to be modular; arms are treated separately to grippers.

Use the robot ttm files defined in robots/ttms. These have been altered slightly from the original ones shipped with CoppeliaSim to allow them to be used with motional planning out of the box. 
The 'tip' of the robot may not be where you want it, so feel free to play around with this.

```python
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper

pr = PyRep()
# Launch the application with a scene file that contains a robot
pr.launch('scene_with_panda.ttt') 
pr.start()  # Start the simulation

arm = Panda()  # Get the panda from the scene
gripper = PandaGripper()  # Get the panda gripper from the scene

velocities = [.1, .2, .3, .4, .5, .6, .7]
arm.set_joint_target_velocities(velocities)
pr.step()  # Step physics simulation

done = False
# Open the gripper halfway at a velocity of 0.04.
while not done:
    done = gripper.actuate(0.5, velocity=0.04)
    pr.step()
    
pr.stop()  # Stop the simulation
pr.shutdown()  # Close the application
```

We recommend constructing your robot in a dictionary or a small structure, e.g.


```python
class MyRobot(object):
  def __init__(self, arm, gripper):
    self.arm = arm
    self.gripper = gripper

arm = Panda()  # Get the panda from the scene
gripper = PandaGripper()  # Get the panda gripper from the scene

# Create robot structure
my_robot_1 = MyRobot(arm, gripper)
# OR
my_robot_2 = {
  'arm': arm,
  'gripper': gripper
}
```

#### Running Multiple PyRep Instances

Each PyRep instance needs its own process. This can be achieved using Pythons [multiprocessing](https://docs.python.org/3.6/library/multiprocessing.html) module. Here is a simple example:

```python
from multiprocessing import Process

PROCESSES = 10

def run():
  pr = PyRep()
  pr.launch('my_scene.ttt', headless=True)
  pr.start()
  # Do stuff...
  pr.stop()
  pr.shutdown()

processes = [Process(target=run, args=()) for i in range(PROCESSES)]
[p.start() for p in processes]
[p.join() for p in processes]
```

## Supported Robots

Here is a list of robots currently supported by PyRep:

#### Arms

- Kinova Mico
- Kinova Jaco
- Rethink Baxter
- Rethink Sawyer
- Franka Emika Panda
- Kuka LBR iiwa 7 R800
- Kuka LBR iiwa 14 R820
- Universal Robots UR3
- Universal Robots UR5
- Universal Robots UR10

#### Grippers

- Kinova Mico Hand
- Kinova Jaco Hand
- Rethink Baxter Gripper
- Franka Emika Panda Gripper

#### Mobile Robots

- Kuka YouBot
- Turtle Bot
- Line Tracer

Feel free to send pull requests for new robots!

## Adding Robots

If the robot you want is not currently supported, then why not add it in!

[Here is a tutorial for adding robots.](tutorials/adding_robots.md)

## Contributing

We want to make PyRep the best tool for rapid robot learning research. If you would like to get involved, then please [get in contact](https://www.doc.ic.ac.uk/~slj12/)!

Pull requests welcome for bug fixes!

## Projects Using PyRep

If you use PyRep in your work, then get in contact and we can add you to the list!

- [RLBench: The Robot Learning Benchmark & Learning Environment, arxiv 2019](https://arxiv.org/abs/1909.12271)
- [Learning One-Shot Imitation from Humans without Humans, arxiv 2019](https://arxiv.org/abs/1911.01103)
- [Task-Embedded Control Networks for Few-Shot Imitation Learning, CoRL 2018](https://arxiv.org/abs/1810.03237)
- [Transferring End-to-End Visuomotor Control from Simulation to Real World for a Multi-Stage Task
, CoRL 2017](https://arxiv.org/abs/1707.02267)

## Acknowledgements

- Georges Nomicos (Imperial College London) for the addition of mobile platforms.
- Boyuan Chen (UC Berkeley) for bringing VirtualGL to my attention.

## What Happened to V-REP?

Coppelia Robotics discontinued development of __V-REP__. Instead, they now focus
their efforts on __CoppeliaSim__. CoppeliaSim is 100% compatible with V-REP.
See more information [here](http://coppeliarobotics.com/helpFiles/en/versionInfo.htm#coppeliaSim4.0.0).

PyRep is fully compatible with both V-REP and CoppeliaSim.


## Citation

```
@article{james2019pyrep,
  title={PyRep: Bringing V-REP to Deep Robot Learning},
  author={James, Stephen and Freese, Marc and Davison, Andrew J.},
  journal={arXiv preprint arXiv:1906.11176},
  year={2019}
}
```
