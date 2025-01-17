# AGRO 

This repo contains the starter code for the AGRO project in [Artificial Intelligence in Dynamic Action (AIDA) lab, Skoltech.](https://sites.skoltech.ru/aida)



## Dependencies for running locally
* cmake >= 3.8
    * All OSes: [click here for installation instructions](https://cmake.org/install/)

* make >= 4.1 (Linux, Mac), 3.81 (Windows)
    * Linux: make is installed by default on most Linux distros
    * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)


## Instructions
`mkdir -p AIDA_ws/src`

`cd AIDA_ws/src`

`git clone https://gitflic.ru/project/aidynamicaction/project2022-agrobot.git`

`cd project2022-agrobot`

`sudo mv robot_package agro`

`sudo mv agro ../`

`cd ../..`

`catkin_make`

Based on the shell you use on your pc:
`source devel/setup.<your_shell_postfix>`

and then copy the last command to your .bashrc or .zshrc!
`echo "source $PWD/devel/setup.<your_shell_postfix>" >> ~.<your shell rc>`



## Commands

1-`roscd agro`

2-`cd ../../`

3-`catkin_make`

4-`roscore`

5-`roslaunch agro main.launch`

