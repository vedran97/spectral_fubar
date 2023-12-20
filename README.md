# Project Spectral Fubar
![CICD Workflow status](https://github.com/vedran97/spectral_fubar/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/vedran97/spectral_fubar/branch/master/graph/badge.svg)](https://codecov.io/gh/vedran97/spectral_fubar)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Description

This project uses a Husky robot, mounted with a variety of vision sensors, for post natural disaster road inspection.
The Husky robot uses a depth camera, and object detection/localization algorithm to detect, localize an obstacle/debris seen on a road.
![Alt text](screenshot/husky_outdoor%20world%20with%20obstacles.png)

## Personnel

1. Vedant Ranade
2. Jerry Pittman
3. Aaqib Barodawala

## Environment

1. This repo is designed to be used with ROS2 Humble and Ubuntu 22.04

## AIP and Final Deliverables

1. Sprint Log is here: [LINK](https://docs.google.com/document/d/1LM4T2IaXjHfa725iT_XrGdjAjX6Vu2G_JORIrd1daMI/edit?usp=sharing)
2. AIP Work Logs are here: [LINK](https://docs.google.com/spreadsheets/d/1apXGEWr1nkUdqm7aSSiJ8yui3v2pkUTxmMRXO4ydDmY/edit?usp=sharing)
3. Final Deliverables are here: [LINK](https://drive.google.com/drive/folders/1iyCzRlYM_VHO__MNFvLk7GiNthm--4n8?usp=drive_link) (including proposal and final presentation)
4. PHASE 2 VIDEO is here [LINK](https://drive.google.com/file/d/1cSz5KwAFxXcn649TfvL50kB0wMkjQPaL/view?usp=sharing).

## Repository Setup instruction

1. clone this repository in your ros2 workspace's src folder by ```git clone https://github.com/vedran97/spectral_fubar.git```
2. clone the following repository in your ros2 workspace's src folder ```https://github.com/husky/husky.git```
3. launch a terminal window in workspace root, ```cd src/husky && git checkout humble-devel && cd -```
4. Run ```rosdep install -i --from-path src --rosdistro humble -y``` in a command line in workspace root
5. Run ```sudo apt-get install ros-humble-realsense2-*```

## Build instructions

1. Build the workspace:```colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers $(nproc)```
2. Source the workspace: ```source install/setup.bash```

## Cpp-Tools instructions

```bash
# run clang-format from workspace root

  cd src/spectral_fubar && clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -vE -e "^(./build/|./install/|./log/)") && cd -

# run cppcheck from inside the pkg directory
  mkdir results -p && cppcheck --enable=all --std=c++17 -I include/ --suppress=missingInclude --inline-suppr $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/|./test/)" ) &> results/cppcheck

# run cpplint from inside the pkg directory

  mkdir results -p && cpplint  --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/|./test/)" ) &> results/cpplint

```

## Test instructions

1. cd to your colcon_ws
2. source appropriate ros
3. Run this ```rm -rf build/spectral_fubar```
4. Run this next to build package,source package,run the test```colcon build --symlink-install --cmake-args -DCOVERAGE=1 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DINTEGRATION_TEST=ON --parallel-workers $(nproc) --packages-select spectral_fubar && source install/setup.bash && colcon test --packages-select spectral_fubar```
5. Run this to check results: ```cat log/latest_test/spectral_fubar/stdout_stderr.log```

## Generate code cov report (run in repo root)

1. ```colcon build --symlink-install --cmake-args -DCOVERAGE=1```
2. ```source install/setup.bash```
3. ```colcon test```
4. ```ros2 run spectral_fubar generate_coverage_report.bash```
5. ```rm -rf src/spectral_fubar/codecov_html```
6. ```mkdir src/spectral_fubar/codecov_html```
7. ```genhtml --output-dir ./src/spectral_fubar/codecov_html/ ./build/spectral_fubar/test_coverage.info```
8. ```cp ./build/spectral_fubar/test_coverage.info ./src/spectral_fubar/```

## Docs generation

0. ```sudo apt install -y doxygen lcov gcovr pandoc```
1. ```colcon build --event-handlers console_cohesion+ --packages-select spectral_fubar --cmake-target "docs"```

## Launch Instructions

Change directory to your ROS2 workspace

```bash
cd <your_ros2_workspace>
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers $(nproc)
source install/setup.bash
export HUSKY_URDF_EXTRAS=$(ros2 pkg prefix spectral_fubar)/share/spectral_fubar/urdf/realsense.urdf.xacro
export CPR_URDF_EXTRAS=$(ros2 pkg prefix spectral_fubar)/share/spectral_fubar/urdf/realsense.urdf.xacro
export GAZEBO_MODEL_PATH=$(ros2 pkg prefix spectral_fubar)/share/spectral_fubar/models:~/.gazebo/models:$GAZEBO_MODEL_PATH
ros2 launch spectral_fubar husky_outdoor.launch.py

```

In another terminal

```bash
cd <your_ros2_workspace>
source install/setup.bash
export HUSKY_URDF_EXTRAS=$(ros2 pkg prefix spectral_fubar)/share/spectral_fubar/urdf/realsense.urdf.xacro
export CPR_URDF_EXTRAS=$(ros2 pkg prefix spectral_fubar)/share/spectral_fubar/urdf/realsense.urdf.xacro
export GAZEBO_MODEL_PATH=$(ros2 pkg prefix spectral_fubar)/share/spectral_fubar/models:~/.gazebo/models:$GAZEBO_MODEL_PATH
ros2 run spectral_fubar inspector

```
