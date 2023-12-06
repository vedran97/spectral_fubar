# Project Spectral Fubar
![CICD Workflow status](https://github.com/vedran97/spectral_fubar/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/vedran97/spectral_fubar/branch/main/graph/badge.svg)](https://codecov.io/gh/vedran97/spectral_fubar)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Description

This project uses a Husky robot, mounted with a variety of vision sensors, for post natural disaster road inspection.
The Husky robot uses a depth camera, and object detection/localization algorithm to detect, localize an obstacle/debris seen on a road.

## Personnel

1. Vedant Ranade
2. Jerry Pittman
3. Aaqib Barodawala

## Environment

1. This repo is designed to be used with ROS2 Humble and Ubuntu 22.04

## AIP and Final Deliverables

1. Sprint Log is here: [LINK](https://docs.google.com/document/d/1LM4T2IaXjHfa725iT_XrGdjAjX6Vu2G_JORIrd1daMI/edit?usp=sharing)
2. AIP Work Logs are here: [LINK](https://docs.google.com/spreadsheets/d/1apXGEWr1nkUdqm7aSSiJ8yui3v2pkUTxmMRXO4ydDmY/edit?usp=sharing)
3. Final Deliverables are here: [LINK](https://drive.google.com/drive/folders/1iyCzRlYM_VHO__MNFvLk7GiNthm--4n8?usp=drive_link) (including proposal)

## Repository Setup instruction

1. clone this repository in your ros2 workspace's src folder by ```git clone https://github.com/vedran97/spectral_fubar.git```
2. Run ```rosdep install -i --from-path src --rosdistro humble -y``` in a command line in workspace root

## Build instructions

1. Build the workspace:```colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers $(nproc)```
2. Source the workspace: ```source install/setup.bash```

## Cpp-Tools instructions

```bash
# run clang-format from workspace root

  cd src/spectral_fubar && clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -vE -e "^(./build/|./install/|./log/)") && cd -

# run cppcheck from inside the pkg directory
  mkdir results -p && cppcheck --enable=all --std=c++17 -I include/ --suppress=missingInclude --inline-suppr $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) &> results/cppcheck

# run cpplint from inside the pkg directory

  mkdir results -p && cpplint  --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) &> results/cpplint

```

## Test instructions

1. Add steps to run tests

## Docs generation

1. Add steps to run documentation generation
