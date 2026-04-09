#!/bin/bash
project_path=$(cd `dirname $0`; pwd)
project_name="${project_path##*/}"
cd ~/linke_intelligent_car
catkin_make --pkg $project_name
cd -
