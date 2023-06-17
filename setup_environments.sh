#!/bin/bash

#python3 -m pip install -r ./common_requirements.txt

package_names=("bicycle-model")

for package_name in ${package_names[@]}; do
  echo ""
  echo $package_name
  cd /workspaces/KinematicBicycleModel/catkin_ws/src/$package_name
  source ./setup_venv.sh
done

cd /workspaces/KinematicBicycleModel
