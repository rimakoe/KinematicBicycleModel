#!/bin/bash

package_names=("bicycle-model")

for package_name in ${package_names[@]}; do
  echo ""
  echo $package_name
  sudo find ./catkin_ws/src/$package_name/scripts -type f -name '*.py'
  sudo find ./catkin_ws/src/$package_name/scripts -type f -name '*.py' -exec chmod +x {} \;
  git -C ./catkin_ws/src/$package_name config core.fileMode false
done
