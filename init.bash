#!/usr/bin/env bash

echo "Checkout submodules..."
git submodule init
git submodule update
echo "Done."
echo "Updating rosplan dependencies, this may take a few minutes..."
cd src/ROSPlan
sed -i "s/github.com\/KCL-Planning/gitee.com\/yuanzhouxue/g" .gitmodules
sed -i "s/github.com\/gerardcanal\/PPDDL_determinization/gitee.com\/yuanzhouxue\/ppddl_parser/g" .gitmodules
sed -i "s/github.com\/gerardcanal/gitee.com\/yuanzhouxue/g" .gitmodules
sed -i "s/github.com\/ssanner/gitee.com\/yuanzhouxue/g" .gitmodules
sed -i "s/github.com\/oscar-lima/gitee.com\/yuanzhouxue/g" .gitmodules
sed -i "s/github.com\/sebastianstock/gitee.com\/yuanzhouxue/g" .gitmodules
git submodule sync
git submodule init
git submodule update
git commit -a "update submodule url"
echo "Done."

echo "Installing ROS depedencies..."
sudo apt update
sudo apt install ros-melodic-people-msgs ros-melodic-moveit-ros-planning-interface ros-melodic-four-wheel-steering-msgs ros-melodic-urdf-geometry-parser ros-melodic-base-local-planner ros-melodic-tf2-bullet ros-melodic-move-base-msgs bison flex -y
echo "Done."

echo -e "\033[32m All things done, you can work further. \033[0m"
