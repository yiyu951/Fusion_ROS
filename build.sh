#! /bin/bash
colcon build \
--symlink-install \
--packages-up-to fusion_bringup \
--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 