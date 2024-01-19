#! /bin/bash

# 获取脚本文件的绝对路径
script_path="$(readlink -f "$0")"
# 获取脚本所在文件夹的绝对路径
script_dir_absolute="$(dirname "$script_path")/../"

 echo "The folder is: $script_dir_absolute"
cd $script_dir_absolute  && \
rm -rf build/ && rm -rf install/ && rm -rf log/