#! /bin/bash

# 获取脚本文件的绝对路径
script_path="$(readlink -f "$0")"
# 获取脚本所在文件夹的绝对路径
script_dir_absolute="$(dirname "$script_path")"
# echo "The absolute path of the script is: $script_path"
# echo "The absolute path of the script's folder is: $script_dir_absolute"
name="$(git config --global user.name)"
email="$(git config --global user.email)"

echo "package dir: $script_dir_absolute"
echo "package name: $1"
echo "maintainer: email=$email, name=$name"

if [ "$#" -lt 1 ]; then
    echo "paramter need more than 1"
    exit 1
elif [ "$#" -lt 2 ]; then
    echo "Your command: ros2 pkg create --maintainer-name $name --maintainer-email $email  $1"
    echo "--------------------"
    cd "$script_dir_absolute" && \
        ros2 pkg create \
        --maintainer-name "$name" \
        --maintainer-email "$email" "$1"
else
    echo "Your dependences: ${*:2}"
    echo "Your command: pkg create $1 --maintainer-name $name --maintainer-email $email --dependencies ${*:2} "
    echo "--------------------"
    # shellcheck disable=SC2086
    # shellcheck disable=SC2048
    cd "$script_dir_absolute" && \
        ros2 pkg create \
        "$1" \
        --maintainer-name "$name" \
        --maintainer-email "$email" \
        --dependencies ${*:2}         
fi
