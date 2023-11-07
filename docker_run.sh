#!/bin/bash

set -e

# Default settings
CUDA="off"
ROS_DISTRO="melodic"
USER_ID="$(id -u)"

function usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "    -b,--base-only <AUTOWARE_HOST_DIR> If provided, run the base image only and mount the provided Autoware folder."
    echo "                                       Default: Use pre-compiled Autoware image"
    echo "    -c,--cuda <on|off>                 Enable Cuda support in the Docker."
    echo "                                       Default: $CUDA"
    echo "    -h,--help                          Display the usage and exit."
    echo "    -i,--image <name>                  Set docker images name."
    echo "                                       Default: $IMAGE_NAME"
    echo "    -p,--pre-release <on|off>          Use pre-release image."
    echo "                                       Default: $PRE_RELEASE"
    echo "    -r,--ros-distro <name>             Set ROS distribution name."
    echo "                                       Default: $ROS_DISTRO"
    echo "    -s,--skip-uid-fix                  Skip uid modification step required when host uid != 1000"
    echo "    -t,--tag-prefix <tag>              Tag prefix use for the docker images."
    echo "                                       Default: $TAG_PREFIX"
}

OPTS=`getopt --options b:c:hi:p:r:st: \
         --long base-only:,cuda:,help,image-name:,pre-release:,ros-distro:,skip-uid-fix,tag-prefix: \
         --name "$0" -- "$@"`
eval set -- "$OPTS"

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

SHARED_DOCKER_DIR=/home/
SHARED_HOST_DIR=/home/whale/code/driver/chassis-perf

# AUTOWARE_DOCKER_DIR=/home/autoware/Autoware

VOLUMES="--volume=$XSOCK:$XSOCK:rw
         --volume=$XAUTH:$XAUTH:rw
         --volume=$SHARED_HOST_DIR:$SHARED_DOCKER_DIR:rw"

# if [ "$BASE_ONLY" == "true" ]; then
#     SUFFIX=$SUFFIX"-base"
#     VOLUMES="$VOLUMES --volume=$AUTOWARE_HOST_DIR:/home "
# fi

DOCKER_VERSION=$(docker version --format '{{.Client.Version}}' | cut --delimiter=. --fields=1,2)
if [ $CUDA == "on" ]; then
    SUFFIX=$SUFFIX"-cuda"
    if [[ ! $DOCKER_VERSION < "19.03" ]] && ! type nvidia-docker; then
        RUNTIME="--gpus all"
    else
        RUNTIME="--runtime=nvidia"
    fi
fi

if [ $PRE_RELEASE == "on" ]; then
    SUFFIX=$SUFFIX"-rc"
fi

# Create the shared directory in advance to ensure it is owned by the host user
# mkdir -p $SHARED_HOST_DIR

# IMAGE=$IMAGE_NAME:$TAG_PREFIX-$ROS_DISTRO$SUFFIX
IMAGE=robest_melodic/ros-melodic-full:V1

echo "Launching $IMAGE"

docker run \
    -it --rm\
    -v ~/.ssh:/root/.ssh\
    $VOLUMES \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    --env="USER_ID=$USER_ID" \
    --privileged \
    --net=host \
    -w /home/\
    $RUNTIME \
    $IMAGE

#     -it -rm \ -rm 参数用于在容器退出后自动删除容器

# [perf] judge if or no use cuda. but it is't by test. 
#!/bin/bash

# set -e

# 默认设置
# CUDA="off"
# IMAGE_NAME="fishros2/ros"
# TAG_PREFIX="melodic-desktop-full"
# ROS_DISTRO="melodic"
# BASE_ONLY="false"
# PRE_RELEASE="off"
# AUTOWARE_HOST_DIR="/home/whale/RBD_ws/RoBest_Car/beilu_ws"
# USER_ID=$(id -u)

# function usage() {
#     echo "Usage: $0 [OPTIONS]"
#     # ...省略其他选项...
# }

# # Convert a relative directory path to absolute
# function abspath() {
#     local path=$1
#     if [ ! -d $path ]; then
#     exit 1
#     fi
#     pushd $path > /dev/null
#     echo $(pwd)
#     popd > /dev/null
# }

# # 检查CUDA是否可用
# function check_cuda() {
#     if lspci | grep -q 'NVIDIA'; then
#         if nvidia-smi > /dev/null 2>&1; then
#             CUDA="on"
#         fi
#     fi
# }

# check_cuda
