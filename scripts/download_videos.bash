#!/bin/bash

PACKAGE_DIR="$(rospack find ltm_samples)"
DEST_DIR="${PACKAGE_DIR}/samples/video"

# get videos from https://www.sample-videos.com/
declare -a arr=(
    "https://www.sample-videos.com/video/mp4/720/big_buck_bunny_720p_1mb.mp4"
    "https://www.sample-videos.com/video/mp4/720/big_buck_bunny_720p_2mb.mp4"
    "https://www.sample-videos.com/video/mp4/720/big_buck_bunny_720p_5mb.mp4"
    "https://www.sample-videos.com/video/mp4/720/big_buck_bunny_720p_10mb.mp4"
    "https://www.sample-videos.com/video/mp4/720/big_buck_bunny_720p_20mb.mp4"
    "https://www.sample-videos.com/video/mp4/720/big_buck_bunny_720p_30mb.mp4"
    "https://www.sample-videos.com/video/mp4/480/big_buck_bunny_480p_1mb.mp4"
    "https://www.sample-videos.com/video/mp4/480/big_buck_bunny_480p_2mb.mp4"
    "https://www.sample-videos.com/video/mp4/480/big_buck_bunny_480p_5mb.mp4"
    "https://www.sample-videos.com/video/mp4/480/big_buck_bunny_480p_10mb.mp4"
    "https://www.sample-videos.com/video/mp4/480/big_buck_bunny_480p_20mb.mp4"
    "https://www.sample-videos.com/video/mp4/480/big_buck_bunny_480p_30mb.mp4"
    "https://www.sample-videos.com/video/mp4/360/big_buck_bunny_360p_1mb.mp4"
    "https://www.sample-videos.com/video/mp4/360/big_buck_bunny_360p_2mb.mp4"
    "https://www.sample-videos.com/video/mp4/360/big_buck_bunny_360p_5mb.mp4"
    "https://www.sample-videos.com/video/mp4/360/big_buck_bunny_360p_10mb.mp4"
    "https://www.sample-videos.com/video/mp4/360/big_buck_bunny_360p_20mb.mp4"
    "https://www.sample-videos.com/video/mp4/360/big_buck_bunny_360p_30mb.mp4"
    "https://www.sample-videos.com/video/mp4/240/big_buck_bunny_240p_1mb.mp4"
    "https://www.sample-videos.com/video/mp4/240/big_buck_bunny_240p_2mb.mp4"
    "https://www.sample-videos.com/video/mp4/240/big_buck_bunny_240p_5mb.mp4"
    "https://www.sample-videos.com/video/mp4/240/big_buck_bunny_240p_10mb.mp4"
    "https://www.sample-videos.com/video/mp4/240/big_buck_bunny_240p_20mb.mp4"
    "https://www.sample-videos.com/video/mp4/240/big_buck_bunny_240p_30mb.mp4"
)


# DESTINATION DIRECTORY
printf "Downloading videos into folder: ${DEST_DIR}\n"


GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# DOWNLOAD
for url in "${arr[@]}"
do
    FILENAME="${url##*/}"
    FULL_FILENAME=${DEST_DIR}/${FILENAME}

    # delete empty file if exists (this can happen when the file download failed)
    if [ -f ${FULL_FILENAME} ] && [ ! -s "${FULL_FILENAME}" ]; then
        printf "${YELLOW} - deleting empty video file: ${FILENAME}${NC}\n"
        rm -f ${FULL_FILENAME}
    fi

    # download
    if  [ ! -f ${FULL_FILENAME} ]; then
        printf "${YELLOW} - downloading: ${FILENAME}${NC}\n"
        wget ${url} -O ${FULL_FILENAME}
    else
        printf "${GREEN} - already exists: ${FILENAME}${NC}\n"
    fi
done


