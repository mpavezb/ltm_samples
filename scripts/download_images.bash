#!/bin/bash

PKG_DIR="$(rospack find ltm_samples)"
IMG_DIR="${PKG_DIR}/samples/images"

function print_green {
    local string="$1"
    local green='\033[0;32m'
    local no_color='\033[0m'
    printf "${green}${string}${no_color}\n"
}

function print_yellow {
    local string="$1"
    local yellow='\033[1;33m'
    local no_color='\033[0m'
    printf "${yellow}${string}${no_color}\n"
}

function download_set {
    local _type="$1"
    local _n="$2"
    local _url="$3"

    local _filename _full_filename
    local _target_dir=${IMG_DIR}/${_type}
    local _stat _number
    mkdir -p ${_target_dir}

    print_green "\n= = = = = = = = = = = = = = = = ="
    print_green "Downloading <${_type}> images into:"
    print_green " - ${_target_dir}"
    print_green "= = = = = = = = = = = = = = = = ="

    for (( i=0; i<${_n}; i++ ));
    do
        _number=$(( i + 1 ))
        _stat="${_number}/${_n}"
        _filename=${_number}.jpg
        _full_filename=${_target_dir}/${_filename}

        # delete empty file if exists (this can happen when the download failed)
        if [ -f ${_full_filename} ] && [ ! -s "${_full_filename}" ]; then
            print_yellow  " - ${_stat}: deleting empty <${_type}> image: ${_filename}"
            rm -f ${_full_filename}
        fi

        # download
        if [ ! -f ${_full_filename} ]; then
            print_yellow  " - ${_stat}: downloading <${_type}> to: ${_filename}"
            wget ${_url} -O ${_full_filename}
        else
            print_green " - ${_stat}: <${_type}> already exists: ${_filename}"
        fi
    done
}

download_set "face"   20  "https://loremflickr.com/320/240/face"
download_set "body"   20  "https://loremflickr.com/320/240/human"
download_set "object" 20  "https://loremflickr.com/320/240/food"
download_set "robot"  10  "https://loremflickr.com/320/240/robot"
download_set "place"  20  "https://loremflickr.com/320/240/kitchen,bedroom"
