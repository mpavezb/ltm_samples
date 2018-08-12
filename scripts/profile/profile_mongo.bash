#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
log_file="${THIS_DIR}/profile_mongo_$(date +"%y-%m-%d_%H-%M-%S").log"
log_file_last="${THIS_DIR}/profile_mongo.log"

# FIX JSON FORMAT WHEN CTRL IS ISSUED
# trap ctrl-c and call ctrl_c()
trap ctrl_c INT
function fix_file() {
	local file=$1
	if [ ! -f "${file}" ]; then
		echo "Log file does not exists: ${file}"
		return;
	fi
	head -n -1 "${file}" > "${file}.tmp" # omit last line
	echo "]" >> "${file}".tmp            # append closing character
	mv "${file}".tmp "${file}"
	echo "Log file fixed: ${file}"
}
function ctrl_c() {
	echo "** Trapped CTRL-C ... Fixing log files..."
	fix_file "${log_file_last}"
	fix_file "${log_file}"
}

# clean last log file
rm -f "${log_file_last}"

# gather information
mongo --quiet ltm_db "${THIS_DIR}"/profile_mongo.js | # ejecute mongo analyzer
	tee -a "${log_file_last}"                | # save to last file
	# tee -a "${log_file}"                     | # save to timestamped file
	grep -E "iteration|time" --line-buffered | # display it and timestamp
	sed -u 'N;s/\n/ /'                         # in one line
