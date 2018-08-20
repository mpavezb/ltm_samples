#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
log_file="${THIS_DIR}/profile_cpu_ram_$(date +"%y-%m-%d_%H-%M-%S").log"
log_file_last="${THIS_DIR}/profile_cpu_ram.log"

ltm_process="ltm_server"
sample_period=2.0
# sample_frequency=2.0
# sample_period=$(python -c "print 1.0/${sample_frequency}")

echo ""
echo "CPU and RAM profiling for the LTM suite"
echo "======================================="

echo "Prerequisites:"
echo "--------------"
echo " - Checking LTM server process:"
if pgrep "${ltm_process}" >/dev/null; then
	ltm_pid=$(pgrep "${ltm_process}")
	echo "   ... LTM server process found with PID: ${ltm_pid}."
else
	echo "   ... Ups. LTM server process not found under name ${ltm_process}."
	exit 0
fi

echo " - Checking MongoDB server process:"
if ps -edaf | grep mongo | grep -v grep >/dev/null; then
	mongo_pid=$(ps -edaf | grep mongo | grep -v grep | awk '{print $2}')
	echo "   ... MongoDB server process found with PID: ${mongo_pid}."
else
	echo "   ... Ups. MongoDB server process not found."
	exit 0
fi

echo ""
echo "Information:"
echo "--------------"
echo " - Row data: timestamp %cpu %mem"
echo " - Formats : CPU and RAM are in range [0.0, 100]%"
echo "   - CPU and RAM: .2f precision values in range [0.0, 100]%."
echo "   - Timestamp  : %y-%m-%d_%H-%M-%S. e.g., 18-08-11_09-33-47"
echo ""

echo ""
echo "Working:"
echo "-------------"
# echo " - Sample Frequency: ${sample_frequency} [Hz]"
echo " - Sample Period   : ${sample_period} [s]"
echo " - Log file        : ${log_file} ..."
echo "   ... Log will be also saved into the .../profile_cpu_ram.log file."
rm -f "${log_file_last}" # clean last log file

# Use top command to gather LTM and MongoDB stats
# obs: grep --line-buffered, sed -u and awk '{system("")}' are used to avoid buffering
top -b -p "${ltm_pid}","${mongo_pid}" -d "${sample_period}" -o COMMAND | # sort by command name
	grep -E "ltm_server|mongo" --line-buffered      | # get interesting lines
	awk '{print $12, $9, $10; system("")}'          | # name,cpu,ram columns
	sed -u 'N;s/\n/ /'                              | # merge 2 (ONLY TWO) lines into one
	awk '{print strftime("%y-%m-%d_%H-%M-%S"), $0; system("")}' | # append timestamp
	tee -a "${log_file_last}" "${log_file}" # show on screen and save


# OBS: cant use the 'ps' command as it only shows average usage by each 
# process. Peaks and transient records will not be seen.
#
# while sleep ${sample_period}; do
# 	echo $(date +"%y-%m-%d_%H-%M-%S") $(ps -p $ltm_pid -o pcpu= -o pmem=) >> ${log_file}
# done
