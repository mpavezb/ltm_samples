#!/bin/bash

total_ram_kb=$(grep MemTotal /proc/meminfo | awk '{print $2}')
total_ram_gb=$(python -c "gb = ${total_ram_kb} / (1024.0 * 1024.0); print('%.1f' % gb)")

echo "Machine Information:"
echo " - CPU : $(grep 'model name' /proc/cpuinfo | head -n1 | cut -d: -f2 | xargs)"
echo " - RAM : ${total_ram_gb} GB (${total_ram_kb} kB)"
echo " - Disk: "
df -h | head -n1 | sed 's/^/     /'
df -h | grep '^/dev/sd' | grep -E '\ /$|\ /home$' | sed 's/^/     /'
