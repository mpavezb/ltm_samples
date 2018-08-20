#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import csv
import matplotlib.pyplot as plt

folder = "/home/mpavezb/workspaces/memoria/ltm_ws/src/ltm_samples/scripts/profile"
file = folder + "/profile_cpu_ram.log"

stamps = []
mongo_cpu = []
mongo_ram = []
ltm_cpu = []
ltm_ram = []
total_cpu = []
total_ram = []

with open(file, 'rb') as csvfile:
	reader = csv.reader(csvfile, delimiter=' ', quotechar='|')
	for row in reader:
		stamps.append(row[0])

		mcpu = float(row[2])
		mram = float(row[3])
		lcpu = float(row[5])
		lram = float(row[6])

		mongo_cpu.append(mcpu)
		mongo_ram.append(mram)
		ltm_cpu.append(lcpu)
		ltm_ram.append(lram)
		total_cpu.append(mcpu + lcpu)
		total_ram.append(mram + lram)

# counter = range(len(stamps))

# USO CPU
fig, ax = plt.subplots()
ax.plot(ltm_cpu, '.k')
ax.plot(mongo_cpu, '.k')
ax.plot(total_cpu, '.k')
ax.plot(ltm_cpu, 'k--', label='LTM')
ax.plot(mongo_cpu, 'k:', label='MongoDB')
ax.plot(total_cpu, 'k', label='Total')
ax.legend(loc='upper right', shadow=True)
ax.grid()
plt.axis([0, len(ltm_cpu)-1, 0, 20])
plt.xlabel('tiempo [s]')
plt.ylabel('% [0, 100]')
plt.title('Uso de CPU por los servidores LTM y de MongoDB')
plt.savefig("graph/cpu_ram.eps", format="eps", dpi=1000)
# plt.show()


# # USO RAM
# fig, ax = plt.subplots()
# ax.plot(ltm_cpu, 'k--', label='LTM')
# ax.plot(mongo_cpu, 'k:', label='MongoDB')
# ax.plot(total_cpu, 'k', label='Total')
# ax.legend(loc='upper right', shadow=True)
# ax.grid()
# plt.xlabel('tiempo [s]')
# plt.ylabel('% [0, 100]')
# plt.title('Uso de RAM por los servidores LTM y de MongoDB')
# plt.show()
# ax.plot(ltm_ram, 'k--', label='LTM %ram')
# ax.plot(mongo_ram, 'k:', label='mongodb %ram')
# ax.plot(total_ram, 'k', label='total %ram')
