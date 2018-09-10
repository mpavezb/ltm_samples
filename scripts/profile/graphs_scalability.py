#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import csv
import matplotlib.pyplot as plt
import rospkg

version = "v2"
base_folder = rospkg.RosPack().get_path('ltm_samples') + "/scripts/profile/"
results_folder = base_folder + "results/"
graphs_folder = base_folder + "graph/"
results_file = results_folder + "scalability_ouput_" + version + ".csv"


stamps = []
mongo_cpu = []
mongo_ram = []
ltm_cpu = []
ltm_ram = []
total_cpu = []
total_ram = []

iterations = list()
episodes = list()
elapsed_time = list() # [s]
time_in_avg = list()  # [ms]
time_q1_avg = list()  # [ms]
time_q2_avg = list()  # [ms]
time_q3_avg = list()  # [ms]
time_q4_avg = list()  # [ms]
time_q5_avg = list()  # [ms]
mongo_documents = list()
mongo_data_size = list()
mongo_idx_size = list()
mongo_stg_size = list()
mongo_file_size = list()
mongo_docs_and_indices = list()

with open(results_file, 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in reader:
        it = int(float(row[0]))
        n_eps = int(float(row[1]))
        e_time = int(float(row[2]))
        t_insertion = float(row[3])/(1000.0)
        t_q1 = float(row[4])/(1000.0)
        t_q2 = float(row[5])/(1000.0)
        t_q3 = float(row[6])/(1000.0)
        t_q4 = float(row[7])/(1000.0)
        t_q5 = float(row[8])/(1000.0)
        m_docs = int(float(row[9]))
        m_datasize = float(row[10])/(1024.0*1024.0*1024.0)
        m_indexsize = float(row[11])/(1024.0*1024.0*1024.0)
        m_storagesize = float(row[12])/(1024.0*1024.0*1024.0)
        m_filesize = float(row[13])/(1024.0*1024.0*1024.0)

        if n_eps > 1000*1000:
            break
        whitelisted = [0, 500, 1000, 5000]
        if n_eps not in whitelisted and n_eps % 10000 != 0:
            continue

        iterations.append(it)
        episodes.append(n_eps/1000.0)
        elapsed_time.append(e_time)
        time_in_avg.append(t_insertion)
        time_q1_avg.append(t_q1)
        time_q2_avg.append(t_q2)
        time_q3_avg.append(t_q3)
        time_q4_avg.append(t_q4)
        time_q5_avg.append(t_q5)
        mongo_documents.append(m_docs)
        mongo_data_size.append(m_datasize)
        mongo_idx_size.append(m_indexsize)
        mongo_stg_size.append(m_storagesize)
        mongo_file_size.append(m_filesize)

def gen_formatted_ax():
    fig, ax = plt.subplots()
    ax.grid()

    # # formatter function takes tick label and tick position
    # def func(x, pos):
    #     s = str(x)
    #     ind = s.index('.')
    #     return s[:ind] + ',' + s[ind+1:]   # change dot to comma

    # ax.get_xaxis().set_major_formatter(
    #     plt.FuncFormatter(lambda x, p: "{:,}".format(int(x)).replace(',', '.')))
    # ax.get_yaxis().set_major_formatter(
    #     plt.FuncFormatter(func))

    return ax

def graph_operation_times(extended=False):
    # time graphs
    ax = gen_formatted_ax()
    ax.plot(episodes, time_in_avg, '+k-', markersize=6, linewidth=3, label=u'Inserción')
    ax.plot(episodes, time_q1_avg, 'C0.-',  markersize=6, linewidth=1, label=u'Q1')
    ax.plot(episodes, time_q2_avg, 'C1.-',  markersize=6, linewidth=1, label=u'Q2')
    ax.plot(episodes, time_q3_avg, 'C2.-',  markersize=6, linewidth=1, label=u'Q3')
    ax.plot(episodes, time_q4_avg, 'C3.-',  markersize=6, linewidth=1, label=u'Q4')
    ax.plot(episodes, time_q5_avg, 'C4.-',  markersize=6, linewidth=1, label=u'Q5')
    suffix = "_extended"
    if not extended:
        plt.ylim([-0.1, 2.0])
        suffix = ""

    ax.legend(loc='upper left', shadow=True)
    plt.xlabel('episodios [miles]')
    plt.ylabel('tiempo [s]')
    plt.title(u'Duración de operaciones según cantidad de episodios')
    plt.savefig(graphs_folder + "scalability_operation_time" + suffix + ".eps", format="eps", dpi=1000)
    # plt.show()
    plt.close()

def graph_max_qpm():
    ax = gen_formatted_ax()
    dx = 0.000001
    in_max_qpm = [60.0/(x+dx) for x in time_in_avg]
    q1_max_qpm = [60.0/(x+dx) for x in time_q1_avg]
    q2_max_qpm = [60.0/(x+dx) for x in time_q2_avg]
    q3_max_qpm = [60.0/(x+dx) for x in time_q3_avg]
    q4_max_qpm = [60.0/(x+dx) for x in time_q4_avg]
    q5_max_qpm = [60.0/(x+dx) for x in time_q5_avg]
    ax.plot(episodes, in_max_qpm, '+k-', markersize=6, linewidth=3, label=u'Inserción')
    ax.plot(episodes, q1_max_qpm, 'C0.-',  markersize=6, linewidth=1, label=u'Q1')
    ax.plot(episodes, q2_max_qpm, 'C1.-',  markersize=6, linewidth=1, label=u'Q2')
    ax.plot(episodes, q3_max_qpm, 'C2.-',  markersize=6, linewidth=1, label=u'Q3')
    ax.plot(episodes, q4_max_qpm, 'C3.-',  markersize=6, linewidth=1, label=u'Q4')
    ax.plot(episodes, q5_max_qpm, 'C4.-',  markersize=6, linewidth=1, label=u'Q5')
    plt.ylim([0, 600])
    plt.xlim(xmin=0)


    ax.legend(loc='upper right', shadow=True)
    plt.xlabel('episodios [miles]')
    plt.ylabel('CPM')
    plt.title(u'CPM según cantidad de episodios (cota superior)')
    plt.savefig(graphs_folder + "scalability_max_qpm.eps", format="eps", dpi=1000)
    # plt.show()
    plt.close()

def graph_disk_usage():
    ax = gen_formatted_ax()
    ax.plot(episodes, mongo_file_size, 'r-', markersize=1, linewidth=3, label=u'Memoria Secundaria')
    ax.plot(episodes, mongo_stg_size,  '.k-', markersize=4, linewidth=1, label=u'Espacio Reservado')
    ax.plot(episodes, mongo_data_size, '.-',  markersize=6, linewidth=1, label=u'Episodios')
    ax.plot(episodes, mongo_idx_size,  '.-',  markersize=6, linewidth=1, label=u'Índices')
    ax.legend(loc='upper left', shadow=True)
    plt.xlabel('Episodios [miles]')
    plt.ylabel('Uso de disco [GB]')
    plt.title(u'Uso de disco según cantidad de episodios')
    plt.savefig(graphs_folder + "scalability_disk_usage.eps", format="eps", dpi=1000)
    # plt.show()
    plt.close()

graph_operation_times(extended=False)
graph_operation_times(extended=True)
graph_disk_usage()
graph_max_qpm()
