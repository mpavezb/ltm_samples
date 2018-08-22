#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import csv
import matplotlib.pyplot as plt
import rospkg

base_folder = rospkg.RosPack().get_path('ltm_samples') + "/scripts/profile/"
results_folder = base_folder + "results/"
graphs_folder = base_folder + "graph/"
results_file = results_folder + "ltm_scalability_ouput.csv"

stamps = []
mongo_cpu = []
mongo_ram = []
ltm_cpu = []
ltm_ram = []
total_cpu = []
total_ram = []

iterations = list()
episodes = list()
elapsed_time = list()       # [s]
time_insertion_avg = list() # [ms]
time_simple_avg = list()    # [ms]
time_complex_avg = list()   # [ms]
mongo_documents = list()
mongo_datasize = list()
mongo_indexsize = list()
mongo_storagesize = list()
mongo_filesize = list()
mongo_docs_and_indices = list()


with open(results_file, 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in reader:
        it = int(float(row[0]))
        n_eps = int(float(row[1]))
        e_time = int(float(row[2]))
        t_insertion = int(float(row[3]))
        t_search_s = int(float(row[4]))
        t_search_c = int(float(row[5]))
        m_docs = int(float(row[6]))
        m_datasize = int(float(row[7])/(1000.0*1000.0))
        m_indexsize = int(float(row[8])/(1000.0*1000.0))
        m_storagesize = int(float(row[9])/(1000.0*1000.0))
        m_filesize = int(float(row[10])/(1000.0*1000.0))

        if n_eps > 1000*1000:
            break
        whitelisted = [0, 500, 1000, 5000]
        if n_eps not in whitelisted and n_eps % 10000 != 0:
            continue

        iterations.append(it)
        episodes.append(n_eps)
        elapsed_time.append(e_time)
        time_insertion_avg.append(t_insertion)
        time_simple_avg.append(t_search_s)
        time_complex_avg.append(t_search_c)
        mongo_documents.append(m_docs)
        mongo_datasize.append(m_datasize)
        mongo_indexsize.append(m_indexsize)
        mongo_storagesize.append(m_storagesize)
        mongo_filesize.append(m_filesize)


def gen_formatted_ax():
    fig, ax = plt.subplots()
    ax.grid()
    ax.get_xaxis().set_major_formatter(
        plt.FuncFormatter(lambda x, p: "{:,}".format(int(x)).replace(',', '.')))
    ax.get_yaxis().set_major_formatter(
        plt.FuncFormatter(lambda x, p: "{:,}".format(int(x)).replace(',', '.')))

    return ax

def graph_operation_times():
    # time graphs
    ax = gen_formatted_ax()
    ax.plot(episodes, time_insertion_avg, '.k')
    ax.plot(episodes, time_simple_avg, '.k')
    ax.plot(episodes, time_complex_avg, '.k')
    ax.plot(episodes, time_insertion_avg, 'k--', label=u'Inserción')
    ax.plot(episodes, time_simple_avg, 'k:', label=u'Búsqueda Simple')
    ax.plot(episodes, time_complex_avg, 'k', label=u'Búsqueda Compleja')
    ax.legend(loc='upper left', shadow=True)
    plt.xlabel('episodios')
    plt.ylabel('tiempo [ms]')
    plt.title(u'Costo de operaciones según cantidad de episodios')
    plt.savefig(graphs_folder + "scalability_operation_time.eps", format="eps", dpi=1000)
    # plt.show()

def graph_disk_usage():
    ax = gen_formatted_ax()
    ax.plot(episodes, mongo_filesize, '.k')
    ax.plot(episodes, mongo_storagesize, '.k')
    ax.plot(episodes, mongo_datasize, '.k')
    ax.plot(episodes, mongo_indexsize, '.k')
    ax.plot(episodes, mongo_filesize, 'k-.', label=u'Mem. Secundaria')
    ax.plot(episodes, mongo_storagesize, 'k--', label=u'Espacio Reservado')
    ax.plot(episodes, mongo_datasize, 'k-', label=u'Episodios')
    ax.plot(episodes, mongo_indexsize, 'k:', label=u'Índices')
    ax.legend(loc='upper left', shadow=True)
    plt.xlabel('episodios')
    plt.ylabel('uso de disco [MB]')
    plt.title(u'Uso de disco según cantidad de episodios')
    plt.savefig(graphs_folder + "scalability_disk_usage.eps", format="eps", dpi=1000)
    # plt.show()

graph_operation_times()
graph_disk_usage()
