#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import copy
import csv
import matplotlib.pyplot as plt
import rospkg

base_folder = rospkg.RosPack().get_path('ltm_samples') + "/scripts/profile/"
results_folder = base_folder + "results/"
graphs_folder = base_folder + "graph/"
results_file = results_folder + "efficiency_ouput.csv"

class Measurement(object):
    def __init__(self):
        self.cpu_max = list()
        self.cpu_avg = list()
        self.ram_avg = list()
        self.ram_kB_avg = list()

    def update(self, sample):
        self.cpu_max.append(float(sample[0])/4.0)
        self.cpu_avg.append(float(sample[1])/4.0)
        self.ram_avg.append(float(sample[2]))
        self.ram_kB_avg.append(float(sample[3])/1024.0)

class QueryResult(object):
    def __init__(self):
        self.ltm = Measurement()
        self.mongo = Measurement()
        self.sum = Measurement()

    def update(self, sample):
        ltm = [float(i) for i in sample[0:4]]
        mongo = [float(i) for i in sample[4:8]]
        self.ltm.update(ltm)
        self.mongo.update(mongo)
        self.sum.update([sum(x) for x in zip(ltm, mongo)])

class EpisodeResults(object):
    def __init__(self, episodes):
        self.episodes = episodes
        self.qpm = list()
        self.elapsed_seconds = list()
        self.q1 = QueryResult()
        self.q2 = QueryResult()
        self.q3 = QueryResult()
        self.q4 = QueryResult()
        self.q5 = QueryResult()

    def update(self, sample):
        self.qpm.append(int(float(sample[2])))
        self.q1.update(sample[4:12])
        self.q2.update(sample[12:20])
        self.q3.update(sample[20:28])
        self.q4.update(sample[28:36])
        self.q5.update(sample[36:44])

episodes = list()
results = dict()
with open(results_file, 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in reader:
        row_count_ = int(float(row[0]))
        n_ep = int(float(row[1]))
        if n_ep not in results:
            episodes.append(n_ep)
            results[n_ep] = EpisodeResults(n_ep)
        results[n_ep].update(row)


def gen_formatted_ax():
    fig, ax = plt.subplots()
    ax.grid()
    # ax.get_xaxis().set_major_formatter(
    #     plt.FuncFormatter(lambda x, p: "{:,}".format(int(x)).replace(',', '.')))
    # ax.get_yaxis().set_major_formatter(
    #     plt.FuncFormatter(lambda x, p: "{:,}".format(x).replace(',', '.')))
    return ax
  

def graph_cpu_vs_qpm(q):
    ax = gen_formatted_ax()

    idx = 0
    colors = ['k', 'c', 'r', 'm', 'b']
    for n_ep in episodes:
        result = results[n_ep]
        label = str(n_ep) + " episodios"
        query = getattr(result, q)

        x_axis = result.qpm
        # y_axis = query.sum.ram_avg # OK
        # y_axis = query.sum.ram_kB_avg # OK
        # y_axis = query.mongo.cpu_avg # OK
        # y_axis = query.ltm.cpu_avg # OK
        # y_axis = query.sum.cpu_avg # OK
        fmt = colors[idx]
        # ax.plot(x_axis, y_axis, idx)
        ax.plot(x_axis, y_axis, fmt, label=label)
        idx += 1
        
    ax.legend(loc='upper left', shadow=True)
    plt.xlabel('Consultas por minuto')
    plt.ylabel('Uso de CPU [%]')
    plt.title(q.upper() + ": Uso de CPU vs. consultas por minuto")
    plt.savefig(graphs_folder + "efficiency_cpu_qpm_" + q + ".eps", format="eps", dpi=1000)
    # plt.show()

graph_cpu_vs_qpm('q1')
graph_cpu_vs_qpm('q2')
graph_cpu_vs_qpm('q3')
graph_cpu_vs_qpm('q4')
graph_cpu_vs_qpm('q5')


# PLOtS:
# - SUM, RAM AVG
# - 
#
#
#
#
#
#
# def graph_operation_times():
#     # time graphs
#     ax = gen_formatted_ax()
#     # sns.scatterplot(episodes, time_simple_avg, palette=sns.color_palette("GnBu_d"))
#     # sns.scatterplot(episodes, time_insertion_avg, palette=sns.color_palette("GnBu_d"))
#     # sns.scatterplot(episodes, time_complex_avg, palette=sns.color_palette("GnBu_d"))
#     ax.plot(episodes, time_simple_avg, '.k')
#     ax.plot(episodes, time_insertion_avg, '.k')
#     ax.plot(episodes, time_complex_avg, '.k')
#     ax.plot(episodes, time_simple_avg, 'k:', label=u'Búsqueda Simple')
#     ax.plot(episodes, time_insertion_avg, 'k--', label=u'Inserción')
#     ax.plot(episodes, time_complex_avg, 'k', label=u'Búsqueda Compleja')
#     ax.legend(loc='upper left', shadow=True)
#     plt.xlabel('episodios [miles]')
#     plt.ylabel('tiempo [s]')
#     plt.title(u'Costo de operaciones según cantidad de episodios')
#     plt.savefig(graphs_folder + "scalability_operation_time.eps", format="eps", dpi=1000)
#     # plt.show()
