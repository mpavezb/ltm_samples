#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import copy
import csv
import matplotlib.pyplot as plt
import rospkg

N_CPUS = 4
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
        self.ram_MB_avg = list()
        self.ram_GB_avg = list()

    def update(self, sample):
        self.cpu_max.append(float(sample[0])/(N_CPUS+0.0))
        self.cpu_avg.append(float(sample[1])/(N_CPUS+0.0))
        self.ram_avg.append(float(sample[2]))
        self.ram_kB_avg.append(float(sample[3]))
        self.ram_MB_avg.append(float(sample[3])/1024.0)
        self.ram_GB_avg.append(float(sample[3])/(1024.0*1024.0))

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

    def __getitem__(self, key):
        return getattr(self, key)

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


queries = ['q1', 'q2', 'q3', 'q4', 'q5']
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


def gen_formatted_ax(scale=1.0):
    fig, ax = plt.subplots()
    ax.grid()
    # ax.get_xaxis().set_major_formatter(
    #     plt.FuncFormatter(lambda x, p: "{:,}".format(int(x)).replace(',', '.')))
    # ax.get_yaxis().set_major_formatter(
    #     plt.FuncFormatter(lambda x, p: "{:,}".format(x).replace(',', '.')))
    size_x = fig.get_size_inches()[0]
    size_y = fig.get_size_inches()[1]
    fig.set_size_inches(scale * size_x, scale*size_y)
    return fig, ax
  
def graph_cpu_vs_qpm_by_query(n_ep, attr):
    # fixed episode
    fig, ax = gen_formatted_ax(scale=0.75)
    data = results[n_ep]
    ax.plot(data.qpm, data.q1[attr].cpu_avg, 'C0.-', label="Q1")
    ax.plot(data.qpm, data.q2[attr].cpu_avg, 'C1.-', label="Q2")
    ax.plot(data.qpm, data.q3[attr].cpu_avg, 'C2.-', label="Q3")
    ax.plot(data.qpm, data.q4[attr].cpu_avg, 'C3.-', label="Q4")
    ax.plot(data.qpm, data.q5[attr].cpu_avg, 'C4.-', label="Q5")
    plt.ylim(ymax=30)

    title_preffix = attr.upper()
    if attr is "sum":
        title_preffix = "Sistema LTM"
    if attr is "ltm":
    	title_preffix = "LTMp"
    if attr is "mongo":
    	title_preffix = "MongoDBp"


    ax.legend(loc='upper left', shadow=True)
    plt.xlabel('CPM')
    plt.ylabel('uso de CPU [%]')
    plt.title(title_preffix + u": Uso de CPU según CPM (" + str(n_ep) + " eps)")
    plt.savefig(graphs_folder + "eff__cpu_qpm_by_query__" + attr + "__" + str(n_ep) + "_episodes.eps", format="eps", dpi=1000)
    # plt.show()
    plt.close()

def graph_cpu_vs_qpm_by_eps(q, attr):
    # fixed query
    fig, ax = gen_formatted_ax(scale=0.75)

    for n_ep in episodes:
        data = results[n_ep]
        label = str(n_ep) + " eps"
        query = getattr(data, q)
        ax.plot(data.qpm, query[attr].cpu_avg, '.-', label=label)
    plt.ylim(ymax=30)

    title_preffix = attr.upper()
    leg_loc = 'upper left'
    if attr is "sum":
        title_preffix = "Sistema LTM"
        if q in ["q3", "q4"]:
            leg_loc = 'upper right'
    if attr is "ltm":
    	title_preffix = "LTMp"
    if attr is "mongo":
    	title_preffix = "MongoDBp"
        
    ax.legend(loc=leg_loc, shadow=True)
    plt.xlabel(u'CPM')
    plt.ylabel(u'uso de CPU [%]')
    plt.title("[" + q.upper() + "] " + title_preffix + u": Uso de CPU según CPM")
    plt.savefig(graphs_folder + "eff_cpu_qpm_by_eps__" + attr + "__" + q + ".eps", format="eps", dpi=1000)
    # plt.show()
    plt.close()

def graph_ram_vs_qpm_by_eps():
    # fixed query
    fig, ax = gen_formatted_ax()
    for n_ep in episodes:
        result = results[n_ep]
        label = str(n_ep) + " episodios"
        query = getattr(result, 'q1')

        x_axis = result.qpm
        # y_axis = query.sum.ram_avg
        y_axis = query.sum.ram_MB_avg
        ax.plot(x_axis, y_axis, '.-', label=label)
        
    ax.legend(loc='upper left', shadow=True)
    plt.xlabel(u'CPM')
    plt.ylabel(u'uso de RAM [MB]')
    plt.title(u"Uso de RAM según CPM")
    plt.savefig(graphs_folder + "eff__ram_qpm.eps", format="eps", dpi=1000)
    # plt.show()
    plt.close()

def graph_cpu_vs_qpm_by_query_all():
    attrs = ["sum", "mongo", "ltm"]
    for n_ep in episodes:
        for attr in attrs:
            graph_cpu_vs_qpm_by_query(n_ep, attr)

def graph_cpu_vs_qpm_by_eps_all():
    attrs = ["sum", "mongo", "ltm"]
    for q in queries:
        for attr in attrs:
            graph_cpu_vs_qpm_by_eps(q, attr)

graph_ram_vs_qpm_by_eps()
graph_cpu_vs_qpm_by_eps_all()
graph_cpu_vs_qpm_by_query_all()