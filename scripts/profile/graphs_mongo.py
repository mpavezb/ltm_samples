#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import json
from pprint import pprint
import matplotlib.pyplot as plt
import numpy as np

# -------------------------------------------------------------------
# required to load json files with python strings instead of unicode.
# -------------------------------------------------------------------
def json_load_byteified(file_handle):
    return _byteify(
        json.load(file_handle, object_hook=_byteify),
        ignore_dicts=True
    )

def _byteify(data, ignore_dicts=False):
    # if this is a unicode string, return its string representation
    if isinstance(data, unicode):
        return data.encode('utf-8')
    # if this is a list of values, return list of byteified values
    if isinstance(data, list):
        return [_byteify(item, ignore_dicts=True) for item in data]
    # if this is a dictionary, return dictionary of byteified keys and values
    # but only if we haven't already byteified it
    if isinstance(data, dict) and not ignore_dicts:
        return {
            _byteify(key, ignore_dicts=True): _byteify(value, ignore_dicts=True)
            for key, value in data.iteritems()
        }
    # if it's anything else, return it in its original form
    return data
    # -------------------------------------------------------------------

def load_json(filename):
    return json_load_byteified(open(filename))


folder = "/home/mpavezb/workspaces/memoria/ltm_ws/src/ltm_samples/scripts/profile"
file = folder + "/profile_mongo.log"


data = load_json(file)

n_values = len(data)


counter = []
time = []

db_data_size = []
# db_storage_size = []
# db_file_size = []

ep_count = []
ep_size = []
# ep_storage_size = []

streams_count = []
streams_size = []
# streams_storage_size = []

en_instances_count = []
en_meta_count = []
en_trail_count = []
en_sum_count = []

en_instances_size = []
en_meta_size = []
en_trail_size = []
en_sum_size = []

# en_instances_storage_size = []
# en_meta_storage_size = []
# en_trail_storage_size = []
# en_sum_storage_size = []

for sample in data:
    counter.append(sample["iteration"])
    time.append(sample["time"])

    db_data = sample["db"]
    db_data_size.append(db_data["data_size"])
    # db_storage_size.append(db_data["storage_size"])
    # db_file_size.append(db_data["file_size"])

    ep_data = sample["episodes"]
    ep_count.append(ep_data["count"])
    ep_size.append(ep_data["size"])
    # ep_storage_size.append(ep_data["storage_size"])

    streams_data = sample["streams"]["total"]
    streams_count.append(streams_data["count"])
    streams_size.append(streams_data["size"])
    # streams_storage_size.append(streams_data["storage_size"])


    en_data = sample["entities"]["total"]

    en_instances_data = en_data["instances"]
    en_instances_count.append(en_instances_data["count"])
    en_instances_size.append(en_instances_data["size"])
    # en_instances_storage_size.append(en_instances_data["storage_size"])
    
    en_trail_data = en_data["trail"]
    en_trail_count.append(en_trail_data["count"])
    en_trail_size.append(en_trail_data["size"])
    # en_trail_storage_size.append(en_trail_data["storage_size"])
    
    en_meta_data = en_data["meta"]
    en_meta_count.append(en_meta_data["count"])
    en_meta_size.append(en_meta_data["size"])
    # en_meta_storage_size.append(en_meta_data["storage_size"])

    en_sum_data = en_data["sum"]
    en_sum_count.append(en_sum_data["count"])
    en_sum_size.append(en_sum_data["size"])
    # en_sum_storage_size.append(en_sum_data["storage_size"])


def plot__db_size_vs_time():
	# db size  vs  tiempo
	fig, ax = plt.subplots()
	
	np_db_data_size = np.array(db_data_size)/1024.0
	# np_db_file_size = np.array(db_file_size)/1024.0
	# np_db_storage_size = np.array(db_storage_size)/1024.0

	ax.plot(np_db_data_size, 'k--', label='data size')
	# ax.plot(np_db_file_size, 'k:', label='file size')
	# ax.plot(np_db_storage_size, 'k', label='storage size')
	ax.legend(loc='lower right', shadow=True)
	ax.grid()
	# plt.axis([0, len(ep_count)-1, 0, max(en_meta_count)])
	plt.xlabel('TODO')
	plt.ylabel('espacio ocupado [MB]')
	plt.title('Uso de disco en documentos de MongoDB')
	plt.savefig("graph/mongo_disk.eps", format="eps", dpi=1000)
	# plt.show()

def plot__count_vs_time():
	# cantidades  vs  tiempo
	fig, ax = plt.subplots()
	ax.plot(ep_count, 'k--', label='episodios')
	ax.plot(streams_count, 'k:', label='streams')
	ax.plot(en_meta_count, 'k', label='registros')
	ax.legend(loc='upper left', shadow=True)
	ax.grid()
	plt.axis([0, len(ep_count)-1, 0, max(en_meta_count)])
	plt.xlabel('TODO')
	plt.ylabel('cantidad')
	plt.title('Cantidad de documentos en MongoDB')
	plt.savefig("graph/mongo_docs.eps", format="eps", dpi=1000)
	# plt.show()

plot__db_size_vs_time()
plot__count_vs_time()
