// Usage: $ mongo mongo_profiler.js
//
// All json units are expressed in integer kB.
//

// The following disk usage data is collected through mongodb stat calls.
//
// Disk Usage data:
// ok - total disk usage (db size)
//
// Episodes
// 	ok - number of documents
// 	ok - average episode size
// 	ok - episode collection size
// 	!! - max episode size
// 
// For each stream:
// 	ok - number of documents
// 	ok - collection size
// 	ok - average stream size
// 	!! - max stream size
//
// For each entity
// 	ok - number of registries
// 	ok - number of instances
//     ok - registry collection size
//     ok - instance collection size
//     ok - total collection size
//     ok - avg registry document size
//     ok - avg instance document size
//     !! - max registry document size
//     !! - max instance document size

function kB(bytes) {
	return Math.round((100*bytes/1024.0))/100
}

function MB(bytes) {
	return Math.round((100*bytes/(1024.0 * 1024.0)))/100
}

function fmt(float) {
	return (Math.round(float*100)/100).toString();
}

function get_timer() {
	return new Date();
}

function await_period(timer, period) {
	// sleep remaining time
	end_time = new Date();
	var elapsed_time = (end_time - timer);
	var sleep_time = Math.max(0.0, period - elapsed_time);
	sleep(sleep_time);
}

function parse_collection(db, name, total) {
	var collection = db.getCollection(name);
	var stats = collection.stats();
	// printjson(stats);

	var _col = {};
	_col.collection =  name;
	_col.count = stats.count;
	_col.avg_size = kB(stats.avgObjSize);
	_col.size = kB(stats.size);
	_col.storage_size = kB(stats.storageSize);

	if (Object.keys(total).length == 0) {
		total.count = 0;
		total.size = 0;
		total.storage_size = 0;
	}
	total.count += _col.count;
	total.size += _col.size;
	total.storage_size += _col.storage_size;

	return _col;
}

function get_db_stats(json) {
	var stats = db.runCommand({ dbStats: 1, scale: 1 });
	// printjson(stats);
	json.data_size = kB(stats.dataSize);
	json.storage_size = kB(stats.storageSize);
	json.file_size = kB(stats.fileSize);
}

function get_streams_stats(db, streams, json) {
	json.collection_names = streams;
	json.collections = [];

	var total = {};
	streams.forEach(function(stream) {
		var collection_name = "stream:" + stream;
		json.collections.push(parse_collection(db, collection_name, total));
	});
	json.total = total;
}

function entity_total_sum(total) {
	var sum = {};
	sum.count = total.instances.count + total.trail.count + total.meta.count;
	sum.size = total.instances.size + total.trail.size + total.meta.size;
	sum.storage_size = total.instances.storage_size + total.trail.storage_size + total.meta.storage_size;	
	return sum;
}

function get_entity_stats(db, entity, json, total) {
	var collection_name = "entity:" + entity;
	var collection_name_trail = collection_name + ".trail";
	var collection_name_meta = collection_name + ".meta";

	var info = {};
	info.name = entity;
	info.instances = parse_collection(db, collection_name, total.instances);
	info.trail = parse_collection(db, collection_name_trail, total.trail);
	info.meta = parse_collection(db, collection_name_meta, total.meta);
	info.sum = entity_total_sum(info);
	json.push(info);
}

function get_entities_stats(db, entities, json) {
	json.collection_names = entities;
	json.collections = [];	

	var total = {};
	total.instances = {};
	total.trail = {};
	total.meta = {};
	entities.forEach(function(entity) {
		get_entity_stats(db, entity, json.collections, total);
	});
	total.sum = entity_total_sum(total);
	json.total = total;
}

// params
var sample_frequency = 0.5; // [Hz]
var times = 10000;

// connection
var conn = new Mongo();
var db = conn.getDB("ltm_db");

// retrieve collections
var streams = [];
var entities = [];
var collections = db.getCollectionNames();
collections.forEach(function(name) {
	if (name.startsWith("entity:")) {
		if (name.endsWith(".meta") || name.endsWith(".trail")) return;
		entities.push(name.replace("entity:", ""));
	}

	if (name.startsWith("stream:")) {
		streams.push(name.replace("stream:", ""));
	}
});
// print(" - entities: " + entities);
// print(" - streams: " + streams);

function date_to_str(d) {
	var year = d.getFullYear().toString().substr(2);
	var month = ("0" + d.getMonth().toString()).slice(-2);
	var day = ("0" + d.getDate().toString()).slice(-2);
	var s = "-";
	var hour = ("0" + d.getHours().toString()).slice(-2);
	var minute = ("0" + d.getMinutes().toString()).slice(-2);
	var second = ("0" + d.getSeconds().toString()).slice(-2);
	return year + s + month + s + day + "_" + hour + s + minute + s + second; 
}

var result = {};
var cnt = 0;
var start_time, end_time;
var sample_period = 1000.0/sample_frequency; // in [ms]
print("[");
while (cnt < times) {
	cnt += 1;
	var timer = get_timer();

	var json = {};
	json.iteration = cnt;
	json.units = "kB";
	json.time = date_to_str(timer);

	// DB
	json.db = {};
	get_db_stats(json.db);

	// episodes
	json.episodes = parse_collection(db, "episodes", {});

	// streams
	json.streams = {};
	// get_streams_stats(db, streams, json.streams);

	// entities
	json.entities = {};
	// get_entities_stats(db, entities, json.entities);

	printjson(json);
	if (cnt < times) print(",");
	await_period(timer, sample_period);
}
print("]");