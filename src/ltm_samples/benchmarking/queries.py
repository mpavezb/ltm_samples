#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import random
from ltm_samples.faker.episodes import EpisodeFaker

def field_equals_int():
    uid = str(random.randint(1, 1000))
    return '{"uid": ' + uid + '}'

def array_contains_field():
    # episode array field contains provided string
    tags = EpisodeFaker().gen_tags()
    a_tag = tags[0]
    return '{"children_tags": "' + a_tag + '"}'

def field_in_array():
    # episode field is in provided array
    location_a, location_b = EpisodeFaker().gen_rand_locations()
    return '{"where.location": { "$in": ["' + location_a + '", "' + location_b + '"] } }'

def logical_or():
    emotion = str(random.uniform(0, 1.0))
    tags = EpisodeFaker().gen_tags()
    a_tag = tags[0]
    json_1 = '{"relevance.emotional.value": { "$gt": ' + emotion + ' } }'
    json_2 = '{"children_tags": "' + a_tag + '"}'
    return '{"$or": [ ' + json_1 + ', ' + json_2 + ']}'
    
def nested_or_and():
    child_id = str(random.randint(1, 1000))
    start_id = random.randint(1, 1000)
    end_id = str(start_id + random.randint(1, 100))
    start_id = str(start_id)
    emotion = str(random.uniform(0, 1.0))

    json_1 = '"$or": [ {"children_ids": ' + child_id + '}, {"uid": { "$gte": ' + start_id + ', "$lte": ' + end_id + ' } } ]'
    json_2 = '"relevance.emotional.value": { "$gt": ' + emotion +' }'
    return '{' + json_1 + ', ' + json_2 + '}'