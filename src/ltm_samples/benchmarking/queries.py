#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import random
from ltm_samples.faker.episodes import EpisodeFaker

# COMPARISON
# $eq     Matches values that are equal to a specified value.
# $ne     Matches all values that are not equal to a specified value.
# $gt     Matches values that are greater than a specified value.
# $gte    Matches values that are greater than or equal to a specified value.
# $lt     Matches values that are less than a specified value.
# $lte    Matches values that are less than or equal to a specified value.
# $in     Matches any of the values specified in an array.
# $nin    Matches none of the values specified in an array.

# LOGICAL
# $and    Joins query clauses with a logical AND returns all documents that match the conditions of both clauses.
# $or     Joins query clauses with a logical OR returns all documents that match the conditions of either clause.
# $nor    Joins query clauses with a logical NOR returns all documents that fail to match both clauses.
# $not    Inverts the effect of a query expression and returns documents that do not match the query expression.

# EVALUATION
# $expr        Allows use of aggregation expressions within the query language.
# $jsonSchema  Validate documents against the given JSON Schema.
# $mod         Performs a modulo operation on the value of a field and selects documents with a specified result.
# $regex       Selects documents where values match a specified regular expression.
# $text        Performs text search.
# $where       Matches documents that satisfy a JavaScript expression.

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