# operations.py

"""
Order preserved list operations to perform ...

Functions

Last Updated: 

"""

import random
from typing import Callable

from .state import State




def single_operation(state: State,
             method: Callable):
    
    method(state)
        
    return state.flip(state.flows)


def _pop_unused_random(unused_nodes, pos_in_unused):
    "Returns a random unused node in O(1)."
    i = random.randrange(len(unused_nodes))
    unused_nodes[i], unused_nodes[-1] = unused_nodes[-1], unused_nodes[i]
    x = unused_nodes.pop()
    pos_in_unused[unused_nodes[i]] = i  if i < len(unused_nodes) else None 
    pos_in_unused.pop(x, None)
    return x


def _unused_push(x, unused_nodes,  pos_in_unused):
    "Push a node back to the unused pool in O(1)."
    pos_in_unused[x] = len(unused_nodes)
    unused_nodes.append(x)


def replace_random_node(tour, unused_nodes, pos_in_unused):
    "Replace a random tour node with a random unused node"
    i = random.randrange(len(tour))
    new_x = _pop_unused_random(unused_nodes, pos_in_unused)      # O(1)
    old_x = tour[i]
    tour[i] = new_x
    _unused_push(old_x, pos_in_unused) 


def remove_random_node(tour, pos_in_unused):
    "Remove a random node in O(k) in worst case"
    i = random.randrange(len(tour))
    old_x = tour.pop(i)                # O(k-i), preserves order
    _unused_push(old_x, pos_in_unused)
    return tour


def add_random_node(tour, unused_nodes, pos_in_unused):
    "Add a random unused node to a random place in the tour"
    new_node = _pop_unused_random(unused_nodes, pos_in_unused)
    place = random.randrange(len(tour)+1)
    tour.insert(place, new_node)
    return tour




def swap_two_nodes(tour):
    "Switch the places of two random numbers in the tour → O(1)"
    i, j = random.sample(range(len(tour)), 2)
    tour[i], tour[j] = tour[j], tour[i]
    return tour



def swap_two_opt(tour: list) -> list[int]:
    "Performs a single, random 2-opt swap on a tour by reversing a segment."
    while True:
        i, j = sorted(random.sample(range(len(tour)), 2))
        if j - i >= 2:
            break
    tour[i:j+1] = tour[i:j+1][::-1]
    return tour 

