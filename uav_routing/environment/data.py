"""
data.py
=======
Parses Solomon-format text files into dictionaries for graph construction.
Provides save/load utilities for serialized Python objects.
"""

import pickle
import pandas as pd



class ValidationError(Exception):
    "Raised when data is not validated."


def save(object, path):
    with open(path, "wb") as f:
        pickle.dump(object, f)


def load(path):
    with open(path, "rb") as f:
        G = pickle.load(f)
    return G


def to_dataframe(path):
    " Convert a dictionary to a pandas DataFrame. "
    data =  data_to_dict(path)
    df = pd.DataFrame.from_dict(data, orient='index')
    return df
    

def data_to_dict(path):
    """
    Step 1: Parse text to dictionary.
    """
    nodes = {}
    
    with open(path, 'r') as file:
        file.readline() # Skip Line 1
        file.readline() # Skip Line 2
        
        third = file.readline()
        fields = third.strip().split()
        
        base = fields[0]
        nodes[int(base)] = {
                "position": (float(fields[1]), float(fields[2])),
                "info_at_lowest": 0,
                "time_window": [0, float(fields[8])],
                "info_slope": 0 
            }
        
        for line in file:
            fields = line.strip().split()
            
            if not fields:
                continue  # skip empty lines (e.g. trailing newline)
            
            try:
                valid = _validator(fields)
            except Exception as e:
                print(e)
                continue
            
            node_id = int(fields[0])
            nodes[node_id] = {
                "position": (float(fields[1]), float(fields[2])),
                "info_at_lowest": float(fields[4]),
                "time_window": (float(fields[8]), float(fields[9])),
                "info_slope": None, 
            }
    return nodes, base





def _validator(fields):
    """
    Validate a single data row from a Solomon file.
    
    Raises:
        ValidationError: If field count is not 9 or 10.
        ValueError: If time window is invalid (end <= start).
    """
    if len(fields) not in (9, 10):
        print("length", len(fields))
        print("row", fields)
        raise ValidationError(f"Field length must be 9 or 10, got {len(fields)} for {fields}.")
    
    if (float(fields[9]) - float(fields[8])) <= 0:
        raise ValueError(f"Time window of node {int(fields[0])} is not valid.")








