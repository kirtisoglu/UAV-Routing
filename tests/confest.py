import random
from typing import Optional
from unittest.mock import patch

import networkx as nx
import pytest

random.seed(2025)


@pytest.fixture
def graph_with_ten_nodes():
    
    graph = nx.complete_graph(n=10)
    
    
    
    return graph



- coordinates
- an information collection function