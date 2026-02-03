import pandas as pd
import numpy as np

import pandas as pd
import numpy as np

def print_model_parameter_matrix(graph, mdl, parameter='distance'):
    """
    Prints a matrix of the parameters currently assigned to 
    the variables in the model 'mdl' before solving.
    """
    nodes = sorted(list(graph.nodes))
    n = len(nodes)
    
    # Initialize matrix with zeros
    matrix = np.zeros((n, n))
    
    for i_idx, i in enumerate(nodes):
        for j_idx, j in enumerate(nodes):
            if i == j:
                continue
            
            # Retrieve the variable object from the model
            # This assumes your naming convention is x_i_j
            var = mdl.get_var_by_name(f'x_{i}_{j}')
            
            # If the variable is fixed to 0 (pruned), we label it as 'P'
            if var is not None and var.lb == 0 and var.ub == 0:
                matrix[i_idx, j_idx] = -1 # Marker for pruned/impossible
            else:
                # Map the physical graph parameter
                if parameter == 'distance':
                    matrix[i_idx, j_idx] = graph.edges[(i, j)]['distance']
                elif parameter == 'time_window_width':
                    matrix[i_idx, j_idx] = graph.nodes[j]['time_window'][1] - graph.nodes[j]['time_window'][0]
                elif parameter == 'info_slope':
                    matrix[i_idx, j_idx] = graph.nodes[j].get('info_slope', 0)

    # Create DataFrame
    df = pd.DataFrame(matrix, index=nodes, columns=nodes)
    
    # Formatting: Replace -1 with 'PRUNED' and 0 with '-'
    formatted_df = df.applymap(lambda x: 'PRUNED' if x == -1 else ('-' if x == 0 else f"{x:.2f}"))
    
    print(f"\n--- Pre-Solve Model Matrix: {parameter.upper()} ---")
    print(formatted_df)
    return df