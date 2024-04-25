# this file compares the stats of all the inverse kinematics method comparing 
# Average Iterations, Median Iterations, Non-convergence Count and Non-convergence Percentage
# change line 22 (the trials number) this will change the number of times each method is run
# the higher the trials number the more time it will take but the more accurate the results 
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
from numpy import median
from NR import NR
from GN import GN
from WM import WM
from CM import CM
from SM import SM
from trust_constr import trust_constr
import pandas as pd
from angle_guess import angle_guess
from pose_guess import pose_guess
pd.set_option('display.max_columns', None)  # Show all columns
pd.set_option('display.width', 1000)        # Avoid wrapping to the next line
pd.set_option('display.max_rows', None)     # Show all rows

def run_experiment(method, method_name, trials=250, **kwargs): ##  trials value can changed 
    iteration = []
    non_converge_count = 0
    
    for _ in range(trials):
        angles = angle_guess()
        target = pose_guess()
        converged, q_final, iterations = method(angles, target, 150, **kwargs)
        # print(f"Trial Result: {converged}, Iterations: {iterations}, Type of iterations: {type(iterations)}")  # Debug print
        if converged:
            if isinstance(iterations, int):
                iteration.append(iterations)
            # else:
            #     print(f"Warning: Expected int but got {type(iterations)}")  # Handle unexpected data types
        else:
            non_converge_count += 1
    
    avg_iterations = np.mean(iteration) # if iteration else 0
    median_iterations = median(iteration) # if iteration else 0
    non_converge_percentage = (non_converge_count / trials) * 100
    
    # Plotting
    plt.figure(figsize=(10, 6))
    plt.hist(iteration, bins=range(1, 201), alpha=0.7, label=f'{method_name}')
    plt.title(f'Convergence Iterations Distribution for {method_name}')
    plt.xlabel('Iterations to Converge')
    plt.ylabel('Frequency')
    plt.legend()
    plt.savefig(f'{method_name}_convergence_distribution.png')
    
    return {
        'Method': method_name,
        'Average Iterations': avg_iterations,
        'Median Iterations': median_iterations,
        'Non-convergence Count': non_converge_count,
        'Non-convergence Percentage': non_converge_percentage
    }

# Assuming you've defined or imported your methods correctly, here's how you could structure your main experiment loop:
results = []
methods = [
    {'method': NR, 'name': 'NR', 'kwargs': {}},
    {'method': GN, 'name': 'GN', 'kwargs': {}},
    {'method': WM, 'name': 'WM_0.0001', 'kwargs': {'lm': 0.0001}}, ## value can changed 
    {'method': WM, 'name': 'WM_1e-6', 'kwargs': {'lm': 1e-6}}, ## value can changed 
    {'method': CM, 'name': 'CM_1', 'kwargs': {'lm': 1}}, ## value can changed 
    {'method': CM, 'name': 'CM_0.1', 'kwargs': {'lm': 0.1}}, ## value can changed 
    {'method': SM, 'name': 'SM_0.028773368', 'kwargs': {'lm': 0.028773368}},
    {'method': SM, 'name': 'SM_0.000287734', 'kwargs': {'lm': 0.000287734}},
    {'method': trust_constr, 'name': 'trust_constr', 'kwargs': {}},
]

for method_info in methods:
    result = run_experiment(method_info['method'], method_info['name'], **method_info['kwargs'])
    results.append(result)

# Assuming results is a list of dictionaries as returned by run_experiment, you could then use pandas or another library to format and print this as a table

results_df = pd.DataFrame(results)
print(results_df)

