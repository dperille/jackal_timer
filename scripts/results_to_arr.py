import os
import numpy as np
import math

results = np.zeros(300)
for fcount, fname in enumerate(file_list):
    with open(fname) as f:
        trial_num = 0
        while trial_num < 299:
            trial_num = int(f.readline())
            trial_val = float(f.readline())

            if trial_val >= 50.0:
                trial_val = 50.0
            results[trial_num] = trial_val
            
np.save('time_results.npy', results)