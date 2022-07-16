import os
import numpy as np
goals_file = 'goals.txt'
dir_path = os.path.dirname(os.path.realpath(goals_file))
text = open(dir_path + '/' + goals_file, 'r')

p_seq = []
for line in text:
    if line[0].isnumeric():
        p_seq.append(line)
        print(p_seq)