import numpy as np
import matplotlib.pyplot as plt


fpath = r"/home/car-user/lab1/test.txt"


# define empty list
places = []
index = []
error = []
# open file and read the content in a list
with open(fpath, 'r') as filehandle:  
    filecontents = filehandle.readlines()

    for line in filecontents:
        # remove linebreak which is the last character of the string
        current_place = line[:-1]

        # add item to the list
        idx, err = current_place.split(',')
        index.append(idx)
        error.append(err)




print index
print error
