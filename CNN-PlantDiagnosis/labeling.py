# Imports
import pickle
import numpy as np
import matplotlib.pyplot as plt

data_files = np.load("public_data.npz", allow_pickle=True)
data = data_files["data"]
shrek = data[137]
trolol = data[338]

# Loop through all the images and find the ones that are shrek or trolol
bannedIndexes = []
for i, image in enumerate(data):
    if np.array_equal(image, shrek) or np.array_equal(image, trolol):
        bannedIndexes.append(i)

print(bannedIndexes)
