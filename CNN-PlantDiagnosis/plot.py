# Imports
import pickle
import numpy as np
import matplotlib.pyplot as plt

data_files = np.load("public_data.npz", allow_pickle=True)
data = data_files["data"]

# Loop through all images and plot them to manually inspect them
bannedIndexes = []
for i, image in enumerate(data):
    if i < 335:
        continue
    plt.close("all")

    plt.imshow(image / 255, cmap="gray")
    plt.title(f"Image {i}")
    plt.show(block=False)
    input("Press enter to continue...")
