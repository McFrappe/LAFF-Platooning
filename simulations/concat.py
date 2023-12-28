import numpy as np

# Create two 2D arrays
array1 = np.array([[1, 2], [3, 4]])
array2 = np.array([[5, 6], [7, 8]])

# Vertical concatenation (along rows)
result_vertical = np.concatenate((array1, array2), axis=0)

print("\nVertical Concatenation:")
print(result_vertical)
