import numpy as np
import matplotlib.pyplot as plt

lims = {
    "A": [-1, 11, -2, 2],
    "1": [-1, 14, -1, 14],
    "2": [-7, 36, -7, 7],
}

max_value = {
    "A": 2000,
    "1": 100,
    "2": 3000
}

stride = {
    "A": 5,
    "1": 5,
    "2": 2
}

for ptfun in ["orig", "wave"]: # "wave"
    for ex in ["A", ]: # "1", "2"
        x_min, x_max, y_min, y_max = lims[ex]

        matrix = np.loadtxt(f"pot{ex}_{ptfun}.txt").T

        nan_matrix = np.copy(matrix)
        inv_nan_matrix = np.copy(matrix)

        mx = max_value[ex] if ptfun == "orig" else 0.99999
        inv_nan_matrix[matrix< mx] = np.nan
        nan_matrix[matrix> mx] = np.nan
        matrix[matrix> mx] = mx

        fig = plt.figure()
        plt.imshow(nan_matrix, extent=lims[ex])
        plt.gca().invert_yaxis()  # Invert Y axis to match the array orientation
        plt.axis("equal")
        plt.gca().set_box_aspect((y_max-y_min)/(x_max-x_min)) 
        fig.savefig(f"pot_{ex}_{ptfun}.png")
        
        # Compute the gradient of the matrix
        # Returns two arrays: gradient in x direction (Gx) and y direction (Gy)

        matrix = matrix[::stride[ex], ::stride[ex]]


        Gy, Gx = np.gradient(matrix)
        norm = np.max(np.sqrt(Gx**2 + Gy**2))
        Gx /= norm
        Gy /= norm

        # Create the meshgrid for plotting (based on matrix shape)
        x = np.linspace(x_min, x_max, matrix.shape[1])
        y = np.linspace(y_min, y_max, matrix.shape[0])
        X, Y = np.meshgrid(x, y)

        # Plot the quiver plot for the gradient
        figs = 12
        fig = plt.figure(figsize=(figs,figs *1.2* (y_max-y_min)/(x_max-x_min))) #  
        plt.quiver(X, Y, -Gx, -Gy)
        # plt.imshow(norm == 0, extent=lims[ex])

        plt.axis("equal")
        plt.gca().set_box_aspect((y_max-y_min)/(x_max-x_min)) 

        # Add labels and title for clarity
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.tight_layout()

        fig.savefig(f"quiver_{ex}_{ptfun}.png")
# plt.show()
