import os
import matplotlib.pyplot as plt

def new_figure():
    plt.figure()

def show_figure():
    plt.show()

def set_title(title: str):
    plt.gca().set_title(title)

def save_figure():
    for i, fig in enumerate(plt.get_fignums()):
        figure = plt.figure(fig)
        # Get the figure name (if available, else use default)
        figure_name = figure.get_label() or figure.get_suptitle() or plt.gca().get_title() or f"figure_{i}"
        figure_path = os.path.join("..", "..", f"{figure_name}.png")
        figure.savefig(figure_path)
        print(f"Saved {figure_name} as {figure_path}")