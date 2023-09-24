import matplotlib.pyplot as plt
import numpy as np

import matplotlib.animation as animation

# class DataVisualizer():
#     def __init__(self):
#         # Fixing random state for reproducibility
#         np.random.seed(19680801)
#         # Fixing bin edges
#         self.HIST_BINS = np.linspace(0, 4, 100)

#         # histogram our data with numpy
#         self.data = np.random.randn(1000)
#         n, _ = np.histogram(self.data, self.HIST_BINS)

#     def prepare_animation(self, bar_container):

#         def animate(self, frame_number):
#             # simulate new data coming in
#             self.data = np.random.randn(1000)
#             n, _ = np.histogram(self.data, self.HIST_BINS)
#             for count, rect in zip(n, bar_container.patches):
#                 rect.set_height(count)
#             return bar_container.patches
#         return animate


# # Output generated via `matplotlib.animation.Animation.to_jshtml`.
# histogram = DataVisualizer()
# fig, ax = plt.subplots()
# _, _, bar_container = ax.hist(histogram.data, histogram.HIST_BINS, lw=1,
#                               ec="yellow", fc="green", alpha=0.5)
# ax.set_ylim(top=55)  # set safe limit to ensure that all data is visible.

# ani = animation.FuncAnimation(fig, histogram.prepare_animation(bar_container), 50,
#                               repeat=False, blit=True)
# plt.show()


# Fixing random state for reproducibility
np.random.seed(19680801)
# Fixing bin edges
HIST_BINS = np.linspace(-4, 4, 100)

# histogram our data with numpy
data = np.random.randn(1000)
n, _ = np.histogram(data, HIST_BINS)


def prepare_animation(bar_container):

    def animate(frame_number):
        # simulate new data coming in
        data = np.random.randn(1000)
        n, _ = np.histogram(data, HIST_BINS)
        for count, rect in zip(n, bar_container.patches):
            rect.set_height(count)
        return bar_container.patches
    return animate


# Output generated via `matplotlib.animation.Animation.to_jshtml`.

fig, ax = plt.subplots()
_, _, bar_container = ax.hist(data, HIST_BINS, lw=1,
                              ec="yellow", fc="green", alpha=0.5)
ax.set_ylim(top=55)  # set safe limit to ensure that all data is visible.

ani = animation.FuncAnimation(fig, prepare_animation(bar_container), 50,
                              repeat=False, blit=True)
plt.show()
