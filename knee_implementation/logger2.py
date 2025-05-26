import numpy as np
from matplotlib import pyplot as plt

class Logger2():
    def __init__(self):
        self.log = {
            ('current', 'l'): [],
            ('current', 'r'): []
        }

    def log_data(self, ankle_l, ankle_r):
        self.log[('current', 'l')].append(ankle_l)
        self.log[('current', 'r')].append(ankle_r)

    def initialize_plot(self, frequency=1):
        self.frequency = frequency
        self.plot_info = [
            {'axis': 0, 'batch': 'current', 'item': 'l', 'color': 'red', 'style': '-','label':'right'},
            {'axis': 1, 'batch': 'current', 'item': 'r', 'color': 'red', 'style': '-','label':'left'},
        ]

        plot_num = max(item['axis'] for item in self.plot_info) + 1
        self.fig, self.ax = plt.subplots(plot_num, 1, figsize=(6, 8))

        self.lines = {}
        for item in self.plot_info:
            key = (item['batch'], item['item'])
            self.lines[key], = self.ax[item['axis']].plot(
                [], [], 
                color=item['color'], 
                linestyle=item['style'],
                label=item['label']
            )

        for ax in self.ax:
            ax.legend()
            
        for i in range(plot_num):
            self.ax[i].set_xlabel("Time step")
            self.ax[i].set_ylabel("[rad]")
            self.ax[i].legend(loc='upper right')
            
        self.fig.suptitle("Knee joint value", fontsize=14)



        plt.ion()
        plt.show()

    def update_plot(self, time):
        if time % self.frequency != 0:
            return

        for item in self.plot_info:
            key = (item['batch'], item['item'])
            data = np.array(self.log[key])
            self.lines[key].set_data(np.arange(len(data)), data)

        for ax in self.ax:
            ax.relim()
            ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
