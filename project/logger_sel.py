import numpy as np
from matplotlib import pyplot as plt

class Logger():
    def __init__(self, initial, plot_item):
        self.log = {}
        self.plot_item = plot_item
        for item in initial.keys():
            for level in initial[item].keys():
                self.log['error_val', item, level] = []
                


    def log_data(self, error):
        for item in error.keys():
            for level in error[item].keys():
                self.log['error_val', item, level].append(error[item][level])
                # self.log['current', item, level].append(current[item][level])

    def initialize_plot(self, frequency=1, offset = 0):
        self.frequency = frequency
        self.plot_info = [
            {'axis': 0, 'batch': 'error_val', 'item': self.plot_item, 'level': 'pos', 'dim': 0+offset, 'color': 'blue' , 'style': '-' },
            
            {'axis': 1, 'batch': 'error_val', 'item': self.plot_item, 'level': 'pos', 'dim': 1+offset, 'color': 'blue' , 'style': '-' },
            
            
            {'axis': 2, 'batch': 'error_val', 'item': self.plot_item, 'level': 'pos', 'dim': 2+offset, 'color': 'blue' , 'style': '-' },
            
        ]

        plot_num = np.max([item['axis'] for item in self.plot_info]) + 1
        self.fig, self.ax = plt.subplots(plot_num, 1, figsize=(6, 8))

        self.lines = {}
        for item in self.plot_info:
            key = item['batch'], item['item'], item['level'], item['dim']
            self.lines[key], = self.ax[item['axis']].plot([], [], color=item['color'], linestyle=item['style'])
        
        plt.ion()
        plt.show()

    def update_plot(self, time):
        if time % self.frequency != 0:
            return

        for item in self.plot_info:
            trajectory_key = item['batch'], item['item'], item['level']
            trajectory = np.array(self.log[trajectory_key]).T[item['dim']]
            line_key = item['batch'], item['item'], item['level'], item['dim']
            self.lines[line_key].set_data(np.arange(len(trajectory)), trajectory)

        # set limits
        for i in range(len(self.ax)):
            self.ax[i].relim()
            self.ax[i].autoscale_view()
            
        # redraw the plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()