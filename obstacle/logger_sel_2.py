import numpy as np
from matplotlib import pyplot as plt

class Logger():
    def __init__(self, initial, plot_item1, plot_item2):
        self.log = {}
        self.plot_item1 = plot_item1
        self.plot_item2 = plot_item2
        
        for item in initial.keys():
            for level in initial[item].keys():
                self.log['error_val', item, level] = []
                


    def log_data(self, error):
        for item in error.keys():
            for level in error[item].keys():
                self.log['error_val', item, level].append(error[item][level])
                # self.log['current', item, level].append(current[item][level])

    def initialize_plot(self,title, frequency=1, offset1 = 0, offset2 = 0):
        self.frequency = frequency
        self.plot_info = [
            {'axis': 0, 'batch': 'error_val', 'item': self.plot_item1, 'level': 'pos', 'dim': 0+offset1, 'color': 'blue' , 'style': '-', 'label': self.plot_item1 },
            
            {'axis': 1, 'batch': 'error_val', 'item': self.plot_item2, 'level': 'pos', 'dim': 0+offset2, 'color': 'blue' , 'style': '-', 'label': self.plot_item2 },
            
            
            
            
        ]

        plot_num = np.max([item['axis'] for item in self.plot_info]) + 1
        self.fig, self.ax = plt.subplots(plot_num, 1, figsize=(6, 8))
        self.fig.suptitle(title, fontsize=14)


        self.ax[0].set_ylabel('[m]')
        self.ax[1].set_ylabel('[m]')
        self.ax[1].set_xlabel('time step')

        self.lines = {}
        for item in self.plot_info:
            key = item['batch'], item['item'], item['level'], item['dim']
            self.lines[key], = self.ax[item['axis']].plot([], [], color=item['color'], linestyle=item['style'], label=item['label'])
        
        for ax in self.ax:
            ax.legend()
            ax.axhline(y=0.015, color='black', linestyle='--')  # Add constant line
            ax.axhline(y=0.02, color='black', linestyle='--')  # Add constant line
            #ax.grid(True)
        
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