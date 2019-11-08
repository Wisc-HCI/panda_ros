from ctypes import *
from time import sleep
from matplotlib.animation import FuncAnimation

panda_controller_ext = CDLL("build/libpanda_controller_ext.so")
panda_controller_ext.get_state_history.argtypes = [POINTER(c_long), POINTER(POINTER(c_double)), POINTER(POINTER(c_double))]
fig, cartesianAxes = plt.subplots(nrows = 1, ncols = 1)
cartesianLines = [cartesianAxes.plot([])[0] for i in range(3)]

def forever():
    i = 0
    while True:
        yield i 
        i += 1

def reset():
    for l in cartesianLines:
        l.set_data([],[])
    cartesianAxes.set_xlim(0,1)
    cartesianAxes.set_ylim(0,1)

def update(frame):
    if not panda_controller_ext.isRunning():
        reset()
        return
    
    

def watchRobot():
    ani = FuncAnimation(fig, update, forever(), init_func = reset)
    


    


if __name__ == '__main__':
    print('Starting monitor')


