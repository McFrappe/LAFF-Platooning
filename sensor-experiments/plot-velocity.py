import numpy as np
import matplotlib.pyplot as plt


spark8tapes = np.genfromtxt(
        'spark8tapes.csv',
        delimiter=',',
        skip_header=1,
        names=['t', 'v'])
nonspark8tapes = np.genfromtxt(
        '8tapes.csv',
        delimiter=',',
        skip_header=1,
        names=['t', 'v'])
onetape = np.genfromtxt(
        '1tape.csv',
        delimiter=',',
        skip_header=1,
        names=['t', 'v'])
sixtape = np.genfromtxt(
        '6tape.csv',
        delimiter=',',
        skip_header=1,
        names=['t', 'v'])

fig, axs = plt.subplots(4)

for ax in axs:
    ax.set_ylim([5, 18])

axs[0].set_title('8 tapes evenly distributed, Spark')
axs[0].plot(spark8tapes['t'], spark8tapes['v'])
axs[0].plot(spark8tapes['t'], [15 for _ in range(0, len(spark8tapes['t']))], 'r--')

axs[1].set_title('8 tapes evenly distributed')
axs[1].plot(nonspark8tapes['t'], nonspark8tapes['v'])
axs[1].plot(nonspark8tapes['t'], [9 for _ in range(0, len(nonspark8tapes['t']))], 'r--')

axs[2].set_title('1 tape')
axs[2].plot(onetape['t'], onetape['v'])
axs[2].plot(onetape['t'], [7 for _ in range(0, len(onetape['t']))], 'r--')

axs[3].set_title('6 tapes')
axs[3].plot(sixtape['t'], sixtape['v'])
axs[3].plot(sixtape['t'], [10 for _ in range(0, len(sixtape['t']))], 'r--')

plt.xlabel('Time (s)')
plt.ylabel('Velocity (km/h)')
plt.suptitle('Velocity of a car, 8 tapes evenly distributed')
plt.show()
