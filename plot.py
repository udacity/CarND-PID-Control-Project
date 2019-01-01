import ipdb
import pandas as pd
import matplotlib.pyplot as plt

fig, ax = plt.subplots(2, 1)

low_throttle = pd.read_csv('output/low_throttle.csv', sep=',', header=None).T[0]
low_throttle.plot(color='r', ax=ax[0])
ax[0].set_xlabel("CTE")
ax[0].set_title("Low throttle cross track error")
ax[0].grid(True)

high_throttle = pd.read_csv('output/high_throttle.csv', sep=',', header=None).T[0]
high_throttle.plot(color='b', ax=ax[1])
ax[1].set_xlabel("CTE")
ax[1].set_title("Low throttle cross track error")
ax[1].grid(True)

fig.tight_layout()

plt.savefig('output/CTE.png', figsize=(12, 10), dpi=80, facecolor='w', edgecolor='k')
plt.show()
