import matplotlib.pyplot as plt
import pandas as pd
names = ['Time (s)', 'Joint 1', 'Force']
log_csv = pd.read_csv('log.csv', header=None, names=names)

log_csv.plot(0, 1, grid=True)
#log_csv.plot(0, 2, grid=True)

plt.show()