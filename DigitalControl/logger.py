
filename = '.log.csv'
subplots = True
names = ['Time (s)', 'Joint 1', 'Joint 2', "Vel 1", "Vel 2", "Acc 1", "Acc 2"]


class _Logger:
    def __init__(self):
        self.queue = []
        self.last_data = None
        self.skipped = False
        self.file = open(filename, 'w')

    def log(self, time, data, dt):
        if self.last_data == data:
            self.skipped = True
            if len(self.queue) > 0:
                self.dump()
            return

        # Needed for straight lines on plot
        if self.skipped:
            self.queue.append([time-dt, *self.last_data])

        self.last_data = data
        self.queue.append([time, *data])

    def dump(self):
        strings = (
            # [1,2,3] -> '1,2,3\n'
            ','.join((f'{el:.3f}' for el in line)) + '\n'
            for line in self.queue
        )
        self.file.writelines(strings)
        self.queue.clear()

    def close(self):
        self.dump()
        self.file.close()


Logger = _Logger() if __name__ != '__main__' else None


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import pandas as pd
    log_csv = pd.read_csv(filename, header=None, names=names)
    log_csv.plot(0, [1, 2], subplots=subplots,grid=True)
    plt.figure(1)
    log_csv.plot(0, [3, 4], subplots=subplots,grid=True)
    plt.figure(2)
    log_csv.plot(0, [5, 6], subplots=subplots,grid=True)
    plt.show()

