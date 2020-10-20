import matplotlib.pyplot as plt

def get_data(filename):
    x = []
    y = []
    with open(filename) as f:
        for line in f.read().splitlines():
            if line:
                y.append(float(line.split(' ')[1]))
                x.append(float(line.split(' ')[0]))
    return x, y


plt.title("Resistance profile for different external pressure")
plt.xlabel("Resistance on tip [Pa]")
plt.ylabel("Probe Y position [m]")
for filename in ('60kPa', '80kPa', '100kPa'):
    x, y = get_data(filename)
    plt.plot(y, x, label=filename)
plt.legend()
plt.show()
