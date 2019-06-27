import numpy as np
import matplotlib.pyplot as plt

N = 2100
MA = 1

t = np.linspace(0, 2*np.pi, int(N))
sine = np.round(1000*MA*(1+np.sin(t))/2.0)/10
duty_table = [repr(float(a)) for a in sine]
plt.plot([float(a) for a in sine])
plt.show()
c_code = '''
# ifndef _SIN_TABLE_
# define _SIN_TABLE_

const int sine_len = %d;
const int sinewave[] = {%s};

# endif
'''
with open("sine_table.h", "w") as file:
    file.write(c_code % (N, ','.join(duty_table)))
