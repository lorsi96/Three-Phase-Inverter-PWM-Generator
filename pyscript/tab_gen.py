import numpy as np

N = 2100
MA = 1

t = np.linspace(0, 2*np.pi, int(N))
sine = np.round(100*MA/2.0*(1+np.sin(t)))
duty_table = [repr(int(a)) for a in sine]


c_code = '''
# ifndef _SIN_TABLE_
# define _SIN_TABLE_

const int sine_len = %d;
const int sinewave[] = {%s};

# endif
'''
with open("sine_table.h", "w") as file:
    file.write(c_code % (N, ','.join(duty_table)))
