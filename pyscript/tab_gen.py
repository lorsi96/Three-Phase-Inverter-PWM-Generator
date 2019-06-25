import numpy as np
import matplotlib.pyplot as plt

LINE_FREQ = 50
MF = 9
MA = 1


def sine_2_duty(dc=1.):
    return(list(np.round(dc*1000)/10))


t = np.linspace(0, 1/LINE_FREQ, int(LINE_FREQ*MF))
sine = MA/2.0*(1+np.sin(2*np.pi*LINE_FREQ*t))
duty_table = [repr(a) for a in sine_2_duty(sine)]


c_code = '''
#ifndef _SIN_TABLE_
#define _SIN_TABLE_

const unsigned int sine_len = %d;
const double sinewave[] = {%s};

#endif
'''
with open("sine_table.h", "w") as file:
    file.write(c_code % (len(duty_table), ','.join(duty_table)))
