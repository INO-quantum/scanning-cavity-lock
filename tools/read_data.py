#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# read buffers for given PEAK from Arduino Due and save to file

import serial
import numpy as np
from numpy.linalg import inv
from scipy.signal import savgol_filter, savgol_coeffs
from scipy.optimize import newton
from struct import pack, unpack, calcsize

from os import listdir
from os.path import isfile, split, splitext, join

import matplotlib as mpl
import matplotlib.pyplot as plt
# for save button use current working directory
mpl.rcParams["savefig.directory"] = ""
#mpl.rcParams['font.family'] = 'sans-serif' # 'DejaVu Sans'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.linewidth'] = 2

PORT            = ['/dev/ttyACM0', 'COM5', None][2] # Arduino port (Linux or Windows) or None = load file
BUF_SIZE        = 128
NUM_BUFFER      = 8
NUM_PEAKS       = 6                 # 3 or 6
PEAK            = 1                 # peak 0,1,2.. to read from Arduino or file or NUM_PEAKS = all of them (TODO: not tested I think).
DERIV           = True              # if True get derivative from Arduino
DERIV_FUNC      = ['diff', 'SG5', 'SG9', 'Scipy'][2] # function to calculate derivative. Arduino uses SG5 or SG9.
FULL            = True              # get full (True) or half peak (False, TODO: gives errors)

MCK             = 84.0              # master clock in MHz
TICKS_RELOAD    = (2**24)-1         # reload value for ticks

HEADER          = '<2I4H4I2i4H%iI'%(NUM_BUFFER) # header (32bit aligned)
HEADER_ENTRIES  = 16+NUM_BUFFER     # consistency check of HEADER
FOOTER          = '<Hh'             # number of samples and error
BUFFER          = '<%iH'            # number of samples of uint16_t
DBUFFER         = '<%ii'            # number of samples of int32_t

MIN_SLOPE       = 20                # minimum slope in units of 3.3V/2^12/ADC conversion

NUM_FIT_POINTS  = 4                 # number of points for fitting zero crossing

conv_y = 3.3/((2**12)-1)            # conversion y-axis into Volts

THRESHOLD       = np.array([0.25, 0.5]) # low high threshold in Volts

ADC_TIME        = 1.5               # ADC conversion time in microseconds

# filter scaling coefficients in mV/us obtained with Scipy (see output for DERIV_FUNC)
filter_scaling = {  'diff' : 1                    *1000 / ADC_TIME,
                    'SG5'  : 1.000e-01            *1000 / ADC_TIME,
                    'SG9'  : 0.011904761904761897 *1000 / ADC_TIME, }

# file names for saving and loading
save_name = './data/raw_%i.csv'
load_name = './sample_data/raw_%i_14.csv'

# if true save file with new name
save_new_name = True

colors = ['Blue', 'Orange', 'Green', 'Violet', 'Brown', 'Magenta']

def plot(data, data_labels, data_args=None, 
         curves=[], curves_labels=[], curves_args=None, 
         title=None, xlabel=None, ylabel=None, 
         xrng=None, yrng=None,
         twin=None,
         size=(14,7), pos=[0.055, 0.08, 0.89, 0.86],
         legend_args=None,
         fmt=None):
    """
    plot data and curves with given labels.
    data         = list of [[x0,y0],[x1,y1],...] plotted as data points
    data_labels  = labels of each data
    data_args    = dictionary with plot arguments like color, style, etc. if None, default values are taken.
    curves       = list of [[x0,y0],[x1,y1],...] plotted as lines between the points
    curve_labels = labels of each curve
    curve_args   = dictionary with plot arguments like color, style, etc. if None, default values are taken.
    title        = if not None title of figure
    xlabel       = if not None x label of figure
    ylabel       = if not None y label of figure
    xrng         = if not None [xmin,xmax]
    yrng         = if not None [ymin,ymax]
    twin         = if not None, must be returned axis of plot and adds a twin axis with this data
    size         = figure size (h,v)
    pos          = relative axis position [left, bottom, width, height]
    legend_args  = if not None arguments for axis.legend
    fmt          = if not None format for coordinates. x,y for normal axis, and x,y0,y1 for twin axis.
    """
    if twin is None:
        fig = plt.figure(figsize=size) #size in inches (h,v)
        ax = fig.add_axes(pos)
        if title is not None: ax.set_title(title, fontsize=14)

        if xlabel is not None: ax.set_xlabel(xlabel, fontsize=14, labelpad=5)

        ax.xaxis.set_tick_params(which='major', size=10, width=2, direction='in', top  ='on', labelsize=12)
        ax.xaxis.set_tick_params(which='minor', size= 7, width=2, direction='in', top  ='on', labelsize=12)
    else:
        fig = twin.figure
        ax = twin.twinx()
        
    if xrng is not None:
        ax.set_xlim(xrng)
    if yrng is not None:
        ax.set_ylim(yrng)

    if ylabel is not None: ax.set_ylabel(ylabel, fontsize=14, labelpad=5)
    ax.yaxis.set_tick_params(which='major', size=10, width=2, direction='in', right='on', labelsize=12)
    ax.yaxis.set_tick_params(which='minor', size= 7, width=2, direction='in', right='on', labelsize=12)

    if False:
        scale = 0.05
        y_min = np.min([np.min(d[0]) for d in data])
        y_max = np.max([np.max(d[0]) for d in data])
        dy = (y_max-y_min)*scale
        ax.set_xlim(y_min-dy, y_max+dy)

        y_min = np.min([np.min(d[1]) for d in data])
        y_max = np.max([np.max(d[1]) for d in data])
        dy = (y_max-y_min)*scale
        ax.set_ylim(y_min-dy, y_max+dy)

    #plt.ticklabel_format(axis='y', style='sci', scilimits=(0,0))
    #plt.yscale('log')

    if data is not None:
        for i,d in enumerate(data):
            if d is not None:
                if data_args is None or data_args[i] is None:
                    args = {'color': colors_tab(((i*2)+int(i//colors_tab.N))%colors_tab.N)}
                else:
                    args = data_args[i]
                if len(d) == 3: # error bar
                    for dj in np.transpose(d):
                        ax.plot([dj[0],dj[0]], [dj[1]-dj[2],dj[1]+dj[2]], color=args['color'], linewidth=2)
                ax.scatter(d[0], d[1], label=data_labels[i], **args)

    if curves is not None:
        for i,d in enumerate(curves):
            if d is not None:
                if curves_args is None or curves_args[i] is None:
                    args = {'color': colors_tab(((i*2+1)+int(i//colors_tab.N))%colors_tab.N)}
                else:
                    args = curves_args[i]
                ax.plot(d[0], d[1], label=curves_labels[i], **args)

    largs = {'bbox_to_anchor':(0.98, 0.97), 'loc':'upper right', 'frameon':True, 'fontsize':10}
    if legend_args is not None: 
        largs.update(legend_args)
    if twin is None:
        ax.legend(**largs)
        if fmt is None:
            fmt = "x=%.3f y=%.3f"
        ax.format_coord = lambda x,y: fmt % (x,y)
    else:
        if fmt is None:
            fmt = "x=%.3f y0=%.3f y1=%.3f"
        def format_coord(x, y):
            dsp   = ax.transData.transform((x,y))
            x2,y2 = twin.transData.inverted().transform(dsp)
            return fmt % (x, y2, y)
            
        l0, lb0 = twin.get_legend_handles_labels()
        l1, lb1 = ax  .get_legend_handles_labels()
        twin.legend(l0+l1, lb0+lb1, **largs)
        ax.format_coord = format_coord
    
    #plt.savefig(title + '.png', dpi=300, transparent=False, bbox_inches='tight')
    
    return ax

#returns string in the form value(error)
def str_error(value, error):
    try:
        pwr = np.log10(error)
        if pwr < 0:
            pwr = -int(np.floor(pwr))
            fmt = '%%.%if(%%i)' % (pwr)
            txt = fmt % (round(value,pwr),int(round(error*(10**pwr))))
        else:
            pwr = int(np.floor(pwr))
            txt = '%i(%i)' % (int(round(value/(10**pwr))*(10**pwr)),int(round(error/(10**pwr)))*(10**pwr))
        return txt
    except Exception as e: # something went wrong
        print(e)
        return '%.3f ± %.3f' % (value, error)

def ticks_diff(t_start, t_end, check_negative=False):
    '''
    returns difference of ticks. ticks count down until 0 when they get reloaded to TICKS_RELOAD.
    if check_negative then t_end might have been BEFORE t_start and t_end > t_start looks like ticks reloaded.
    in this case if |wrapped time| > |t_start - t_end| returns t_start - t_end which is negative. 
    '''
    dt = t_start-t_end if (t_start > t_end) else (t_start + TICKS_RELOAD) - t_end
    if check_negative and t_start < t_end:
        dt_neg = t_start - t_end
        if -dt_neg < dt: return dt_neg
    return dt
    
def check_header(peak_to_read):
    check_error = False
    
    # get number of buffers to read
    n_read = 1;
    n = n_high
    while n != n_low:
        n = (n + 1) % NUM_BUFFER
        n_read += 1

    # conversion time for ADC samples in us
    conv_x = float(cal_ticks)/(MCK*(2**cal_div))
    delta = np.abs(conv_x - ADC_TIME)
    if delta > 1.0/MCK:
        print('unexpected conversion time %.3f us != %.3f us' % (conv_x, ADC_TIME))
        check_error = True

    if peak != peak_to_read:
        print('unexpected peak %i != %i' % (peak, peak_to_read))
        check_error = True

    if tot_buf != NUM_BUFFER:
        print('unexpected number of buffers %i != %i' % (tot_buf, NUM_BUFFER))
        check_error = True
        
    if len(t_buf) != NUM_BUFFER:
        print('unexpected number of buffer times %i != %i' % (len(t_buf), NUM_BUFFER))
        check_error = True

    if FULL:
        expect = (n_read-1)*BUF_SIZE + i_low - i_high + 2*num_edge
        if samples != expect:
            print('unexpected number of samples %i != %i (%i buffers to read)' % (samples, expect, n_read))
            check_error = True

    # ensure t_buf[n_high] <= t_high <= buffer duration 
    # if n_low != n_high we can check also that t_high <= t_buf[n_low] 
    n_next = (n_high + 1) % NUM_BUFFER
    if (ticks_diff(t_buf[n_high], t_high)/MCK > BUF_SIZE*conv_x):
        print('\nunexpected t_high = %.3f us after buffer %i start > buffer duration %.3f us\n' % (ticks_diff(t_buf[n_high], t_high)/MCK, n_high, BUF_SIZE*conv_x))
        check_error = True
    elif (n_low != n_high) and (ticks_diff(t_high, t_buf[n_next])/MCK > BUF_SIZE*conv_x):
        print('\nunexpected t_high = %.3f us before next buffer %i start > buffer duration %.3f us\n' % (ticks_diff(t_high, t_buf[n_next])/MCK, n_next, BUF_SIZE*conv_x))
        check_error = True
    # ensure t_buf[n_low] <= t_low <= buffer duration
    # we cannot check t_high <= t_buf[n_next] since n_next is most likely an old buffer written before n_low.
    if (ticks_diff(t_buf[n_low], t_low)/MCK > BUF_SIZE*conv_x):
        print('\nunexpected t_low = %.3f us after buffer %i start > buffer duration %.3f us\n' % (ticks_diff(t_buf[n_low], t_low)/MCK, n_low, BUF_SIZE*conv_x))
        check_error = True

    # check that i_high and t_high agree within 30 ticks = 0.36us
    delta = abs(ticks_diff(t_buf[n_high], t_high)/MCK - i_high*conv_x)
    if delta > 30/MCK:
        print('\nunexpected time difference between t_high %.3f us and i_high %.3f us from buffer %i start time %.3f us (%.3f us, %i ticks difference)\n' % (ticks_diff(t_buf[n_high], t_high)/MCK, i_high*conv_x, n_high, ticks_diff(t_trig, t_buf[n_high])/MCK, delta, delta*MCK))
        #check_error = True
    else:
        print('note: |t_high - t_low| = |%.3f us - %.3f us| = %.3f us (%i ticks, buffer %i start time %.3f us)' % (ticks_diff(t_buf[n_high], t_high)/MCK, i_high*conv_x, delta, delta*MCK, n_high, ticks_diff(t_trig, t_buf[n_high])/MCK))

    # check that i_low and t_low agree within 40 ticks = 0.48us
    # i_low has always a larger difference than i_high?
    delta = abs(ticks_diff(t_buf[n_low], t_low)/MCK - i_low*conv_x)
    if delta > 40/MCK:
        print('\nunexpected time difference between t_low %.3f us and i_low %.3f us from buffer %i start time %.3f us (%.3f us, %i ticks difference)\n' % (ticks_diff(t_buf[n_low], t_low)/MCK, i_low*conv_x, n_low, ticks_diff(t_trig, t_buf[n_low])/MCK, delta, delta*MCK))
        #check_error = True
    else:
        print('note: |t_low - i_low| = |%.3f us - %.3f us| = %.3f us (%i ticks, buffer %i start time %.3f us)' % (ticks_diff(t_buf[n_low], t_low)/MCK, i_low*conv_x, delta, delta*MCK, n_low, ticks_diff(t_trig, t_buf[n_low])/MCK))

    # check that i_peak agrees with t_peak within ADC_TIME
    # this is without correction on slope which reduces time since i_peak is index of first negative derivative
    t_peak_calc = ticks_diff(t_trig, t_high) + (i_peak - 2*num_edge)*cal_ticks/(2**cal_div)
    delta = t_peak_calc - t_peak
    if delta > ADC_TIME*MCK:
        print('\ni_peak time %.3f us does not agree with t_peak %.3f us (error %.3f us, %i ticks)\n' % (t_peak_calc/MCK, t_peak/MCK, delta/MCK, delta))
        #check_error = True
    else:
        print('note: t(i_peak) - t_peak = %.3f us (ok)\n' % (delta/MCK))        
    
    if (DERIV and get_deriv != 1):
        print('unexpected deriv bit %i != 1' % (get_deriv))
        check_error = True
    elif (not DERIV and get_deriv != 0):
        print('unexpected deriv bit %i != 0' % (get_deriv))
        check_error = True

    if (FULL and get_full != 1):
        print('unexpected full bit %i != 1' % (get_full))
        check_error = True
    elif (not FULL and get_full != 0):
        print('unexpected full bit %i != 0' % (get_full))
        check_error = True

    print('peak %i, buffer %i-%i (%i), %i samples, position %i, time %.3f us (%i ticks)' % (peak, n_high, n_low, n_read, samples, i_peak, t_peak/MCK, t_peak))
    print('time  [trig, high, low] = [%i, %i, %i] ticks = [0, %.3f, %.3f] us' % (t_trig, t_high, t_low, ticks_diff(t_trig, t_high)/MCK, ticks_diff(t_trig, t_low)/MCK))
    print('index [high, low] = [%i, %i]' % (i_high, i_low))
    print('time buffer = [' + (', '.join(['%i'%tb for tb in t_buf])) + '] ticks')
    print('            = [' + (', '.join(['%.3f'%(ticks_diff(t_trig, tb, True)/MCK) for tb in t_buf])) + '] us')
    print('ticks/ADC = %i/2^%i = %.3f = %.3f us/ADC' % (cal_ticks, cal_div, float(cal_ticks)/2**cal_div, conv_x))
    
    if check_error: 
        print('\nstop due to previous error(s)\n')
        exit()

    return n_read, conv_x
    
def polynomial(x, a):
    'returns polynomial sum_k a_k*x^k'
    return np.sum([ak*x**k for k,ak in enumerate(a)], axis=0)
    
def polynomial_deriv(x, a):
    'returns derivative of polynomial sum_k a_k*k*x^(k-1)'
    return np.sum([ak*(k+1)*x**k for k,ak in enumerate(a[1:])], axis=0)

def poly_fit(x, y, order):
    '''
    polynomial fit of y with given order around x.
    if x is None then order = list of linear function values f(x).
    for line:                 order = 1 or order = (1, x)
    for 2nd order polynomial: order = 2 or order = (1, x, x**2)  
    returns (polynomial coefficients, error matrix)
    '''
    if x is None:
        if isinstance(order, (list, tuple, np.ndarray)):
            f = order
        else:
            print('poly_fit error: order must be polynomial order or list of functions of x with x=None')
            exit()
    else:
        if isinstance(order, int):
            f = np.array([x**o for o in range(order+1)])
        else:
            print('poly_fit error: order must be polynomial order or list of functions of x with x=None')
            exit()        
    alpha = np.empty(shape=(len(f), len(f)), dtype=float)
    for r in range(len(f)): 
        for c in range(r, len(f)):
            alpha[r,c] = alpha[c,r] = np.sum(f[r]*f[c])
    beta = np.array([np.sum(y*fo) for fo in f])
    epsilon = inv(alpha)
    a = epsilon @ beta
    return (a, epsilon)

if __name__ == '__main__':

    peaks = list(range(NUM_PEAKS)) if PEAK == NUM_PEAKS else [PEAK]
    for peak_to_read in peaks:

        if PORT is None:
            print('reading Arduino data from file:', load_name%peak_to_read)

            # load data from file
            line_data = []
            with open(load_name%peak_to_read, 'r') as f:
                # read all lines and split into colums with ','
                while True:
                    line = f.readline()
                    if len(line) == 0: break
                    line = line.rstrip().split(',')
                    line_data.append(line)

            # read header until first data
            print('header:')
            header = {}
            for i,l in enumerate(line_data):
                try:
                    val = int(l[0])
                    index = i
                    break
                except ValueError:
                    key   = l[0].strip()
                    value = int(l[1]) if len(l) == 2 else [int(li) for li in l[1:]]
                    header[key] = value
                    print(key, value)

            # header
            print('header:', header)
            # flags
            peak      = header['peak']
            n_high    = header['n_high'] # in very old files: n_start
            n_low     = header['n_low']  # in very old files: n_end
            tot_buf   = header['tot_buf']
            num_edge  = header['num_edge']
            cal_div   = header['cal_div']
            get_deriv = header['deriv']
            get_full  = header['full']
            # other header data
            cal_ticks = header['cal_ticks']
            samples   = header['samples']
            i_high    = header['i_high']
            i_low     = header['i_low']
            i_peak    = header['i_peak']
            t_trig    = header['t_trig']
            t_high    = header['t_high']
            t_low     = header['t_low']
            t_peak    = header['t_peak']
            t_buf     = header['t_buf']
            # old files might have them 
            try:
                l_error   = header['l_error']
            except Exception:
                l_error = 0
            try:
                r_error   = header['r_error']
            except Exception:
                r_error = 0
            try:
                l_out     = header['l_out']
            except Exception:
                l_out = 0
            try:
                r_low     = header['r_low']
            except Exception:
                r_low = 0
            try:
                r_high    = header['r_high']
            except Exception:
                r_high = 0
            try:
                error     = header['error']
            except Exception:
                error = 0

            # check numbers
            n_read, conv_x = check_header(peak_to_read)

            if error != 0:
                print('\nWarning: error = %i indicates error in Arduino! data buffer might be overwritten.\n' % (error))

            # read data
            data = []
            for i,l in enumerate(line_data[index:index+samples]):
                ii, val = [int(li) for li in l]
                if ii != i:
                    print(load_name%peak_to_read, 'error data index %i != %i' % (ii, i))
                    exit()
                data.append(val)
            data = np.array(data)
            index += samples

            data_d = []
            # read derivative
            if DERIV:                
                for i,l in enumerate(line_data[index:index+samples]):
                    ii, val = [int(li) for li in l]
                    if ii != i:
                        print(load_name%peak_to_read, 'error derivative index %i != %i' % (ii, i))
                        exit()
                    data_d.append(val)
                data_d = np.array(data_d)

        else:
            print('connect Arduino Due on native port (farther from power plug). if does not send data: reset the Due and try again.\n')

            if peak_to_read == peaks[0]:
                # open USB port
                com = serial.Serial(PORT)

                # Arduino waits for 'p#\n' to send peak # data.
                #com.write(bytes(np.array([p], dtype=np.int8)))
                if DERIV:
                    cmd = b'D%i\n' if FULL else b'd%i\n'  
                else:
                    cmd = b'P%i\n' if FULL else b'p%i\n'  
                com.write(cmd%PEAK)

            # read header '<2I4H4I2i2H8I'
            data = com.read(calcsize(HEADER))
            data = unpack(HEADER, data)
            if len(data) != HEADER_ENTRIES:
                print('error reading header: %i entries != expected %i!' % (len(data), HEADER_ENTRIES))
                exit()
            flags     = data[0]
            cal_ticks = data[1]
            samples   = data[2]
            i_high    = data[3]
            i_low     = data[4]
            i_peak    = data[5]
            t_trig    = data[6]
            t_high    = data[7]
            t_low     = data[8]
            t_peak    = data[9]
            l_error   = data[10]
            r_error   = data[11]
            l_out     = data[12]
            r_low     = data[13]
            r_high    = data[14]
            dummy     = data[15]
            t_buf     = data[16:16+NUM_BUFFER]
            
            peak      = (flags >>  0) & 0x0f
            n_high    = (flags >>  4) & 0x0f
            n_low     = (flags >>  8) & 0x0f
            tot_buf   = (flags >> 12) & 0x0f
            num_edge  = (flags >> 16) & 0x0f
            cal_div   = (flags >> 20) & 0xff
            get_deriv = (flags >> 28) & 0x01
            get_full  = (flags >> 29) & 0x01
                    
            # check numbers
            n_read, conv_x = check_header(peak_to_read)

            # read data samples
            tmp = com.read(calcsize(BUFFER%samples))
            data = np.array(unpack(BUFFER%samples, tmp))
                
            if DERIV:
                # read derivative. must be the same length as data.
                # first 2*num_edge derivatives are invalid. 
                # data_d[2*num_edge] corresponds to data[num_edge]
                tmp = com.read(calcsize(DBUFFER%samples))
                data_d = np.array(unpack(DBUFFER%samples, tmp))
            else: data_d = None
                
            # read footer = # samples | error code
            if True:
                tmp = com.read(calcsize(FOOTER))
                num, error = unpack(FOOTER, tmp)
                error = 0
                if error != 0:
                    print('\nWarning: error = %i indicates error in Arduino! data buffer might be overwritten.\n' % (error))
                    #print(tmp)
                if num != samples:
                    print('\nfooter = %i != %i (number of samples)\n' % (num, samples))
                    print(tmp)
                    exit()
                elif not error:
                    print('%i samples transmitted without error\n' % (samples))
                
                if False:    
                    tmp = com.read(calcsize('<3I'))
                    test = unpack('<3I', tmp)
                    print('test data:', test)

            else:
                # check remaining bytes for debugging. requires timeout > 0.
                t_old = com.timeout
                print('old timeout', t_old)
                com.timeout = 1.0
                more = b''
                while True:
                    tmp = com.read(1)
                    if tmp == b'': break
                    more = more + tmp
                print('received %i additional bytes:' % len(more))
                print(more)
                com.timeout = t_old

            if True:
                # save data to file
                if save_new_name:
                    # get matching filenames in folder
                    # find index of next file with pattern added
                    pattern = '_%i'
                    start, end = pattern.split('%i')
                    path, name = split(save_name%peak_to_read)
                    name, ext = splitext(name)
                    zerolen = len(name) + len(start) + len(end) + len(ext)
                    files = [f for f in listdir(path) if isfile(join(path,f)) and len(f) > zerolen and f.startswith(name) and f.endswith(ext)]
                    if len(files) == 0:
                        index = 0
                    else:
                        index = 0
                        value = None
                        for f in files:
                            middle = f[len(name):len(f)-len(ext)]
                            try:
                                value = int(middle[len(start):len(f)-len(end)])
                            except Exception:
                                continue
                            if value > index: index = value
                        if value is not None:
                            index += 1
                    fname = join(path, name + pattern%(index) + ext)
                else:
                    fname = save_name%peak_to_read
                with open(fname, 'w') as f:
                    # write header
                    # flags
                    f.write('peak     , %i\n' % (peak     ))
                    f.write('n_high   , %i\n' % (n_high   ))
                    f.write('n_low    , %i\n' % (n_low    ))
                    f.write('tot_buf  , %i\n' % (tot_buf  ))
                    f.write('num_edge , %i\n' % (num_edge ))
                    f.write('cal_div  , %i\n' % (cal_div  ))
                    f.write('deriv    , %i\n' % (get_deriv))
                    f.write('full     , %i\n' % (get_full ))
                    # other header data
                    f.write('cal_ticks, %i\n' % (cal_ticks))
                    f.write('samples  , %i\n' % (samples  ))
                    f.write('i_high   , %i\n' % (i_high  ))
                    f.write('i_low    , %i\n' % (i_low    ))
                    f.write('i_peak   , %i\n' % (i_peak   ))
                    f.write('t_trig   , %i\n' % (t_trig   ))
                    f.write('t_high   , %i\n' % (t_high   ))
                    f.write('t_low    , %i\n' % (t_low    ))
                    f.write('t_peak   , %i\n' % (t_peak   ))
                    f.write('l_error  , %i\n' % (l_error  ))
                    f.write('r_error  , %i\n' % (r_error  ))
                    f.write('l_out    , %i\n' % (l_out    ))
                    f.write('r_low    , %i\n' % (r_low    ))
                    f.write('r_high   , %i\n' % (r_high   ))
                    f.write('t_buf    , %s\n' % (', '.join(['%i'%tb for tb in t_buf]))); 
                    f.write('error    , %i\n' % (error    ))
                    # write data lines
                    for j,d in enumerate(data):
                        f.write('%4i,%4i\n' % (j, d))
                    if DERIV:
                        # write derivative lines
                        for j,d in enumerate(data_d):
                            f.write('%4i,%4i\n' % (j, d))
                print(fname, 'written %i data' % (len(data)))

            # close USB port
            if peak_to_read == peaks[-1]: 
                com.close()
    
        data_list      = []
        data_labels    = []
        data_args      = []
        data_list_2    = []
        data_labels_2  = []
        data_args_2    = []
        curve_list     = []
        curve_labels   = []
        curve_args     = []
        curve_list_2   = []
        curve_labels_2 = []
        curve_args_2   = []

        # start time in us        
        dt_start = ticks_diff(t_trig, t_high)/MCK

        # peak data from Arduino
        x = (np.arange(samples)-num_edge)*conv_x + dt_start
        y = data*conv_y
        data_list   += [[x, y]]
        data_labels += ['data (Arduino)']
        data_args   += [{'color':colors[peak_to_read], 's':20}]
        
        # add thresholds
        x = [ticks_diff(t_trig, t_low )/MCK, ticks_diff(t_trig, t_high)/MCK]
        y = THRESHOLD
        data_list   += [[x, y]]
        data_labels += ['t_high/t_low']
        data_args   += [{'color':'Black', 'marker':'+', 's':2000, 'linewidth': 2}]

        # get buffer index and number of samples
        buf = [] # for each buffer 2 entries for start/end: [buffer index, sample index, time in us]
        num = 0
        n = n_high
        while True:
            t = ticks_diff(t_trig, t_buf[n])/MCK
            buf.append([n, num*conv_x, t])
            num += BUF_SIZE
            buf.append([n, num*conv_x, t + BUF_SIZE*conv_x])
            if n == n_low: break
            n = (n+1) % NUM_BUFFER
        buf = np.transpose(buf)
        num = num - BUF_SIZE + i_low - i_high + 2*num_edge
            
        if FULL and num != samples:
            print('unexpected number of samples %i != %i (2)' % (num, samples))
            exit()

        if n_read != int(len(buf[0])//2):
            print('unexpected number of buffers to read %i != %i' % (n_read, int(len(buf[0])//2)))
            print(buf)
            exit()    

        if False: # add buffer index to data
            curve_list   += [[buf[2], buf[0]/10]]
            curve_labels += ['buffer index/10'] 
            curve_args   += [{'color':'Brown', 'linewidth':1, 'linestyle':'dashed'}]

        if False:
            # polynomial fit through data peak
            num   = 5 # +/- number of points around peak to take
            order = 2
            x0 = np.argmax(y)
            xp = x[x0-num: x0+num+1]-x[x0]
            yp = y[x0-num: x0+num+1]
            if True: # polynomial of given order
                ap, ep = poly_fit(x=xp, y=yp, order=order)
                #yfit = polynomial(xp, ap)
                yfit = polynomial_deriv(xp, ap)*conv_x*10
            else:
                # polynomial with even coefficients only - this is always symmetric around x0 and cannot be used to get better peak position!
                ap, ep = poly_fit(x=None, y=yp, order=[xp**o for o in range(0, order+1, 2)])
                yfit = np.zeros(shape=(len(xp),), dtype=float)
                for o in range(0, order+1, 2):
                    yfit += ap[o//2]*xp**o
            curve_list   += [[xp+x[x0], yfit]]
            curve_labels += ['fit peak o=%i'%order]
            curve_args   += [{'color':colors[peak_to_read], 'linewidth':1, 'linestyle':'dashed'}]
            # get position of peak from derivative of fit
            if order == 2:
                if len(ap) == 3: # asymmetric fit
                    t_peak0 = x[x0] - ap[1]/ap[2] 
                else: # symmetric fit is by definition centered around x0
                    t_peak0 = x[x0]
            else:
                t_peak0 = x[x0] + newton(lambda x: polynomial_deriv(x, ap), x0)

        # 1st derivative data from Arduino
        if DERIV:
            # note: 
            # - the first num_edge*2 data is invalid and must be skipped
            #   the first valid sample deriv[2*num_edge] corresponds to data[num_edge] at t_high
            # - we scale raw data using num_edge to distinguish SG5 from SG9 filter
            if num_edge == 2: #SG5
                scaling = conv_y*filter_scaling['SG5']
            elif num_edge == 4: #SG9
                scaling = conv_y*filter_scaling['SG9']
            else:
                print('warning: unexpected num_edge %i is neither 2 nor 4? assume derivative scaling=1')
                scaling = conv_y*1000/ADC_TIME
            x = np.arange(samples-2*num_edge)*conv_x + dt_start
            y = data_d[2*num_edge:]*scaling
            data_list_2   += [[x, y]]
            data_labels_2 += [r'$dy/dx$ (Arduino)']
            data_args_2   += [{'color':'red', 's':20}]
            
            # index of first negative derivative and calculated t_peak from Arduino
            x0 = ticks_diff(t_trig, t_high) + (((i_peak-2*num_edge)*cal_ticks)/(2**cal_div))
            data_list_2   += [[x0/MCK, data_d[i_peak]*scaling], [t_peak/MCK, 0.0]]
            data_labels_2 += ['i_peak (Arduino)', 't_peak (Arduino)']
            data_args_2   += [{'facecolor':'None', 'edgecolor':'Blue','s':100, 'linewidth':2},
                              {'color':'Blue','marker': '+', 's':2000, 'linewidth':2}]

        # calculate derivative
        ddx         = None
        if DERIV_FUNC == 'diff': 
            # discreted difference
            print('discrete difference used for derivative')
            error = np.abs(1000/ADC_TIME - filter_scaling['diff'])
            if error > 1e-12:
                print('error filter_scaling %.3e' % (error))
                exit()            
            calc_edge = 0.5
            dx = dt_start + (np.arange(0, len(data)-1) + calc_edge - num_edge)*conv_x
            dy = (data[1:]-data[:-1])*conv_y*filter_scaling['diff']
        elif DERIV_FUNC == 'SG5': 
            # Savitzky-Golay filter should have factor 3 and not 2 as from paper [2,-2]
            # the difference is small and x2 is easier to calculate than x3
            c = savgol_coeffs(window_length=4, polyorder=1, deriv=1, delta=1)
            sc = np.min(np.abs(c))
            print('SG5 Savitzky-Golay FIR filter coefficients:', c/sc, 'scaling %.3e' % (sc))
            error = np.abs(sc*1000/ADC_TIME - filter_scaling['SG5'])
            if error > 1e-12:
                print('error filter_scaling %.3e' % (error))
                exit()                
            calc_edge = 2
            dx = np.arange(calc_edge, len(data)-calc_edge)*conv_x + dt_start - num_edge*conv_x
            dy = (3*data[4:] + data[3:-1] - data[1:-3] - 3*data[:-4])*conv_y*filter_scaling['SG5']
            if True:
                # alternative coefficients as power of 2 = 'orig'
                ddx_label = r'$dy/dx$ (%s pwr2)'%DERIV_FUNC
                ddx = dx
                ddy = (2*data[4:] + data[3:-1] - data[1:-3] - 2*data[:-4])*conv_y*filter_scaling['SG5']
            else:
                # second derivative using same filter
                ddx_label = r'$dy/dx$ (%s pwr2)'%DERIV_FUNC
                ddx = dx[calc_edge : -calc_edge]
                ddy = (3*dy[4:] + dy[3:-1] - dy[1:-3] - 3*dy[:-4])*5
        elif DERIV_FUNC == 'SG9': 
            # Savitzky-Golay filter [4,-4]
            c = savgol_coeffs(window_length=8, polyorder=1, deriv=1, delta=1)
            sc = np.min(np.abs(c))
            print('SG9 Savitzky-Golay FIR filter coefficients:', c/sc, 'scaling %.3e' % (sc))
            error = np.abs(sc*1000/ADC_TIME - filter_scaling['SG9'])
            if error > 1e-12:
                print('error filter_scaling %.3e' % (error))
                print([sc*1000/ADC_TIME, filter_scaling['SG9'], sc])
                exit()                
            calc_edge = 4
            dx = np.arange(calc_edge, len(data)-calc_edge)*conv_x + dt_start - num_edge*conv_x
            dy = (  (data[5:-3] - data[3:-5]) + 
                  3*(data[6:-2] - data[2:-6]) + 
                  5*(data[7:-1] - data[1:-7]) + 
                  7*(data[8:  ] - data[ :-8]) )*conv_y*filter_scaling['SG9']
            if True:
                # alternative coefficients as power of 2. the difference is small but calculation much more efficient.
                ddx_label = r'$dy/dx$ (%s pwr2)'%DERIV_FUNC
                ddx = dx
                ddy = (  (data[5:-3] - data[3:-5]) + 
                       2*(data[6:-2] - data[2:-6]) + 
                       4*(data[7:-1] - data[1:-7]) + 
                       8*(data[8:  ] - data[ :-8]) )*conv_y*filter_scaling['SG9']
            else:
                # second derivative
                ddx_label = r'$d^2y/dx^2$ (%s)'%DERIV_FUNC
                ddx = dx[calc_edge : -calc_edge]
                ddy = (  (dy[5:-3] - dy[3:-5]) + 
                       3*(dy[6:-2] - dy[2:-6]) + 
                       5*(dy[7:-1] - dy[1:-7]) + 
                       7*(dy[8:  ] - dy[ :-8]) )/10
        elif DERIV_FUNC == 'Scipy':
            # use Scipy.signal.savgol_filter
            window_length = 8 # even gives smaller range of coefficients
            polyorder     = 1
            deriv         = 1
            delta         = 1
            c = savgol_coeffs(window_length=window_length, polyorder=polyorder, deriv=deriv, delta=delta)
            sc = np.min(np.abs(c))
            print('Scipy using Savitzky-Golay FIR filter coefficients:', c/sc, 'scaling %.3e' % (sc))
            calc_edge = int(window_length//2) - 0.5 # TODO: had to shift by 0.5 for better agreement?
            dx = (np.arange(0, len(data))+num_edge-calc_edge)*conv_x + dt_start
            dy = savgol_filter(data, window_length=window_length, polyorder=polyorder, deriv=deriv, delta=delta)*conv_y*1000/ADC_TIME

        curve_list_2   += [[dx, dy]]
        curve_labels_2 += [r'$dy/dx$ (%s)'%DERIV_FUNC]
        curve_args_2   += [{'color':colors[peak_to_read], 'linewidth':1, 'linestyle':'solid'}]

        if not DERIV:
            # without derivative use DERIV_FUNC to get y-value of Arduino i_peak
            # Arduino i_peak = index of first negative derivative
            y0 = dy[i_peak-2*num_edge]
            x0 = ticks_diff(t_trig, t_high) + (((calc_edge - num_edge + (i_peak-2*num_edge))*cal_ticks)/(2**cal_div))
            data_list_2   += [[x0/MCK, y0], [t_peak/MCK, 0.0]]
            data_labels_2 += ['i_peak (Arduino)', 't_peak (Arduino)']
            data_args_2   += [{'facecolor':'None', 'edgecolor':'Blue','s':100, 'linewidth':2},
                         {'color':'Blue','marker': '+', 's':2000, 'linewidth':2}]

        if ddx is not None:
            curve_list_2   += [[ddx, ddy]]
            curve_labels_2 += [ddx_label]
            curve_args_2   += [{'color':colors[peak_to_read], 'linewidth':1, 'linestyle':'dashed'}]

            if False:                         
                # get min/max 2nd derivative
                minmax2 = np.array([np.min(ddy), np.max(ddy)])
                i_minmax2 = [np.where(ddy == m)[0][0] for m in minmax2]
                print('\n[min, max] 2nd derivative [%.3f, %.3f] V = [%.3f, %.3f] ATW at index [%i, %i] = [%.3f, %.3f] us' % (
                    minmax2[0], minmax2[1], 
                    minmax2[0]/conv_y, minmax2[1]/conv_y, 
                    i_minmax2[0], i_minmax2[1], 
                    ddx[i_minmax2[0]], ddx[i_minmax2[1]]))

        # get index of first negative derivative     
        index = np.where(np.sign(dy) < 0)[0]
        i_peak_calc = index[0]
        print('derivative zero between index [%i,%i] = [%.3f, %.3f] us at dy [%.3f, %.3f] mV/us with difference %.3f mV/us' % (
            i_peak_calc-1, i_peak_calc, 
            dt_start + (calc_edge - num_edge)*conv_x + (i_peak_calc-1)*conv_x, 
            dt_start + (calc_edge - num_edge)*conv_x + (i_peak_calc  )*conv_x, 
            dy[i_peak_calc-1], dy[i_peak_calc], 
            dy[i_peak_calc] - dy[i_peak_calc-1], 
            ))

        # calculate peak time in ticks
        x0_calc = ticks_diff(t_trig, t_high) + (((calc_edge - num_edge + i_peak_calc)*cal_ticks)/(2**cal_div))
        x = x0_calc + np.arange(-int(NUM_FIT_POINTS//2), NUM_FIT_POINTS-int(NUM_FIT_POINTS//2))*cal_ticks/(2**cal_div)
        y = dy[i_peak_calc - int(NUM_FIT_POINTS//2) : i_peak_calc + NUM_FIT_POINTS-int(NUM_FIT_POINTS//2)]

        if NUM_FIT_POINTS == 2: 
            # linear regression with NUM_FIT_POINTS = 2 
            t_peak_calc = x0_calc + (-1 + dy[i_peak_calc-1]/(dy[i_peak_calc-1]-dy[i_peak_calc]))*cal_ticks/(2**cal_div)
            params = [(x[1]*y[0]-x[0]*y[1])/(x[1]-x[0]), (y[1]-y[0])/(x[1]-x[0])]
        else:
            # linear regression
            if len(x) != NUM_FIT_POINTS or len(y) != NUM_FIT_POINTS:
                print('error number fit points [%i, %i] != %i' % (len(x), len(y), NUM_FIT_POINTS))
                exit()
            params = np.array([np.sum(x*x)*np.sum(y) - np.sum(x)*np.sum(x*y), len(x)*np.sum(x*y) - np.sum(x)*np.sum(y)])/(len(x)*np.sum(x*x) - np.sum(x)**2)
            error  = np.sqrt(np.array([np.sum(x*x), len(x)])*np.sum((params[0] + x*params[1] - y)**2)/((len(x)*np.sum(x*x) - np.sum(x)**2)*(len(x) - len(params))))
            
            t_peak_calc = -(np.sum(x*x)*np.sum(y) - np.sum(x)*np.sum(x*y))/(len(x)*np.sum(x*y) - np.sum(x)*np.sum(y))
            if np.abs(t_peak_calc + params[0]/params[1]) > 1e-10:
                print('error zero crossing %.3f != %.3f (difference %.3e)' % (t_peak_calc, -params[0]/params[1], np.abs(t_peak_calc + params[0]/params[1])))
                exit()         
            print('linear regression with %i points: a0 = %s, a1 = %s' % (NUM_FIT_POINTS, str_error(params[0], error[0]), str_error(params[1], error[1])))
            
        data_list_2    += [[x/MCK, y]]
        data_labels_2  += ['fit points']
        data_args_2    += [{'facecolor':'None', 'edgecolor':'Red', 's':50, 'linewidth':2}]
        curve_list_2   += [[x/MCK, params[0]+params[1]*x]]
        curve_labels_2 += ['fit curve']
        curve_args_2   += [{'color':'Red', 'linewidth':2}]

        # add calculated peak position
        data_list_2   += [[x0_calc/MCK, dy[i_peak_calc]], [t_peak_calc/MCK, params[0] + params[1]*t_peak_calc]]
        data_labels_2 += ['i_peak (calc)', 't_peak (calc)']
        data_args_2   += [{'facecolor':'None', 'edgecolor':'Red' ,'s':100, 'linewidth':2},
                          {'color':'Red' ,'marker': '+', 's':2000, 'linewidth':2}]

        #q0 = dy[i_peak_calc-1]/conv_y
        #q1 = dy[i_peak_calc]/conv_y
        #test = ticks_diff(t_trig, t_high) + (((i_peak_calc-(num_edge<<1))*(q0-q1) + q0)*(cal_ticks>>cal_div))/(q0-q1)
        #print(t_peak_calc/MCK, test/MCK, abs(t_peak_calc-test)/MCK, (((i_peak_calc-(num_edge<<1))*(q0-q1) + q0)*(cal_ticks>>cal_div)), q0-q1)

        if (i_peak-2*num_edge) != (i_peak_calc + calc_edge - num_edge): 
            # different derivative functioncs might have +/-1 difference in peak index
            print('\nwarning: peak index from Arduino %i != calculated %i!\n' % (i_peak-2*num_edge, i_peak_calc + calc_edge - num_edge))
        
        if abs(t_peak - t_peak_calc) > 20:
            print('\nwarning: peak position %.3f us from Arduino != calculated %.3f us, difference %.3f us (%i ticks)!\n' % (t_peak/MCK, t_peak_calc/MCK, abs(t_peak - t_peak_calc)/MCK, abs(t_peak - t_peak_calc)))
        else:
            print('note: peak position %.3f us from Arduino within difference %.3f us (%.3f ticks) of python.\n' % (t_peak/MCK, abs(t_peak - t_peak_calc)/MCK, abs(t_peak - t_peak_calc)))


        if True:
            # plot data
            # num_edge samples are sent before t_high and num_edge after t_low
            # data[num_edge] corresponds to t_high
            lists  = data_list + curve_list + data_list_2 + curve_list_2
            xmin   = np.min([np.min(d[0]) for d in lists])
            xmax   = np.max([np.max(d[0]) for d in lists])
            lists  = data_list + curve_list
            ymin   = np.min([np.min(d[1]) for d in lists])
            ymax   = np.max([np.max(d[1]) for d in lists])
            lists  = data_list_2 + curve_list_2
            ymin_2 = np.min([np.min(d[1]) for d in lists])
            ymax_2 = np.max([np.max(d[1]) for d in lists])
            ax = plot(
                  title         = 'scanning transfer cavity lock: peak %i'%peak_to_read,
                  data          = data_list, 
                  data_labels   = data_labels,
                  data_args     = data_args,
                  curves        = curve_list,
                  curves_labels = curve_labels,
                  curves_args   = curve_args,
                  xlabel        = r'time ($\mu$s)',
                  ylabel        = 'signal (V)',
                  xrng          = [xmin-0.1*(xmax-xmin), xmax+0.1*(xmax-xmin)],
                  pos           = [0.06, 0.08, 0.86, 0.86]
                  )
            ax2 = plot(
                  twin          = ax,
                  data          = data_list_2, 
                  data_labels   = data_labels_2,
                  data_args     = data_args_2,
                  curves        = curve_list_2,
                  curves_labels = curve_labels_2, 
                  curves_args   = curve_args_2,
                  ylabel        = r'derivative (mV/$\mu$s)',
                  legend_args   = {'bbox_to_anchor':(0.97, 0.93), 'loc':'upper right', 'frameon':True, 'fontsize':10, 'markerscale':0.3}
                  )
            # add zero
            ax2.axhline(y=0.0, color='Gray', linestyle='dotted')

            if False:
                # add thresholds
                x = [ticks_diff(t_trig, t_low )/MCK, 
                     ticks_diff(t_trig, t_high)/MCK]
                y = THRESHOLD
                dx = 7.5    # size x in mu
                dy = 0.1    # size y in V
                for xi,yi in zip(x,y):
                    ax.hlines([yi], xmin=xi-dx, xmax=xi+dx, colors='Black', linestyles='dotted')
                    ax.vlines([xi], ymin=yi-dy, ymax=yi+dy, colors='Black', linestyles='dotted')
            
            if False:
                # add peak position
                x = [t_peak/MCK, t_peak_calc/MCK]
                y = [0,0]
                styles = [{'colors':'Red'  , 'linestyles':'solid' , 'linewidth':3},
                          {'colors':'Black', 'linestyles':'dotted', 'linewidth':3}]
                dx = 7.5    # size x in mu
                dy = 0.1    # size y in V
                for xi,yi,style in zip(x,y,styles):
                    ax.hlines([yi], xmin=xi-dx, xmax=xi+dx, **style)
                    ax.vlines([xi], ymin=yi-dy, ymax=yi+dy, **style)
    
    plt.show()

    print('done')
