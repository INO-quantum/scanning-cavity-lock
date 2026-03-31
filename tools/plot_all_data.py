#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# plot all raw data peaks on top of each other

import numpy as np

from os import listdir
from os.path import isfile, split, splitext, join

import matplotlib as mpl
import matplotlib.pyplot as plt
# for save button use current working directory
mpl.rcParams["savefig.directory"] = ""
#mpl.rcParams['font.family'] = 'sans-serif' # 'DejaVu Sans'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.linewidth'] = 2

conv_y = 3.3/((2**12)-1)            # conversion y-axis into Volts

# peak index 0-2, 3=all to load
PEAK = 1

# if True plot derivative from Arduino
DERIV = True

# center peaks using calculated or Arduino peak position, if None use absolute position
CENTER = [None, 'Due', 'calc'][1]

# folder and file from where to load files
# first index '%i' = peak, 2nd index pattern = repetition
pattern = '_%i'
load_name = './sample_data/raw_%i.csv'

# peak colors
colors = ['Blue', 'Orange', 'Green', 'Violet', 'Brown', 'Magenta']

MCK             = 84.0              # master clock in MHz
TICKS_RELOAD    = (2**24)-1         # reload value for ticks

# buffers
NUM_BUFFER      = 8
BUF_SIZE        = 128

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
        if isinstance(legend_args, bool): largs = None 
        else:                             largs.update(legend_args)
    if largs is not None:
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
    error = False
    
    # get number of buffers to read
    n_read = 1;
    n = n_high
    while n != n_low:
        n = (n + 1) % NUM_BUFFER
        n_read += 1

    # conversion ADC samples to us
    conv_x = float(cal_ticks)/(MCK*(2**cal_div))

    if peak != peak_to_read:
        print('unexpected peak %i != %i' % (peak, peak_to_read))
        error = True

    if tot_buf != NUM_BUFFER:
        print('unexpected number of buffers %i != %i' % (tot_buf, NUM_BUFFER))
        error = True
        
    if len(t_buf) != NUM_BUFFER:
        print('unexpected number of buffer times %i != %i' % (len(t_buf), NUM_BUFFER))
        error = True

    expect = (n_read-1)*BUF_SIZE + i_low - i_high + 2*num_edge
    if samples != expect:
        print('unexpected number of samples %i != %i (%i buffers to read)' % (samples, expect, n_read))
        error = True

    # ensure t_buf[n_high] <= t_high <= buffer duration 
    # if n_low != n_high we can check also that t_high <= t_buf[n_low] 
    n_next = (n_high + 1) % NUM_BUFFER
    if (ticks_diff(t_buf[n_high], t_high)/MCK > BUF_SIZE*conv_x):
        print('unexpected t_high = %.3f us after buffer %i start > buffer duration %.3f us' % (ticks_diff(t_buf[n_high], t_high)/MCK, n_high, BUF_SIZE*conv_x))
        error = True
    elif (n_low != n_high) and (ticks_diff(t_high, t_buf[n_next])/MCK > BUF_SIZE*conv_x):
        print('unexpected t_high = %.3f us before next buffer %i start > buffer duration %.3f us' % (ticks_diff(t_high, t_buf[n_next])/MCK, n_next, BUF_SIZE*conv_x))
        error = True
    # ensure t_buf[n_low] <= t_low <= buffer duration
    # we cannot check t_high <= t_buf[n_next] since n_next is most likely an old buffer written before n_low.
    if (ticks_diff(t_buf[n_low], t_low)/MCK > BUF_SIZE*conv_x):
        print('unexpected t_low = %.3f us after buffer %i start > buffer duration %.3f us' % (ticks_diff(t_buf[n_low], t_low)/MCK, n_low, BUF_SIZE*conv_x))
        error = True

    # check that i_high and t_high agree within 30 ticks = 0.36us
    delta = abs(ticks_diff(t_buf[n_high], t_high)/MCK - i_high*conv_x)
    if delta > 30/MCK:
        print('unexpected time difference between t_high %.3f us and i_high %.3f us from buffer %i start time %.3f us (%.3f us, %i ticks difference)' % (ticks_diff(t_buf[n_high], t_high)/MCK, i_high*conv_x, n_high, ticks_diff(t_trig, t_buf[n_high])/MCK, delta, delta*MCK))
        error = True
    else:
        print('note: |t_high - t_low| = |%.3f us - %.3f us| = %.3f us (%i ticks, buffer %i start time %.3f us)' % (ticks_diff(t_buf[n_high], t_high)/MCK, i_high*conv_x, delta, delta*MCK, n_high, ticks_diff(t_trig, t_buf[n_high])/MCK))
    
    # check that i_low and t_low agree within 40 ticks = 0.48us
    # i_low has always a larger difference than i_high?
    delta = abs(ticks_diff(t_buf[n_low], t_low)/MCK - i_low*conv_x)
    if delta > 40/MCK:
        print('unexpected time difference between t_low %.3f us and i_low %.3f us from buffer %i start time %.3f us (%.3f us, %i ticks difference)' % (ticks_diff(t_buf[n_low], t_low)/MCK, i_low*conv_x, n_low, ticks_diff(t_trig, t_buf[n_low])/MCK, delta, delta*MCK))
        error = True
    else:
        print('note: |t_low - i_low| = |%.3f us - %.3f us| = %.3f us (%i ticks, buffer %i start time %.3f us)' % (ticks_diff(t_buf[n_low], t_low)/MCK, i_low*conv_x, delta, delta*MCK, n_low, ticks_diff(t_trig, t_buf[n_low])/MCK))

    if (DERIV and get_deriv != 1):
        print('unexpected deriv bit %i != 1' % (get_deriv))
        error = True
    elif (not DERIV and get_deriv != 0):
        print('unexpected deriv bit %i != 0' % (get_deriv))
        error = True

    print('peak %i, buffer %i-%i (%i), %i samples, position %i, time %.3f us (%i ticks)' % (peak, n_high, n_low, n_read, samples, i_peak, t_peak/MCK, t_peak))
    print('time  [trig, high, low] = [%i, %i, %i] ticks = [0, %.3f, %.3f] us' % (t_trig, t_high, t_low, ticks_diff(t_trig, t_high)/MCK, ticks_diff(t_trig, t_low)/MCK))
    print('index [high, low] = [%i, %i]' % (i_high, i_low))
    print('time buffer = [' + (', '.join(['%i'%tb for tb in t_buf])) + '] ticks')
    print('            = [' + (', '.join(['%.3f'%(ticks_diff(t_trig, tb, True)/MCK) for tb in t_buf])) + '] us')
    print('ticks/ADC = %i/2^%i = %.3f = %.3f us/ADC' % (cal_ticks, cal_div, float(cal_ticks)/2**cal_div, conv_x))
    
    if error: 
        print('\nstop due to previous error(s)\n')
        exit()

    return n_read, conv_x
    
if __name__ == '__main__':

    data_list    = []
    data_labels  = []
    data_args    = []
    curve_list   = []
    curve_labels = []
    curve_args   = []

    peaks = [0,1,2] if PEAK == 3 else [PEAK]
    for peak_to_read in peaks:

        # get matching filenames in folder
        # find index of next file with pattern added
        start, end = pattern.split('%i')
        path, name = split(load_name%peak_to_read)
        name, ext  = splitext(name)
        zerolen = len(name) + len(start) + len(end) + len(ext)
        files = [f for f in listdir(path) if isfile(join(path,f)) and len(f) > zerolen and f.startswith(name) and f.endswith(ext)]
        
        for fname in files:

            findex = int(fname[len(name)+len(start):-len(end)-len(ext)])
            print('\nreading Arduino peak %i, index %i from file:'%(peak_to_read, findex), fname)

            # load data from file
            line_data = []
            with open(join(path, fname), 'r') as f:
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
            # flags
            peak      = header['peak']
            n_high    = header['n_high']
            n_low     = header['n_low']
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
            error     = header['error']

            # check numbers
            n_read, conv_x = check_header(peak_to_read)

            # read data
            data = []
            for i,l in enumerate(line_data[index:index+samples]):
                ii, val = [int(li) for li in l]
                if ii != i:
                    print(fname%peak_to_read, 'error data index %i != %i' % (ii, i))
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
                        print(fname%peak_to_read, 'error derivative index %i != %i' % (ii, i))
                        exit()
                    data_d.append(val)
                data_d = np.array(data_d)
                
                # find first negative derivative value
                # first 2*num_edge data is invalid and must be skipped. index includes this data.
                index = 2*num_edge + np.where(data_d[2*num_edge:] < 0)[0][0]
                if index != i_peak:
                    #print(data_d)
                    print('error: first negative derivative at index %i != %i reported by Arduino!' % (index, i_peak))
                    exit()
                elif np.any(data_d[:2*num_edge] < 0):
                    print('\nwarning: negative derivative in first %i data should never happen!?' % (2*num_edge))
                    print(data_d[:2*num_edge], '\n')
                
                # calculate peak position in ticks
                q0 = data_d[index-1]
                q1 = data_d[index  ]
                t_peak_calc = ticks_diff(t_trig, t_high) + (((i_peak-2*num_edge-1 + q0/(q0-q1))*cal_ticks)/2**cal_div)
                if True: # check Arduino overflow
                    q0 = np.array(data_d[index-1], dtype=np.int32)
                    q1 = np.array(data_d[index  ], dtype=np.int32)
                    cal_ticks = np.array(cal_ticks, dtype=np.uint32)
                    cal_div   = np.array(cal_div  , dtype=np.int32)
                    if False: # this overflows by one bit!
                        tmp2 = (((i_peak-2*num_edge-1)*(q0-q1)) + q0)*cal_ticks
                        tmp = np.array(i_peak-2*num_edge-1           , dtype=np.uint32)
                        tmp = np.array(((tmp*(q0-q1)) + q0)*cal_ticks, dtype=np.uint32)
                    else: # this does not overflow when (cal_ticks << 12) < 2^32
                        tmp2 = np.array((i_peak-2*num_edge-1 + q0/(q0-q1))*cal_ticks, dtype=np.uint32)
                        tmp  = np.array((i_peak-2*num_edge-1)*cal_ticks, dtype=np.uint32)
                        tmp += np.array((q0*cal_ticks)/(q0-q1), dtype=np.uint32)
                    if tmp2 != tmp:
                        print('\nArduino uint32_t overflow! %i (%i bits) != %i (%i bits)' % (tmp, np.ceil(np.log2(tmp)), tmp2, np.ceil(np.log2(tmp2))))
                        print('q0*cal_ticks = %i (%i bits)' % (q0*cal_ticks, np.ceil(np.log2(q0*cal_ticks))))
                        exit()
                    tmp = np.array(tmp/(q0-q1), dtype=np.uint32)
                    t_peak_ovl = ticks_diff(t_trig, t_high) + (tmp>>cal_div)
            else:
                # i_peak = index of peak obtained from Arduino = index of first negative derivative shifted
                t_peak_calc = ticks_diff(t_trig, t_high) + (((i_peak-2*num_edge)*cal_ticks)>>cal_div)

            # start time in us        
            dt_start = ticks_diff(t_trig, t_high)/MCK
            
            if True: # plot Arduino found peak position in derivative curve
                x = t_peak/MCK
                if CENTER == 'calc':
                    x -= t_peak_calc/MCK
                elif CENTER == 'Due':
                    x -= t_peak/MCK
                data_list   += [[x, 0]]
                data_labels += ['peak %i'%findex]
                data_args   += [{'color':colors[peak_to_read], 's':20}]
        
            if abs(t_peak - t_peak_calc) >= MCK:
                print('\nwarning: peak position %.3f us from Arduino != calculated %.3f us, difference %.3f us (%i ticks)!\n' % (t_peak/MCK, t_peak_calc/MCK, abs(t_peak - t_peak_calc)/MCK, abs(t_peak - t_peak_calc)))
                #exit()
            else:
                print('note: peak position %.3f us from Arduino within difference %.3f us (%i ticks) of python.' % (t_peak/MCK, abs(t_peak - t_peak_calc)/MCK, abs(t_peak - t_peak_calc)))

            # peak data from Arduino
            x = (np.arange(samples)-num_edge)*conv_x + dt_start
            y = data*conv_y
            if CENTER == 'calc':
                x -= t_peak_calc/MCK
            elif CENTER == 'Due':
                x -= t_peak/MCK
            curve_list   += [[x, y]]
            curve_labels += ['data %i'%findex] #, r'$dy/dx$', r'$d^2y/dx^2$']
            curve_args   += [{'color':colors[peak_to_read], 'linewidth':1}] #, {'color':'Red', 's':20},]

            # 1st derivative data from Arduino
            if DERIV:
                # note: the first num_edge*2 data is invalid and must be skipped
                #       the first valid sample deriv[2*num_edge] corresponds to data[num_edge] at t_high
                #d_scale = np.max(np.abs(data_d))
                #print('note: derivative scale %.3e' % d_scale)
                x = np.arange(samples-2*num_edge)*conv_x + dt_start
                y = data_d[2*num_edge:]*conv_y/10
                if CENTER == 'calc':
                    x -= t_peak_calc/MCK
                elif CENTER == 'Due':
                    x -= t_peak/MCK
                curve_list   += [[x, y]]
                curve_labels += [r'$dy/dx$ %i'%findex] #, r'$d^2y/dx^2$']
                curve_args   += [{'color':'red', 'linewidth':1}]
            
            #break

    if True:
        # plot data
        # num_edge samples are sent before t_high and num_edge after t_low
        # data[num_edge] corresponds to t_high
        ax = plot(
              title         = 'scanning transfer cavity lock: '+ ('all peaks' if PEAK==3 else 'peak %i'%PEAK) +' ADC data',
              data          = data_list, 
              data_labels   = data_labels,
              data_args     = data_args,
              curves        = curve_list,
              curves_labels = curve_labels,
              curves_args   = curve_args,
              xlabel        = r'time ($\mu$s)',
              ylabel        = 'signal (V)',
              pos           = [0.065, 0.08, 0.88, 0.86],
              legend_args   = False,
              )
    
    plt.show()

    print('done')
