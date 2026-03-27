#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# get statistics from "p#r\n" and "Ls#r\n" data obtained with serial monitor

import numpy as np
from scipy.optimize import curve_fit

import matplotlib as mpl
import matplotlib.pyplot as plt
# for save button use current working directory
mpl.rcParams["savefig.directory"] = ""
#mpl.rcParams['font.family'] = 'sans-serif' # 'DejaVu Sans'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.linewidth'] = 2

#timing
MCK             = 84.0              # master clock in MHz

# conversion y-axis into Volts
conv_y = {'scaling':(5/6-1/6)*3.3/((2**12)-1), 'offset':1/6*3.3}

# cavity free spectral range in MHz
FSR = 1500

# errors in MHz of in mu
ERROR_MHZ = True
ERROR_UNIT = 'MHz' if ERROR_MHZ else r'$\mu$s'

# arbitrary +/- error limit shown as horizontal lines in error vs. time
if ERROR_MHZ: error_limit = 0.2 # MHz
else:         error_limit = 0.5 # us

# Arduino settings
laser_set = 1240
laser_ref = 2400
ramp_set  = 550
num_avg   = [None,32][0]
ramp_freq = 169
max_error = 1.0

# different configurations per file
conf_3  = {'peaks': 3, 'FSR': 3000}
conf_6h = {'peaks': 6, 'FSR': 2400}
conf_6  = {'peaks': 6, 'FSR': 2400, 'laser_set':1240}

# dict of labels and (filename, config, data style, curve style)
if True: # noise
    fnames = {#'test'  : ('./data/20260311/p3_7', conf_3, {'color':'Red'    , 's':30}, {'color':'Red'    , 'linewidth':1}),
              #'ki/kp=100/2k'  : ('./data/20260313/p3r_4', conf_3, {'color':'Red'    , 's':30}, {'color':'Red'    , 'linewidth':1}),
              #'ki/kp=100/1.5k': ('./data/20260313/p3r_6', conf_3, {'color':'Cyan'   , 's':30}, {'color':'Cyan'   , 'linewidth':1}),
              #'ki/kp=100/1k'  : ('./data/20260313/p3r_1', conf_3, {'color':'Blue'   , 's':30}, {'color':'Blue'   , 'linewidth':1}),
              ##'ki/kp= 50/2k (3 peaks)'   : ('./data/20260313/p3r_3', conf_3, {'color':'Blue' , 's':30}, {'color':'Blue' , 'linewidth':1}), #ref
              #'ki/kp= 50/1.5k': ('./data/20260313/p3r_5', conf_3, {'color':'Magenta', 's':30}, {'color':'Magenta', 'linewidth':1}),
              #'ki/kp= 50/1k'  : ('./data/20260313/p3r_2', conf_3, {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
              #'ki/kp= 65/1.25k (6 peaks)': ('./data/20260313/p6r_7', conf_6, {'color':'Orange'  , 's':30}, {'color':'Orange'  , 'linewidth':1}),
              ##'ki/kp= 65/1.25k (6 peaks 2)': ('./data/20260313/p6r_9', conf_6, {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}), # storng jumps caused by ramp
              #'ki/kp= 65/1.25k': ('./data/20260317/p6r_3', conf_6, {'color':'Orange' , 's':30}, {'color':'Orange'  , 'linewidth':1}),
              #'ki/kp= 20/1.25k': ('./data/20260317/p6r_8', conf_6, {'color':'Orange' , 's':30}, {'color':'Orange' , 'linewidth':1}),
              #'ki/kp= 30/1.25k': ('./data/20260317/p6r_7', conf_6, {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
              #'ki/kp= 40/1.25k': ('./data/20260317/p6r_9', conf_6, {'color':'Magenta', 's':30}, {'color':'Magenta', 'linewidth':1}),
              #'ki/kp= 30/0.5k': ('./data/20260317/p6r_10', conf_6, {'color':'Orange' , 's':30}, {'color':'Orange' , 'linewidth':1}),
              #'ki/kp= 30/1.0k': ('./data/20260317/p6r_17', conf_6, {'color':'Orange' , 's':30}, {'color':'Orange' , 'linewidth':1}),
              #'ki/kp= 30/1.5k': ('./data/20260317/p6r_12', conf_6, {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
              #'ki/kp= 30/2.0k': ('./data/20260317/p6r_16', conf_6, {'color':'Magenta', 's':30}, {'color':'Magenta', 'linewidth':1}),
              #'ki/kp= 30/2.5k': ('./data/20260317/p6r_14', conf_6, {'color':'Magenta', 's':30}, {'color':'Magenta', 'linewidth':1}),
              #'ki/kp= 30/1.5k 16': ('./data/20260317/p6r_19', conf_6, {'color':'Orange' , 's':30}, {'color':'Orange' , 'linewidth':1}),
              #'ki/kp= 30/1.5k 32': ('./data/20260317/p6r_18', conf_6, {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
              #'ki/kp= 30/1.5k 64': ('./data/20260317/p6r_20', conf_6, {'color':'Magenta', 's':30}, {'color':'Magenta', 'linewidth':1}),
              #'ki/kp= 30/1.5k (fit 4)': ('./data/20260319/p6r_0', conf_6, {'color':'Orange'  , 's':30}, {'color':'Orange'  , 'linewidth':1}),
              #'ki/kp= 30/1.5k (test)': ('./data/20260320/p6r_3', conf_6, {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
              ##'ramp ki/kp=500/20' : ('./data/20260327/p6r_20', conf_6, {'color':'Green', 's':30}, {'color':'Green', 'linewidth':2}),
              'test' : ('./data/20260327/p6r_22', conf_6, {'color':'Green', 's':30}, {'color':'Green', 'linewidth':2}),
             }
if False: # step function
    fnames = {#'ki/kp=100/2k'  : ('./data/20260313/Ls20r_4', conf_3, {'color':'Red'    , 's':30}, {'color':'Red'    , 'linewidth':1}),
              #'ki/kp=100/1.5k': ('./data/20260313/Ls20r_6', conf_3, {'color':'Cyan'   , 's':30}, {'color':'Cyan'   , 'linewidth':1}),
              #'ki/kp=100/1k'  : ('./data/20260313/Ls20r_1', conf_3, {'color':'Blue'   , 's':30}, {'color':'Blue'   , 'linewidth':1}),
              #'ki/kp= 50/2k (3 peaks)'   : ('./data/20260313/Ls20r_3', conf_3, {'color':'Blue' , 's':30}, {'color':'Blue' , 'linewidth':1}), # ref
              #'ki/kp= 50/1.5k': ('./data/20260313/Ls20r_5', conf_3, {'color':'Magenta', 's':30}, {'color':'Magenta', 'linewidth':1}),
              #'ki/kp= 50/1k'  : ('./data/20260313/Ls20r_2', conf_3, {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
              #'ki/kp= 65/1.25k (6 peaks)': ('./data/20260313/Ls20r_7', conf_6, {'color':'Orange'  , 's':30}, {'color':'Orange'  , 'linewidth':1}),
              #'ki/kp= 65/1.25k (6 peaks 2)': ('./data/20260313/Ls20r_8', conf_6, {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
              #'ki/kp= 65/1.25k': ('./data/20260317/Ls20r_3', conf_6, {'color':'Orange' , 's':30}, {'color':'Orange' , 'linewidth':1}),
              #'ki/kp= 20/1.25k': ('./data/20260317/Ls20r_8', conf_6, {'color':'Orange' , 's':30}, {'color':'Orange' , 'linewidth':1}),
              #'ki/kp= 30/1.25k': ('./data/20260317/Ls20r_7', conf_6, {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
              #'ki/kp= 40/1.25k': ('./data/20260317/Ls20r_9', conf_6, {'color':'Magenta', 's':30}, {'color':'Magenta', 'linewidth':1}),
              #'ki/kp= 30/0.5k': ('./data/20260317/Ls20r_10', conf_6, {'color':'Orange' , 's':30}, {'color':'Orange' , 'linewidth':1}),
              #'ki/kp= 30/1.0k': ('./data/20260317/Ls20r_11', conf_6, {'color':'Orange' , 's':30}, {'color':'Orange' , 'linewidth':1}),
              #'ki/kp= 30/1.5k': ('./data/20260317/Ls20r_12', conf_6, {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
              #'ki/kp= 30/2.0k': ('./data/20260317/Ls20r_16', conf_6, {'color':'Magenta', 's':30}, {'color':'Magenta', 'linewidth':1}),
              #'ki/kp= 30/2.5k': ('./data/20260317/Ls20r_14', conf_6, {'color':'Magenta', 's':30}, {'color':'Magenta', 'linewidth':1}),
              #'ki/kp= 30/1.5k 16': ('./data/20260317/Ls20r_19', conf_6, {'color':'Orange' , 's':30}, {'color':'Orange' , 'linewidth':1}),
              #'ki/kp= 30/1.5k 32': ('./data/20260317/Ls20r_18', conf_6, {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
              #'ki/kp= 30/1.5k 64': ('./data/20260317/Ls20r_20', conf_6, {'color':'Magenta', 's':30}, {'color':'Magenta', 'linewidth':1}),
              #'ki/kp= 30/1.5k (fit 4)': ('./data/20260319/Ls20r_0', conf_6, {'color':'Blue'  , 's':30}, {'color':'Blue'  , 'linewidth':1}),
              #'laser ref': ('./data/20260327/Ls20r_0', conf_6, {'color':'Blue', 's':30}, {'color':'Blue', 'linewidth':2}),
              #'ramp ref' : ('./data/20260327/Rs20r_9', conf_6, {'color':'Orange', 's':30}, {'color':'Orange', 'linewidth':2}),
              'ramp ki/kp=500/20' : ('./data/20260327/p6r_20', conf_6, {'color':'Green', 's':30}, {'color':'Green', 'linewidth':2}),
             }

if False: # hysteresis
    fnames = {
        '300' : ('./data/20260316/p6r_300' , dict({'laser_set': 300}, **conf_6), {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
        '450' : ('./data/20260316/p6r_450' , dict({'laser_set': 450}, **conf_6), {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
        '600' : ('./data/20260316/p6r_600' , dict({'laser_set': 600}, **conf_6), {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
        '750' : ('./data/20260316/p6r_750' , dict({'laser_set': 750}, **conf_6), {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
        '900' : ('./data/20260316/p6r_900' , dict({'laser_set': 900}, **conf_6), {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
        '950' : ('./data/20260316/p6r_950' , dict({'laser_set': 950}, **conf_6), {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
        '1000': ('./data/20260316/p6r_1000', dict({'laser_set':1000}, **conf_6), {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
        '1150': ('./data/20260316/p6r_1150', dict({'laser_set':1150}, **conf_6), {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
        '1300': ('./data/20260316/p6r_1300', dict({'laser_set':1300}, **conf_6), {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
        '1450': ('./data/20260316/p6r_1450', dict({'laser_set':1450}, **conf_6), {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
        '1600': ('./data/20260316/p6r_1600', dict({'laser_set':1600}, **conf_6), {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
        '1750': ('./data/20260316/p6r_1750', dict({'laser_set':1750}, **conf_6), {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
        '1900': ('./data/20260316/p6r_1900', dict({'laser_set':1900}, **conf_6), {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
        '2050': ('./data/20260316/p6r_1900', dict({'laser_set':1900}, **conf_6), {'color':'Green'  , 's':30}, {'color':'Green'  , 'linewidth':1}),
    }

def plot(data, data_labels, data_args=None, 
         curves=[], curves_labels=[], curves_args=None, 
         title=None, xlabel=None, ylabel=None, 
         xrng=None, yrng=None, xlog=False, ylog=False,
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
        
    if xlog:
        ax.set_xscale('log')
    if ylog:
        ax.set_yscale('log')

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

#returns string in the form value(error)
def str_error(value, error):
    try:
        pwr = np.log10(error)
        #print(pwr)
        pwr = -int(np.floor(pwr))
        #print(pwr)
        fmt = '%%.%if(%%i)' % (pwr)
        #print(fmt)
        txt = fmt % (round(value,pwr),int(round(error*10**pwr)))
        #print(txt)
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

def histogram(y, num_bins):
    y,x = np.histogram(y, bins=num_bins)
    x = (x[1:] + x[0:-1])/2 # center of bins
    if True:
        # remove empty bins
        mask = (y > 0)
        x = x[mask]
        y = y[mask]
    return (x,y)
    
def FFT(y, rate):
    # perform FFT
    fft = np.fft.fft(y)

    # get frequency in Hz
    fft_f = np.fft.fftfreq(len(fft))*rate
    if True: # plot only positive side
        mask = (fft_f >= 0)
        fft_f = fft_f[mask]
    else:
        mask = np.ones(shape=(len(fft_f),),dtype=np.bool)

    # get amplitude
    fft_mag = (np.abs(fft)/len(fft))[mask]
    fft_mag[1:] *= 2 # DC is summed only once, while >0 frequencies have positive and negative components summed up, giving the factor 2.
    DC = fft_mag[0]*np.sign(np.real(fft[0])) # DC has also a sign. imag part must be 0.

    # get phase
    fft_phase = np.angle(fft[mask])
    
    return (fft_f, fft_mag, fft_phase)
    
def resample(x, y):
    # resamples y(x) for num equidistant x-values using Gaussian filtering of nearest points
    dx = np.diff(x)
    assert np.all(dx >= 0)
    # average same x-values. TODO: I think with cumsum can be faster
    xa = np.unique(x)
    ya = np.array([np.mean(y[x == xi]) for xi in xa])
    # check equal spacing
    dx = xa[1:]-xa[:-1]
    if np.min(dx) == np.max(dx):
        return xa, ya
    else:
        dx = np.min(dx)
        print('note: resampling of data with min dx = %.3e' % (dx))
        xp = np.arange(np.min(xa), np.max(xa), dx)
        yp = np.interp(xp, xa, ya)
        return (xp, yp)
    
def Gauss(x, x0, amp, sigma):
    "Gauss 1d"
    return amp*np.exp(-0.5*((x-x0)/sigma)**2)

def fit(x,y,model,p_start):
    """
    fits y=model(x) and returns parameters and errors [par,err].
    starting parameters are p_start.
    """
    try:
        par, cov = curve_fit(model, x, y, p0 = p_start, method='lm')
    except RuntimeError:
        print('warning: model fit failed! return no result.')
        return None
    err = np.sqrt(np.diag(cov))
    return [par.tolist(),err.tolist()]
    
def moving_average(y, num_avg, edges=True):
    # returns moving average of y averaged over num_avg elements
    # edges=True: returns len(y) data where num_avg-1 data at edges have fewer elements averaged.
    #             when num_avg is odd returns corresponding average value for each y-value
    #             when num_avg is even returns corresponding average values for in-between y-values
    #             if you need corresponding x-values, call moving_average on x-values as well.
    # with edges=False returns len(y)-num_avg+1 averages using exactly num_avg elements.
    if edges:
        # include edges
        num_edge = int(num_avg//2)
        avg = np.zeros(shape=(len(y)+num_avg-1,), dtype=y.dtype)
        avg[num_edge:num_edge+len(y)] = y
        avg = np.cumsum(avg)
        avg[num_avg:] -= avg[:-num_avg]
        num_n = np.zeros(shape=(len(y)+num_avg-1,), dtype=int)
        num_n[num_edge:num_edge+len(y)] = 1
        num_n = np.cumsum(num_n)
        num_n[num_avg:] -= num_n[:-num_avg]
        return avg[num_avg-1:] / num_n[num_avg-1:]
    else:
        # exclude edges
        avg = np.cumsum(y)
        avg[num_avg:] -= avg[:-num_avg]
        return avg[num_avg-1:] / num_avg

if False:
    # test moving average
    num_show = 10
    rng = np.random.default_rng()
    if True: # random number of elements and averages
        num     = rng.integers(100, 200)
        num_avg = rng.integers(10 , 35)
    else: # fixed number of elements and averages
        num     = 100
        num_avg = 4
    print('test moving average of', num, 'data over', num_avg, 'elements:')
    data = rng.normal(loc=0.0, scale=10.0, size=num)

    # manual calculation in a loop
    num_edge  = int(num_avg//2) # floor = left edge
    test      = [data[max(i-num_edge, 0):min(i-num_edge+num_avg, num)] for i in range(num)]
    num_n     = np.array([len(t) for t in test])
    test      = np.array([np.sum(t)/len(t) for t in test])
    count = np.count_nonzero(num_n == num_avg)
    if count != num-num_avg+1:
        print(count, 'elements with', num_avg, 'averages, but', num-num_avg+1, '/', len(test), 'expected!')
        exit()

    if num <= num_show:
        # data index which are averaged
        print('index :', [list(range(num))[max(i-num_edge, 0):min(i-num_edge+num_avg, num)] for i in range(num)])
        # number of elements for average
        print('length:', num_n)

    # exclude edges
    print('test num_average excluding edges ...')
    avg     = moving_average(data, num_avg, edges=False)
    if len(avg) != num-num_avg+1:
        print('length', len(avg), '!=', num-num_avg+1)
        exit()
    mask  = np.abs(avg - test[num_edge:num_edge+num-num_avg+1]) > 1e-12
    count = np.count_nonzero(mask)
    if count > 0:
        print(count, 'elements with different average!')
        for j,i in enumerate(np.where(mask)[0]):
            print('%i: %f != %f (error %.3e)' % (i, avg[i], test[num_edge+i], avg[i] - test[num_edge+i]))
            if j > 5:
                print('...')
                break
        exit()
    if num <= num_show:
        print('x-axis:', moving_average(np.arange(num, dtype=float), num_avg, edges=False))
    print('test num_average excluding edges ok')
    
    # include edges
    print('test num_average including edges ...')
    avg     = moving_average(data, num_avg, edges=True)
    if len(avg) != num:
        print('length', len(avg), '!=', num)
        exit()
    mask  = np.abs(avg - test) > 1e-12
    count = np.count_nonzero(mask)
    if count > 0:
        print(count, 'elements with different average!')
        for j,i in enumerate(np.where(mask)[0]):
            print('%i: %f != %f (error %.3e)' % (i, avg[i], test[i], avg[i] - test[i]))
            if j > 5:
                print('...')
                break
        print(test)
        print(avg)
        exit()
    if num <= num_show:
        print('x-axis:', moving_average(np.arange(num, dtype=float), num_avg, edges=True))
    print('test num_average including edges ok')

    print('test moving average ok.')
    exit()
    
def Arduino_get_error(time, times, ramp_up, laser_set=laser_set, laser_ref=laser_ref, ramp_set=ramp_set, num_avg=num_avg, ramp_freq=ramp_freq, max_error=max_error):
    # returns laser_error, ramp_error as calculated by Arduino in ticks
    # compares integer calculation with real-valued calculation and check that its within max_error.
    if num_avg is None:
        t_out  = time
        t_peak = [np.array(t, dtype=np.uint32) for t in times]
    else:
        t_out  =  moving_average(np.array(time), num_avg) + num_avg/2/ramp_freq # shifted since last num_avg samples are taken in time.
        t_peak = [moving_average(np.array(t   , dtype=np.uint32), num_avg).astype(np.uint32) for t in times]
    if ramp_up:
        laser_error_real = (t_peak[1]-t_peak[0]) - ((t_peak[2]-t_peak[0])*laser_set)/laser_ref
        laser_error      = t_peak[1].astype(np.int32)
        laser_error     -= t_peak[0]
        tmp              = t_peak[2].astype(np.int32)
        tmp             -= t_peak[0]
        tmp             *= laser_set # note: this might overflow!
        assert tmp.dtype == np.int32
        tmp              = (tmp / laser_ref).astype(np.int32)
        laser_error     -= tmp
    else:
        laser_error_real = (t_peak[5]-t_peak[4]) - ((t_peak[5]-t_peak[3])*laser_set)/laser_ref
        laser_error      = t_peak[5].astype(np.int32)
        laser_error     -= t_peak[4]
        tmp              = t_peak[5].astype(np.int32)
        tmp             -= t_peak[3]
        tmp             *= laser_set
        assert tmp.dtype == np.int32
        tmp              = (tmp / laser_ref).astype(np.int32)
        laser_error     -= tmp
    assert laser_error.dtype == np.int32
    
    error = np.abs(laser_error - laser_error_real)
    mask  = (error > max_error)
    count = np.count_nonzero(mask)
    if count > 0:
        print('error: %i/%i laser_error disagreements! max. error %.3e, ramp_up = %s' % (count, len(mask), np.max(error), ramp_up))
        for j,i in enumerate(np.where(mask)[0]):
            print('laser_error %i != %.3f, error %.3e, t_peaks:' % (laser_error[i], laser_error_real[i], error[i]), ','.join(['%6i'%t[i] for t in t_peak]))
            if j > 5:
                print('...')
                break
        exit()
    else:
        print('note: max. error laser_error calculation %.3e ticks = %.3e us, ramp_up = %s' % (np.max(error), np.max(error)/MCK, ramp_up))

    ramp_error_real = t_peak[0] - ramp_set*MCK
    ramp_error      = t_peak[0].astype(np.int32)
    ramp_error     -= np.array(ramp_set*MCK, dtype=np.int32)
    assert ramp_error.dtype == np.int32

    error = np.abs(ramp_error - ramp_error_real)
    mask  = (error > max_error)
    count = np.count_nonzero(mask)
    if count > 0:
        print('error: %i/%i ramp_error disagreements! max. error %.3e, ramp_up = %s' % (count, len(mask), np.max(error), ramp_up))
        for j,i in enumerate(np.where(mask)[0]):
            print('laser_error %i != %.3f, error %.3e, t_peaks:' % (ramp_error[i], ramp_error_real[i], error[i]), ','.join(['%6i'%t[i] for t in t_peak]))
            if j > 5:
                print('...')
                break
        exit()
    else: 
        print('note: max. error ramp_error calculation %.3e ticks = %.3e us, ramp_up = %s' % (np.max(error), np.max(error)/MCK, ramp_up))
    
    return t_out, laser_error, ramp_error
    
if __name__ == '__main__':

    peaks_time       = []
    peaks_tlabels    = []
    peaks_targs      = []
    peaks_time_2     = []
    peaks_tlabels_2  = []
    peaks_targs_2    = []
    pcorr_time       = []
    pcorr_tlabels    = []
    pcorr_targs      = []
    laser_time       = []
    laser_tlabels    = []
    laser_targs      = []
    laser_data       = []
    laser_dlabels    = []
    laser_dargs      = []
    laser_curves     = []
    laser_clabels    = []
    laser_cargs      = []
    laser_fft_curves = []
    laser_fft_labels = []
    laser_fft_args   = []
    ramp_time        = []
    ramp_tlabels     = []
    ramp_targs       = []
    ramp_data        = []
    ramp_dlabels     = []
    ramp_dargs       = []
    ramp_curves      = []
    ramp_clabels     = []
    ramp_cargs       = []
    ramp_fft_curves = []
    ramp_fft_labels = []
    ramp_fft_args   = []

    max_peaks = max([value[1]['peaks'] for value in fnames.values()])
    for label,(fname, config, dargs, cargs) in fnames.items():
        # load data from file
        lines = []
        with open(fname, 'r') as f:
            # read all lines and split into colums with ' '
            while True:
                values = [-1]*(max_peaks + 5)
                line = f.readline()
                line = line.rstrip()
                if len(line) == 0: break
                line = line.split(' ')
                if len(line) != config['peaks'] + 6:
                    if (len(line) >= 2) and line[2].startswith('#'):
                        print("line %i note: '%s' (skip)" % (len(lines), ' '.join(line[2:])))
                        continue
                    else:
                        print("line %i error: %i instead of %i columns!" % (len(lines), len(line), config['peaks'] + 6))
                        print(line)
                        exit()
                elif line[1] != '->':
                    print("line %i error: column 1 = '%s' instead of '%s'!" % (len(lines), line[1], '->'))
                    print(line)
                    exit()
                # get time in seconds from "16:05:51.472"
                time = line[0].split(':')
                if len(time) != 3:
                    print("line %i error: %i instead of %i columns!" % (len(lines), len(line), 9))
                    print(line)
                    exit()
                try:
                    time = int(time[0])*3600 + int(time[1])*60 + float(time[2])
                except ValueError:
                    print("line %i error: could not read time!" % (len(lines)))
                    print(line)
                    print(time)
                    exit()
                values[0] = time
                # get integers
                try:
                    for j in range(2, config['peaks'] + 6):
                        values[j-1 if j < config['peaks']+2 else j-config['peaks']-1+max_peaks] = int(line[j])
                except ValueError:
                    print("line %i error: could not read integer!" % (len(lines)))
                    print(line)
                    exit()
                lines.append(values)
        print(fname, ':', len(lines), 'lines loaded')
        
        # conversion ticks into MHz or us
        conv_x = 1/MCK*FSR/config['FSR'] if ERROR_MHZ else 1/MCK

        i = 0
        lines = np.transpose(lines)
        time = lines[i]
        i += 1
        times = lines[i:i+max_peaks]
        i += max_peaks
        laser_error = lines[i]*conv_x # MHz or us
        i += 1
        laser_out = conv_y['offset'] + lines[i]*conv_y['scaling'] # Volt
        i += 1
        ramp_error = lines[i]*conv_x # MHz or us
        i += 1
        ramp_out = conv_y['offset'] + lines[i]*conv_y['scaling'] # Volt

        t_start = np.min(time)
        time = time - t_start
        
        # peaks vs. time
        t_mean = [np.mean(t) for t in times]
        t_std  = [np.std (t) for t in times]
        t_off  = [np.max(t_std)*10, 10*MCK][1]
        peaks_time   += [[time, (t-t_mean[i] + t_off*i)/MCK] for i,t in enumerate(times)]
        peaks_tlabels+= [label+' t0-t%i'%(len(times)-1)] + [None]*(len(times)-1)
        peaks_targs  += [cargs for i in range(len(times))]
        
        if False:
            # plot laser output vs. time together with peaks
            V_mean = np.mean(laser_out)
            V_std  = np.std (laser_out)
            peaks_time_2    += [[time, laser_out]]
            peaks_tlabels_2 += [r'%s laser out %sV'%(label, str_error(V_mean, V_std))]
            peaks_targs_2   += [{'color':'Black', 'zorder':1}]
        
        if True:
            # plot ramp output vs. time together with peaks
            V_mean = np.mean(ramp_out)
            V_std  = np.std (ramp_out)
            peaks_time_2    += [[time, ramp_out]]
            peaks_tlabels_2 += [r'%s ramp out %sV'%(label, str_error(V_mean, V_std))]
            peaks_targs_2   += [{'color':'Gray', 'zorder':1}]

        # laser error vs. time
        laser_time   .append([time, laser_error])
        laser_tlabels.append(label)
        laser_targs  .append(cargs)
        ramp_time    .append([time, ramp_error])
        ramp_tlabels .append(label)
        ramp_targs   .append(cargs)
        
        if True and config['peaks'] == 6:
            # check asymmetry vs. ramp up/down
            # 1. do exact same calculation as in Arduino including average of peaks
            #    this shows a clear deviation between up/down error
            t, l_error_up  , r_error_up   = Arduino_get_error(time, times, ramp_up=True)
            t, l_error_down, r_error_down = Arduino_get_error(time, times, ramp_up=False)
            #print(time[0], laser_error[0], [t[0] for t in times], l_error_up[0], l_error_down[0], (l_error_up[0] + l_error_down[0])/2)
            laser_time   += [[t, l_error_up*conv_x], [t, l_error_down*conv_x]]
            laser_tlabels+= [label+' up', label+' down']
            laser_targs  += [{'color': 'Black'}, {'color': 'Gray'}]
            # 2. correlate FSR and peak position down vs. up
            #    x = ramp up, y = ramp down
            pcorr_time   += [[(times[2]-times[0])/(laser_ref*MCK), (times[5]-times[3])/(laser_ref*MCK)], 
                             [(times[1]-times[0])/(config['laser_set']*MCK), (times[5]-times[4])/(config['laser_set']*MCK)],
                            ]
            pcorr_tlabels+= [label+' FSR', label+' laser peak']
            pcorr_targs  += [{'color':'Red', 's':5}, {'color':'Blue', 's':5}] 

        # FFT of laser error. returns [freq, amp, phase]
        # average same times and resample missing times inbetween
        x,y = resample(time, laser_error)
        laser_fft = FFT(y, rate=1/(x[1]-x[0]))
        laser_fft_curves.append([laser_fft[0], laser_fft[1]])
        laser_fft_labels.append(label)
        laser_fft_args  .append(cargs)
        
        if False:
            # plot resampled signal
            laser_time   .append([x, y])
            laser_tlabels.append(label+' resampled')
            laser_targs  .append(cargs)
            
        # FFT of ramp error. returns [freq, amp, phase]
        x,y = resample(time, ramp_error)
        ramp_fft  = FFT(y, rate=1/(x[1]-x[0]))
        ramp_fft_curves .append([ramp_fft[0], ramp_fft[1]])
        ramp_fft_labels .append(label)
        ramp_fft_args   .append(cargs)
        
        if False:
            # plot resampled signal
            ramp_time   .append([x, y])
            ramp_tlabels.append(label+' resampled')
            ramp_targs  .append(cargs)

        if True:
            # histogram on laser error
            laser_stat = histogram(laser_error, num_bins=100)
            laser_data   .append(laser_stat)
            laser_dlabels.append(label+' data')
            laser_dargs  .append(dargs)
            x,y = laser_stat
            index = np.argmax(y)
            x0 = x[index]
            y0 = y[index]
            sigma = np.std(x-x0)
            tmp = fit(x, y, Gauss, p_start=[x0, y[index], sigma])
            if tmp is not None:
                laser_par, laser_err = tmp
                x_fit = np.linspace(np.min(x), np.max(x), 300)
                laser_fit = [x_fit, Gauss(x_fit, *laser_par)]
                print('laser error mu    = ', laser_par[0], '+/-', laser_err[0], ERROR_UNIT)
                print('laser error sigma = ', laser_par[2], '+/-', laser_err[2], ERROR_UNIT)
                laser_curves .append(laser_fit)
                laser_clabels.append(r'%s fit $\sigma$=%s%s' % (label, str_error(laser_par[2], laser_err[2]), ERROR_UNIT))
                laser_cargs  .append(cargs)

        if True:
            # histogram on ramp error
            ramp_stat = histogram(ramp_error, num_bins=100)
            x,y = ramp_stat
            index = np.argmax(y)
            x0 = x[index]
            y0 = y[index]
            sigma = np.std(x-x0)
            ramp_par, ramp_err = fit(x, y, Gauss, p_start=[x0, y[index], sigma])
            x_fit = np.linspace(np.min(x), np.max(x), 300)
            ramp_fit = [x_fit, Gauss(x_fit, *ramp_par)]
            print('ramp  error mu    = ', ramp_par[0], '+/-', ramp_err[0], ERROR_UNIT)
            print('ramp  error sigma = ', ramp_par[2], '+/-', ramp_err[2], ERROR_UNIT)

            ramp_data    .append(ramp_stat)
            ramp_dlabels .append(label+' data')
            ramp_dargs   .append(dargs)
            ramp_curves .append(ramp_fit)
            ramp_clabels.append(r'%s fit $\sigma$=%s%s' % (label, str_error(ramp_par[2], ramp_err[2]), ERROR_UNIT))
            ramp_cargs  .append(cargs)

    if True:
        # plot peak position vs. time
        ax = plot(
              title         = 'scanning transfer cavity lock: peak positions vs. time',
              data          = [], 
              data_labels   = [],
              data_args     = [],
              curves        = peaks_time,
              curves_labels = peaks_tlabels,
              curves_args   = peaks_targs,
              xlabel        = 'time (s)',
              ylabel        = r'peak position (%s) [arb. offset]' % ERROR_UNIT,
              pos           = [0.06, 0.08, 0.87, 0.86],
              legend_args   = {'bbox_to_anchor':(0.98, 0.97), 'loc':'upper right'} if len(peaks_time_2) == 0 else False,
              )
        if len(peaks_time_2) > 0:
            ax2 = plot(twin     = ax,
                  data          = [], 
                  data_labels   = [],
                  data_args     = [],
                  curves        = peaks_time_2,
                  curves_labels = peaks_tlabels_2,
                  curves_args   = peaks_targs_2,
                  ylabel        = r'Voltage',
                  legend_args   = {'bbox_to_anchor':(0.98, 0.97), 'loc':'upper right'}
                  )
        

    if True and max_peaks == 6:
        # plot correlation FSR and laser peak ramp down vs. ramp up = hysteresis
        ax = plot(
              title         = 'hysteresis = peak position ramp down vs. up',
              data          = pcorr_time, 
              data_labels   = pcorr_tlabels,
              data_args     = pcorr_targs,
              curves        = [[[0,2],[0,2]]],
              curves_labels = [None],
              curves_args   = [{'color':'Gray', 'linestyle':'dotted', 'zorder':1}],
              xlabel        = 'ramp up',
              ylabel        = 'ramp down',
              xrng          = [0.9, 1.1],
              yrng          = [0.9, 1.1],
              size          = (7,7),
              pos           = [0.12, 0.08, 0.84, 0.86],
              legend_args   = {'bbox_to_anchor':(0.03, 0.97), 'loc':'upper left'},
              )
        ax.axhline(y=1, color='Black', linestyle='dotted')
        ax.axvline(x=1, color='Black', linestyle='dotted')

    if True:
        # plot laser error vs. time
        ax = plot(
              title         = 'scanning transfer cavity lock: laser error vs. time',
              data          = [], 
              data_labels   = [],
              data_args     = [],
              curves        = laser_time,
              curves_labels = laser_tlabels,
              curves_args   = laser_targs,
              xlabel        = 'time (s)',
              ylabel        = r'error (%s)'%ERROR_UNIT,
              pos           = [0.065, 0.08, 0.88, 0.86],
              legend_args   = {'bbox_to_anchor':(0.98, 0.97), 'loc':'upper right'},
              )
        ax.axhline(y=+error_limit, color='Black', linestyle='dotted')
        ax.axhline(y=-error_limit, color='Black', linestyle='dotted')

    if True:
        # plot ramp error vs. time
        ax = plot(
              title         = 'scanning transfer cavity lock: ramp error vs. time',
              data          = [], 
              data_labels   = [],
              data_args     = [],
              curves        = ramp_time,
              curves_labels = ramp_tlabels,
              curves_args   = ramp_targs,
              xlabel        = 'time (s)',
              ylabel        = r'error (%s)'%ERROR_UNIT,
              pos           = [0.065, 0.08, 0.88, 0.86],
              legend_args   = {'bbox_to_anchor':(0.98, 0.97), 'loc':'upper right'},
              )
        ax.axhline(y=+error_limit, color='Black', linestyle='dotted')
        ax.axhline(y=-error_limit, color='Black', linestyle='dotted')

    if True:
        # plot FFT of laser error
        ax = plot(
              title         = 'scanning transfer cavity lock: FFT of laser error',
              data          = [], 
              data_labels   = [],
              data_args     = [],
              curves        = laser_fft_curves,
              curves_labels = laser_fft_labels,
              curves_args   = laser_fft_args,
              xlabel        = 'frequency (Hz)',
              ylabel        = r'amplitude (%s)'%ERROR_UNIT,
              xlog          = True,
              ylog          = True,
              pos           = [0.065, 0.08, 0.88, 0.86],
              legend_args   = {'bbox_to_anchor':(0.98, 0.97), 'loc':'upper right'},
              )

    if True:
        # plot FFT of ramp error
        ax = plot(
              title         = 'scanning transfer cavity lock: FFT of ramp error',
              data          = [], 
              data_labels   = [],
              data_args     = [],
              curves        = ramp_fft_curves,
              curves_labels = ramp_fft_labels,
              curves_args   = ramp_fft_args,
              xlabel        = 'frequency (Hz)',
              ylabel        = r'amplitude (%s)'%ERROR_UNIT,
              xlog          = True,
              ylog          = True,
              pos           = [0.065, 0.08, 0.88, 0.86],
              legend_args   = {'bbox_to_anchor':(0.98, 0.97), 'loc':'upper right'},
              )

    if False:
        # plot laser error histogram
        ax = plot(
              title         = 'scanning transfer cavity lock: laser error statistics',
              data          = laser_data, 
              data_labels   = laser_dlabels,
              data_args     = laser_dargs,
              curves        = laser_curves,
              curves_labels = laser_clabels,
              curves_args   = laser_cargs,
              xlabel        = r'laser error (%s)'%ERROR_UNIT,
              ylabel        = 'count',
              pos           = [0.065, 0.08, 0.88, 0.86],
              legend_args   = {'bbox_to_anchor':(0.98, 0.97), 'loc':'upper right'},
              )
        ax.axvline(x=+error_limit, color='Black', linestyle='dotted')
        ax.axvline(x=-error_limit, color='Black', linestyle='dotted')

    if True:
        # plot ramp error histogram
        ax = plot(
              title         = 'scanning transfer cavity lock: ramp error statistics',
              data          = ramp_data, 
              data_labels   = ramp_dlabels,
              data_args     = ramp_dargs,
              curves        = ramp_curves,
              curves_labels = ramp_clabels,
              curves_args   = ramp_cargs,
              xlabel        = r'ramp error (%s)'%ERROR_UNIT,
              ylabel        = 'count',
              pos           = [0.065, 0.08, 0.88, 0.86],
              legend_args   = {'bbox_to_anchor':(0.98, 0.97), 'loc':'upper right'},
              )
        ax.axvline(x=+error_limit, color='Black', linestyle='dotted')
        ax.axvline(x=-error_limit, color='Black', linestyle='dotted')

    plt.show()

    print('done')
