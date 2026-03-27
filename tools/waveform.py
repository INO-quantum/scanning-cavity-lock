#!/usr/local/bin/python3
# -*- coding: UTF-8 -*-

# create waveform

import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import math
from time import time, localtime, strftime
from numpy.polynomial import Polynomial
from scipy.optimize import curve_fit
from scipy.signal import resample
from multiprocessing import Process, Queue
from datetime import datetime

# for save button use current working directory
mpl.rcParams["savefig.directory"] = ""

#mpl.rcParams['font.family'] = 'sans-serif' # 'DejaVu Sans'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.linewidth'] = 2

#colors = cm.get_cmap('tab20')
colors = mpl.colormaps['tab20']

# select Arduino
# Vidor is used for simulating the cavity peaks. 
# Due is the actual SCTL controller and we need the triangle waveform for DMA
arduino = ['Vidor', 'Due'][1]

wave_type = {'Vidor':['sine', 'lorentz'][1], 'Due':'triangle'}[arduino]

samples = {'Vidor': 1024, 'Due': 2*2**12}[arduino]

#DAC bits resolution: Vidor (SAMD) has 10bits, Due (SAM3X) has 12bits
AO_BITS = {'Vidor': 10, 'Due':12}[arduino]
conv = 3.3/((2**AO_BITS)-1) # V/max value    

# buffer repetition frequency in Hz
fbuf = {'Vidor':92, 'Due':1e6/samples}[arduino] 

# file name, header and footer
file = {'Vidor':'samples.cpp', 'Due':'ramp.cpp'}[arduino]
header = {
    'Vidor': '// created by waveform.py %s %s\n\n#include "samples.h"\n\nuint16_t buffer[%i] = {\n',
    'Due'  : '// created by waveform.py %s %s\n\n#include "ramp.h"\n\nuint16_t ramp_buffer[%i] = {\n',
    }[arduino]
footer = '};\n'    

# for Due we must set the channel tag in data:
channel = 1
data_OR = {'Vidor':0, 'Due':[0x0000, 0x1000][channel]}[arduino]   

# data mask used for clipping
# Due must include channel tag in data
AO_MASK = {'Vidor':(2**AO_BITS)-1, 'Due':0x3fff}[arduino]

def plot(data, data_labels, data_args=None, 
         curves=[], curves_labels=[], curves_args=None, 
         title=None, xlabel=None, ylabel=None, 
         xrng=None, yrng=None,
         twin=None,
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
    legend_args  = if not None arguments for axis.legend
    fmt          = if not None format for coordinates. x,y for normal axis, and x,y0,y1 for twin axis.
    """
    if twin is None:
        fig = plt.figure(figsize=(14,7)) #size in inches (h,v)
        ax = fig.add_axes([0.055, 0.08, 0.89, 0.86])
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

def lorentz(x, x0, width):
    'returns normalized lorentzian peak at position x0 and give FWHM width'
    return (0.5*width)**2/((x-x0)**2+(0.5*width)**2)

if __name__ == '__main__':

    # we assume period = full buffer
    fs = samples*fbuf   # sampling frequency in Hz (only used for plotting x-axis, has no effect on output data)
    dt = 1/fs
    print('samplig frequency %.3fkHz, buffer samples %i, buffer period %.3fms, buffer frequency %.3fHz' % (fs/1000, samples, 1000*samples/fs, fs/samples))
    
    # create samples
    x = np.linspace(0, samples, samples)
        
    if wave_type == 'sine': # sine wave
        y = (1+np.sin(2*np.pi*x/samples))/2
    elif wave_type == 'lorentz': 
        # 2x3 lorentzian peaks with main + reg + main and simulating ramp up + asymmetric ramp down
        peaks  = [[0.9 ,  150, 50]] # main first [amplitude in V, position in us, width in us]
        peaks += [[0.6 , 1250, 50]] # regulated  [amplitude in V, position in us, width in us]
        peaks += [[0.75, 2150, 40]] # main sec.  [amplitude in V, position in us, width in us]
        y = np.zeros(shape=(samples,), dtype=float)
        for p in peaks:
            y += p[0]*lorentz(x=x/samples, x0=p[1]*1e-6*fbuf, width=p[2]*1e-6*fbuf)
        # mirror left to right half simulating ramp down of cavity
        mirror = int(2.5e-3*fbuf*samples)  # mirror axis, 0.5 would be half-half
        red    = 0.9                       # amplitude scaling due to loss of resolution
        y[mirror:] = red*resample(y[mirror-1::-1],samples-mirror)
    elif wave_type == 'triangle':
        # full resolution ramp up and ramp down
        num = int(samples//2)
        y = np.zeros(shape=(2*num,), dtype=float)
        y[:num] = 3.3*x[:num]/num
        y[num:] = 3.3*(num-1-x[:num])/num
        
    # scale 3.3V to uint16
    y = np.round(y/conv).astype(np.uint16) & AO_MASK
    
    with open(file, 'w') as f:
        f.write(header % (wave_type, strftime('%Y-%m-%d %H:%M:%S', localtime(time())), samples))
        for i,yi in enumerate(y[:-1]):
            f.write('    %4d, // %4i\n' % (yi    | data_OR, i       ))
        f.write(    '    %4d  // %4i\n' % (y[-1] | data_OR, len(y)-1))
        f.write(footer)
    print(file, samples, 'written\n')
    
    if True: 
        x_us = x*dt*1000
        xmin = np.min(x_us)
        xmax = np.max(x_us)
        ax = plot(title         = 'signal vs. time',
                  data          = [[x_us, y*conv]], 
                  data_labels   = ['data'],
                  data_args     = [{'color':'Blue', 's':2}],
                  curves        = [],
                  curves_labels = [], 
                  curves_args   = [],
                  xlabel        = 'time (ms)',
                  ylabel        = 'signal (V)',
                  legend_args   = {'bbox_to_anchor':(0.02, 0.97), 'loc':'upper left', 'frameon':True, 'fontsize':10, 'markerscale':1}
                  )
        threshold = np.array([200, 400])*3.3/2**12 # threshold set on Arudino Due with 12 bits ADC resolution
        ax.hlines(np.array(threshold), xmin=xmin, xmax=xmax, colors='Black', linestyles='dotted')
    

    # plot all figures
    plt.show()


