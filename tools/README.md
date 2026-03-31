# tools

This folder contains python tools to generate the ramp waveform, download data and analyse data and noise. In addition the Arduino Due commands on the `Serial Monitor` are summarized.


## tools overview

All tools use python 3 with Numpy and Scipy for data analysis and Matplotlib for plotting. 

`waveform.py` allows to generate the waveform data for the ramp of the Arduino Due. This is generally a triangle signal but other waveforms might be thinkable. For example, there is already a section for the `Arduino MKR Vidor 4000` which was used simulate cavity peaks for the Arduino Due. The code in general produces a `.cpp` file with the waveform samples. This file needs to be copied into the Arduino sketch folder and must be included with a separate header `.h` file. This way the waveform can be directly added and uploaded as static data buffer to the sketch. Note that we use direct-memory access (DMA) to output the samples from the buffer on the DAC1 analog output of the Arduino Due. In addition to the 12-bit (unsigned integer) representation of the analog value, bits \[13:12\] have to be set to the `RAMP_CHANNEL_TAG` of the analog output channel (2'b01 for DAC1) where the analog value should be output.

`statistics.py` reads one or several text file(s) containing the data output on `Serial Monitor` and collected with `p3r`, `p6r` or `Ls#r` or `Rs#r` commands. It allows to plot the peak positions, laser and ramp error, the Fourier transform of laser and ramp error, and laser and error histogram and Gaussian fit of mean value and standard deviation. For the 6-peak data it also plots the hysteresis data, i.e. the relative difference in peak positions between ramping up and ramping down. Note, that the 

> [!NOTE]
> At the moment the peak positions show random spikes which have not been understood so far. These spikes are common mode, i.e. all data is shifted which indicates an offset in the overall timing. The details have not been understood so far but it does not affect the performance of the lock.

`read_data.py` allows to download from the Arduino Due peak data and to save to file, or alternatively read data from file. To download data from the Arduino the option `USE_SERIAL` has to be set to `SERIAL_NATIVE` and uploaded on the Arduino. Then the Arduino has to be plugged into the `native USB port` - which is the Micro-USB port farther from the power plug. Ensure that the `Serial Monitor` is closed, otherwise the python script will not connect. In the script select the `PORT` where the Arduino appears on your computer, typically `/dev/ttyACM0` on linux or one of the `COM` ports on Windows (not tested however). When you select `None` then the script loads the data from the file given as `load_name`, otherwise the uploaded data is saved into the `csv` file `save_name`. When `save_new_name = True` then for each uploaded data a filename with incremented index is given. Select for `PEAK` the peak number 0 .. `NUM_PEAKS`-1 which should be uploaded. The peak data is plotted, see the peak image [here](../README.md#Performance). Note, that during the data uploading the Arduino does not perform the feedback on the laser and might miss some peaks. Use this feature for debugging and to get some data.

`plot_all_data.py` simply plots all peak data from all files in a given folder. The files must be loaded with `read_data.py`.

## Arduino commands

Here a summary of all commands which can be sent to Arudino Due using `Serial Monitor` (baud rate 115200). All commands must be terminated with a new line `\n`, i.e. hit `Enter` to send. The symbol `#` stands for an integer number (can be negative in some cases).

  * `p#`   = print peak number `#` (0 .. `NUM_PEAKS`-1) position in ticks = 1μs/86. If `# == NUM_PEAKS` print all peak positions. After peak position prints actual laser error, laser output, ramp error and ramp output.
  * `p#r`  = print peak number contiguously until `Enter` pressed. Note that this command skips samples.
  * `Rs?`  = print actual ramp setpoint in μs (i.e. position of first peak)
  * `Rs#`  = set ramp setpoint
  * `Rs#r` = same output as `p#r` command but change ramp setpoint by `#` after 1s and reset after 6s. Stop output with `Enter`.
  * `Ls?`  = print actual laser setpoint in μs (i.e. difference of position of 2nd to 1st peak) 
  * `Ls#`  = set laser setpoint μs
  * `Rs#r` = same output as `p#r` command but change laser setpoint by `#` after 1s and reset after 6s. Stop output with `Enter`.
  * `Rki?` = print actual ramp integral part
  * `Rki#` = set ramp integral part
  * `Rkp?` = print actual ramp proportional part
  * `Rkp#` = set ramp proportinal part
  * `Lki?` = print actual laser integral part
  * `Lki#` = set laser integral part
  * `Lkp?` = print actual laser proportional part
  * `Lkp#` = set laser proportinal part
  * `Ro?`  = print actual ramp output 0 .. 4095 = 0.55V .. 2.75V 
  * `Ro#`  = set ramp output 0 .. 4095 = 0.55V .. 2.75V
  * `Lo?`  = print actual laser feedback output 0 .. 4095 = 0.55V .. 2.75V 
  * `Lo#`  = set laser feedback output 0 .. 4095 = 0.55V .. 2.75V
  * `rst`  = reset laser and ramp output to default



