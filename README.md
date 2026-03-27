# scanning cavity lock

This is our implementation of the scanning cavity transfer lock with Arduino Due.

This work is based (loosely) on the [publication](https://doi.org/10.1063/1.5067266) `Microcontroller based scanning transfer cavity lock for long-term laser frequency stabilization` by S. Subhankar, A. Restelli, Y. Wang, S. L. Rolston, and J. V. Porto, Rev. Sci. Instrum. 90, 043115 (2019), [and on the github page](https://github.com/JQIamo/Scanning-Transfer-Cavity-Lock).

With respect to the previous publication we have added following features:
  * generation of the cavity triangular ramp and offset on the Arduino Due
  * average of measured peak positions before feedback
  * 9-point or 5-point Savitzky-Golay filter for derivative
  * linear fit of zero crossing with 2,4,6 or 8 points for better resilience against noise
  * 6 peak positions allow 2x feedback on laser per triangular ramp period
  * typical cavity ramp rate 170Hz (6ms) gives 340Hz (3ms) feedback time 
  * measured calculation time ca. 50μs per peak
  * measured 50kHz residual laser noise (1σ standard deviation, obtained from error signal, limited by analog resolution)
  * commands via `Serial Monitor` for feedback optimization, change of setpoint and measurement of noise and step function responds
  * fast download of measured peaks and derivatives via `Arduino Due native USB port`
  * python tools for ramp buffer generation, download of peak data and statistical analysis
  * thorough code review and optimization
  * work in progress: median filter for better noise resilience

Here an overview of the folder structure:

```
└── scanning-cavity-lock    main folder
    ├── SCTL_Arduino_Andi   Arduino Due project folder
    └── tools               python folder
```

More information will be added soon...
