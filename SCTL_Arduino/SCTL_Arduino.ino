// Arduino Due scanning cavity transfer lock by Andi, February 2026
// program on programming USB port (close to power plug)
// optionally:
// - set and read simple data on serial monitor. use to find parameters and for debugging
// - transmit ADC buffer content around peak and derivative 
//   requires to plug into native USB port (farther from power plug)
//   use python skript: read_data.py, connect to PORT = "/dev/ttyACM0" (or ACM1) 
//   for details see comments for 'header' and for CMD_ options below.

// include ramp_buffer
// note: this must include RAMP_CHANNEL_TAG for the given RAMP_OUT channel as defined below
#include "ramp.h"

#define LED                   LED_BUILTIN
#define AIN                   A0                  // analog input
#define LASER_OUT             DAC0                // analog output for feedback on laser
#define RAMP_OUT              DAC1                // analog output for cavity ramp and offset
#define TRIGGER_IN            2                   // external trigger input
#define TEST_OUT_0            7                   // ramp trigger
#define TEST_OUT_1            8                   // for debugging, see TEST_OUT_1_USE

#define NUM_PEAKS             6                   // number of peaks, 3 or 6

#define RAMP_FREQ             162                 // triangle output frequency in Hz, avoid integer multiple of 5,2 to reject 50Hz noise 

// on error LED goes on for the given time
#define LED_ERROR_TIME        0.5                         // seconds
#define LED_ERROR_COUNT       (LED_ERROR_TIME*RAMP_FREQ)  // loop counter

// number of averages to take for peak position
// for NUM_PEAKS == 6 must be nonzero, otherwise laser oscillates between 2 points around actual setpoint due to piezo hysteresis
#define NUM_AVG_P2            5                   // if > 0 take average of 2^NUM_AVG_P2 peak positions to calculate error signal
#if (NUM_AVG_P2 > 0)
#define NUM_AVG               (1<<NUM_AVG_P2)     // number of averages, must be power of 2
#else
#define NUM_AVG               0                   // do not average peak positions
#endif

// serial communication
// SERIAL_NONE   = do not send data (default)
// SERIAL_PROG   = serial communication via programming port. use for debugging and for adjusting of parameters
// SERIAL_NATIVE = native USB port (farther from power plug). use for sending of waveform data (on linux /dev/ttyACM0)
#define SERIAL_NONE           0
#define SERIAL_PROG           1
#define SERIAL_NATIVE         2
#define USE_SERIAL            SERIAL_PROG

// peak position relative to main peaks distance or absolute vs trigger
#define POS_RELATIVE          0
#define POS_ABSOLUTE          1
#define PEAK_POSITION         POS_RELATIVE

// general timing
#define MCK                   84                    // master clock in MHz
#define ADC_TIME_MU           1.5                   // ADC conversion time in mu. this is the measured value (cal_ticks>>cal_div)/MCK
#define ADC_TIME              (ADC_TIME_MU*MCK)     // ADC conversion time in ticks [126]

// startup time after reset without ramp or laser feedback. allows to manually adjust ramp offset.
#define STARTUP_SECONDS       5                           // seconds
#define STARTUP_COUNT         (STARTUP_SECONDS*RAMP_FREQ) // initial counter

// laser setpoint
#define LASER_SET_MU          1230.0                // microseconds between first and second peak (to be controlled)
#define LASER_REF_MU          2400.0                // microseconds between first and third peak (used as reference)
uint32_t laser_set            = LASER_SET_MU;       // laser setpoint in mu. we do not multiply by MCK to avoid 32bit overflow.
uint32_t laser_ref            = LASER_REF_MU;       // laser setpoint reference in mu

// ramp offset setpoint
// note: ensure RAMP_SET_MU > ramp_delay! see notes there.
#define RAMP_SET_MU           550.0                 // microseconds between trigger and first peak
uint32_t ramp_set             = RAMP_SET_MU*MCK;    // ramp offset setpoint in ticks

// PI gain for laser feedback
// minimum kp,ki = 1; 0 = disabled;
// maximum kp,ki = 2^31/(0.5*10^6/RAMP_FREQ*MCK); using max. error and 32bit signed integer; [8283]
// fastest integrator time = 2^12/(3.3V*RAMP_FREQ*MCK)*2^RAMP_DIV/maximum ki [0.1s/V/laser_error in us]
// slowest integrator time = 2^12/(3.3V*RAMP_FREQ*MCK)*2^RAMP_DIV/minimum ki [747s/V/laser_error in us]
#define LASER_DIV             13                    // overall gain divider, larger = slower. adjusted to keep kp,ki in good range.
int32_t laser_kp              = 1500;               // laser proportional gain
int32_t laser_ki              = 30;                 // laser integral gain

// PI gain for cavity feedback
// minimum kp,ki = 1; 0 = disabled;
// maximum kp,ki = 2^31/(0.5*10^6/RAMP_FREQ*MCK); using max. error and 32bit signed integer; [8283]
// fastest integrator time = 2^12/(3.3V*RAMP_FREQ*MCK)*2^RAMP_DIV/maximum ki [0.7 s/V/ramp_error in us]
// slowest integrator time = 2^12/(3.3V*RAMP_FREQ*MCK)*2^RAMP_DIV/minimum ki [6000s/V/ramp_error in us]
#define RAMP_DIV              16                    // overall gain divider, larger = slower. adjusted to keep kp,ki in good range.
int32_t ramp_kp               = 500;                // cavity proportional gain. not used.
int32_t ramp_ki               = 20;                 // cavity integral gain

// amplitude tuning word (ATW)
#define AIN_BITS              12                    // analog input resolution in bits
#define AOUT_BITS             12                    // analog output resolution in bits
#define AOUT_MASK             ((1<<AOUT_BITS)-1)    // analog output bit mask
#define AOUT_MIN              0                     // analog output minimum ATW
#define AOUT_MAX              AOUT_MASK             // analog output maximum ATW
#define AOUT_VREF             3.3                   // analog output voltage scaling value. the true output is 1/6VREF-5/6VREF

// peak low and high threshold
#define LOW_THRESHOLD_V       0.2                   // low threshold voltage
#define HIGH_THRESHOLD_V      0.4                   // high threshold voltage
const uint16_t low_threshold  = (LOW_THRESHOLD_V *AOUT_MAX)/AOUT_VREF;
const uint16_t high_threshold = (HIGH_THRESHOLD_V*AOUT_MAX)/AOUT_VREF;

#define DMA_BUFFERS           8                     // number of DMA buffers must be power of 2
#define BUF_MASK              (DMA_BUFFERS-1)       // mask number of buffers
#define BUF_SIZE_P2           7                     // buffer size as power of 2
const int BUF_SIZE            = (1<<BUF_SIZE_P2);   // DMA buffer size (must be power of 2)

// note: ensure that: (CAL_BUFS*BUF_SIZE*MCK<<AIN_BITS) < 32 otherwise calculation of t_peak might overflow!
#define CAL_BUFS_P2           4                     // number of buffers for calibration of ticks/ADC as power of 2
#define CAL_BUFS              (1<<CAL_BUFS_P2)      // number of buffers for calibration of ticks/ADC (must be power of 2)
const int cal_div             = CAL_BUFS_P2+BUF_SIZE_P2; // log2(CAL_BUFS + BUF_SIZE) = divider for calibration of ticks/ADC in power of 2 = shift right

// peak minimum width in ticks
// this is because we sometimes get 2x high/low threshold IRQs right after each other 
#define MIN_WIDTH_MU          50                    // us
const uint32_t min_width      = MIN_WIDTH_MU*MCK;   // ticks

// ignore peak within ramp_delay time from trigger
// notes:
// - RAMP_DELAY_MIN avoids that buffer start time is before trigger time, where TICK_DIFF (see below) would give wrong results.
// - the LP filters in the ramp amplifier cause that the ramp on cavity is delayed by ~200us vs. trigger
#define RAMP_DELAY_MIN        (BUF_SIZE*ADC_TIME_MU) // ~200us
#define RAMP_DELAY_MU         200                   // us
const uint32_t ramp_delay     = ((RAMP_DELAY_MU > RAMP_DELAY_MIN) ? RAMP_DELAY_MU : RAMP_DELAY_MIN)*MCK; // ticks

// cavity ramp amplitude in ATW 
#define RAMP_FRAC             0.33                  // fraction of output range to use for Vpp
#define RAMP_VPP_SAMPLES      AOUT_MAX*RAMP_FRAC
const uint16_t ramp_Vpp       = RAMP_VPP_SAMPLES;

// output offset, maximum and minimum ATW
// note: output is from 1/6VREF - 5/6VREF = 0.55V - 2.75V = 2.2Vpp
const uint16_t laser_offset   = (AOUT_MAX*1.5)/AOUT_VREF;       // output offset in ATW (1.5V)
const uint32_t laser_min      = AOUT_MIN           <<LASER_DIV; // laser_out min value in ATW<<RAMP_DIV
const uint32_t laser_max      = AOUT_MAX           <<LASER_DIV; // laser_out max value in ATW<<RAMP_DIV
const uint16_t ramp_offset    = ramp_Vpp;                       // output offset in ATW (Vpp around center)
const uint32_t ramp_min       = AOUT_MIN           <<RAMP_DIV;  // ramp_out min value in ATW<<RAMP_DIV
const uint32_t ramp_max       = (AOUT_MAX-ramp_Vpp)<<RAMP_DIV;  // ramp_out max value in ATW<<RAMP_DIV

// actual PI values
int32_t           laser_error = 0;                  // laser error signal
uint32_t          laser_i_out = laser_offset<<LASER_DIV;// laser integrator output
uint16_t          laser_out   = laser_offset;       // laser feedback output
int32_t           ramp_error  = 0;                  // cavity ramp error signal
uint32_t          ramp_i_out  = ramp_offset<<RAMP_DIV;// cavity ramp integrator output
volatile uint16_t ramp_out    = ramp_offset;        // cavity ramp low value
volatile bool     ramp_up     = true;               // cavity actual ramp direction

// SysTick count from TICK_RELOAD down to zero and then reloads SysTick
// time difference must take into account that ticks are reset every 200ms = 2^24-1/84e6
// note: when peak is within 200us of trigger, buffer start time is before trigger! 
//       this would erroneously looks like ticks were reloaded. 
//       RAMP_DELAY_MIN ensures that peaks within this time are ignored.
#define TICK_RELOAD                   0x00FFFFFF    // timer reload value (maximum possible value = 2^24-1)
#define TICK_DIFF(t_start,t_end)      ((t_start >= t_end) ? t_start - t_end : t_start + (TICK_RELOAD - t_end))
#define TICK_DIFF_I32(t_start,t_end)  ((int32_t)TICK_DIFF(t_start,t_end))

// step responds measurement timing for loop_count
#define STEP_TIME_0           (RAMP_FREQ*1)         // wait before positive step [1s]
#define STEP_TIME_1           (RAMP_FREQ*5)         // wait before negative step [5s]

#define IRQ_COUNT             0                     // number of threshold IRQs before changes state. does not work! must be 0.
#define COMPARE_LOW           (0 | (IRQ_COUNT << 12) | (1<<9)) // ADC_EMR register for low threshold
#define COMPARE_HIGH          (1 | (IRQ_COUNT << 12) | (1<<9)) // ADC_EMR register for high threshold

#define PEAK_NONE             -1                    // no peak. must be -1.

// derivative filter function
#define SG5                   0                     // 5-point Savitzky-Golay filter
#define SG9                   1                     // 9-point Savitzky-Golay filter
#define FILTER                SG9
// number of edge indices for derivative
#if FILTER == SG5
#define NUM_FILTER_POINTS     5
#define NUM_EDGE              2
#elif FILTER == SG9
#define NUM_FILTER_POINTS     9
#define NUM_EDGE              4
#endif

// zero crossing linear fit points
// for simplicity must be multiple of 2, maximum NUM_EDGE*2
// for SG5: 2 or 4, default 4
// for SG9: 2,4,6 or 8, default 4
#if FILTER == SG5
#define NUM_FIT_POINTS        4
#elif FILTER == SG9
#define NUM_FIT_POINTS        4
#endif
#define NUM_FIT_HALF          (NUM_FIT_POINTS>>1)

// possible states
#define STATE_RESET           0         // reset done
#define STATE_LOW             1         // low threshold reached
#define STATE_HIGH            2         // high threshold reached

// laser DACC channel tag
#if LASER_OUT == DAC0
#define LASER_CHANNEL_TAG     (0<<12)
#elif LASER_OUT == DAC1
#define LASER_CHANNEL_TAG     (1<<12)
#endif

// ramp DACC channel tag
#if RAMP_OUT == DAC0
#define RAMP_CHANNEL_TAG      (0<<12)
#elif RAMP_OUT == DAC1
#define RAMP_CHANNEL_TAG      (1<<12)
#endif

#define TO1_NONE            0
#define TO1_CALC_TIME       1
#define TO1_THRESHOLD       2
#define TEST_OUT_1_USE      TO1_NONE

// cast x to int32_t
#define INT32(x)              ((int32_t)(x))

#if USE_SERIAL == SERIAL_NATIVE

// data is sent to SerialUSB on recipt of one of these commands:
// note: sending of a lot of data might cause the microcontroller to miss peaks.
//       during calculation of peak and sending of data TEST_OUT_1 is high
// - 'P#\n' = full peak #=0-NUM_PEAKS-1 without derivative
// - 'p#\n' = half peak #=0-NUM_PEAKS-1 without derivative
// - 'D#\n' = full peak #=0-NUM_PEAKS-1 with derivative
// - 'd#\n' = half peak #=0-NUM_PEAKS-1 with derivative
// the microcontroller responds with:
// - header with information about data
// - samples * uin16_t ADC data 
// - samples * int32_t derivative when deriv==1
// - 1 * uint16_t with number of samples. 0 if buffer was overwritten during sending of data.

// note: header is aligned on 32bit boundaries by compiler, i.e. 0s are padded between non-aligned items.
struct header {
  uint32_t  flags;        // {full[29],deriv[28],cal_div[27:20],NUM_EDGE[19:16],tot_buf[15:12],n_low[11:8],n_high[7:4],peak[3:0]}
  uint32_t  cal_ticks;    // ticks for tick/ADV calibration (cal_ticks>>cal_div = 126.000 = 1.500us = ADC_TIME_MU)
  uint16_t  samples;      // number of samples
  uint16_t  i_high;       // high threshold buffer count
  uint16_t  i_low;        // low threshold buffer count
  uint16_t  i_peak;       // sample index of found peak
  uint32_t  t_trig;       // trigger ticks
  uint32_t  t_high;       // high threshold ticks
  uint32_t  t_low;        // low threshold ticks
  uint32_t  t_peak;       // peak time from t_trig in ticks
  int32_t   laser_error;  // laser error signal
  int32_t   ramp_error;   // ramp offset error signal
  uint16_t  laser_out;    // laser feedback output 
  uint16_t  ramp_low;     // ramp low value
  uint16_t  ramp_high;    // ramp high value
  uint16_t  dummy;        // dummy for 32-bit alignment.
  uint32_t  t_buf[DMA_BUFFERS]; // buffer change ticks
};
struct header info;

#endif

volatile uint16_t buf[DMA_BUFFERS][BUF_SIZE];         // DMA buffers
volatile uint8_t  n_buf               = 0;            // actual DMA buffer index
volatile uint8_t  state               = STATE_RESET;  // state
volatile int8_t   peak                = PEAK_NONE;    // peak count
volatile uint8_t  n_high              = 0;            // buffer index high threshold
volatile uint8_t  n_low               = 0;            // buffer index low threshold
volatile uint16_t i_high              = 0;            // buffer counter high threshold
volatile uint16_t i_low               = 0;            // buffer counter low threshold
volatile uint32_t t_trig              = 0;            // tick count trigger
volatile uint32_t t_high              = 0;            // tick count high threshold
volatile uint32_t t_low               = 0;            // tick count low threshold
volatile uint32_t t_peak[NUM_PEAKS]   = {0};          // ticks from t_trig for each peak
volatile uint32_t t_buf[DMA_BUFFERS]  = {0};          // buffer change tick counter used for calibration ticks/ADC
volatile uint32_t cal_count           = 0;            // buffer counter for calibration ticks/ADC
volatile uint32_t cal_tlast           = 0;            // ticks for first buffer for calibration ticks/ADC
volatile uint32_t cal_ticks           = 0;            // ticks for CAL_BUFS buffers

#if NUM_AVG > 0
uint32_t t_peak_avg[NUM_PEAKS][NUM_AVG];
uint32_t t_peak_sum[NUM_PEAKS] = {0};
uint8_t   avg_ptr = 0;
#endif

// forward declaration of external trigger IRQ service routine
void trigger_ISR();

#if USE_SERIAL != SERIAL_NONE
// read and interpret commands from Serial/SerialUSB
// commands must be terminated with '\n'
// returns command index, on error return < 0.
// where command has '#' a number is returned as value
#define CMD_END                   '\n'
#define CMD_INT                   '#'
#define CMD_NUM                   28
#define CMD_NONE                  -1
#define CMD_PEAK_HALF             0
#define CMD_PEAK_HALF_REP         1
#define CMD_PEAK_FULL             2
#define CMD_PEAK_FULL_REP         3
#define CMD_PEAK_DERIV_HALF       4
#define CMD_PEAK_DERIV_HALF_REP   5
#define CMD_PEAK_DERIV_FULL       6
#define CMD_PEAK_DERIV_FULL_REP   7
#define CMD_GET_LASER_SETPOINT    8
#define CMD_SET_LASER_SETPOINT    9
#define CMD_GET_RAMP_SETPOINT     10
#define CMD_SET_RAMP_SETPOINT     11
#define CMD_STEP_LASER            12
#define CMD_STEP_RAMP             13
#define CMD_GET_LASER_KP          14
#define CMD_SET_LASER_KP          15
#define CMD_GET_RAMP_KP           16
#define CMD_SET_RAMP_KP           17
#define CMD_GET_LASER_KI          18
#define CMD_SET_LASER_KI          19
#define CMD_GET_RAMP_KI           20
#define CMD_SET_RAMP_KI           21
#define CMD_GET_LASER_OUT         22
#define CMD_SET_LASER_OUT         23
#define CMD_GET_RAMP_OUT          24
#define CMD_SET_RAMP_OUT          25
#define CMD_RST                   26
#define CMD_REP_OFF               27
const char *  commands[CMD_NUM]   = {"p#\n","p#r\n","P#\n","P#r\n",
                                     "d#\n","d#r\n","D#\n","D#r\n",
                                     "Ls?\n","Ls#\n","Rs?\n","Rs#\n",
                                     "Ls#r\n","Rs#r\n",
                                     "Lkp?\n","Lkp#\n","Rkp?\n","Rkp#\n",
                                     "Lki?\n","Lki#\n","Rki?\n","Rki#\n",
                                     "Lo?\n","Lo#\n","Ro?\n","Ro#\n",
                                     "rst\n","\n"};
uint8_t commands_len[CMD_NUM] = {0};

#if USE_SERIAL == 1
#define SERIAL_TYPE   Serial
#elif USE_SERIAL == 2
#define SERIAL_TYPE   SerialUSB
#endif

int parse_command(int32_t *value) {
  // read command until '\n'
  bool term = false, ok = false;
  int32_t val = 0, sign = 1;
  int i = 0, int_count = 0, cmd = 0, tmp;
  *value = 0;
  while (true) {
    ok = false;
    tmp = SERIAL_TYPE.read();
#if USE_SERIAL == SERIAL_PROG
    if ((tmp >= 32) && (tmp <= 127)) Serial.write(tmp);
    else if (tmp != -1){
      Serial.print("\\0x");
      Serial.println(tmp, HEX);
    }
#endif
    if (tmp == -1) {
      if (term) break; // command finished
      else {
        // we get sometimes '-1' = "no data" between command characters which we ignore.
        // this could however hang the system forever.
        // according to docs Serial.setTimeout should not affect Serial.read().
//#if USE_SERIAL == SERIAL_PROG
//        Serial.print("\\0xff"); // print for debugging - this gives also a waiting time.
//#endif
        continue;
      }
    }
    else if (term) {
      // unexpected character after '\n'
      cmd = -3;
      break;
    }
    else if (commands[cmd][i] == tmp) {
      // matching character
      if (tmp == CMD_END) term = true; // expect -1 in next loop
      ++i;
      // finish reading number   
      if (int_count > 0) {
        *value = sign*val;
        int_count = 0;
        sign = 1;
      }
      continue;
    }
    else if ( (commands[cmd][i] == CMD_INT) || (int_count > 0) ) {
      // read integer
      if (commands[cmd][i] == CMD_INT) {
        val = 0;
        ++i; // stop reading integer with next character
      }
      if ((tmp >= '0') && (tmp <= '9')) {
        val = val*10 + (tmp - '0');
        ++int_count;
        continue; // continue reading integer
      }
      else if ((int_count == 0) && (tmp == '-')) {
        sign = -1;
        ++int_count;
        continue; // continue reading integer
      }
      else {
        // not a number: we come here only when same commands deviate after number, e.g. "p3\n" and "p3r\n".
        //               first we assume number is finished and then we check if we find a matching command.
        *value = sign*val;
        int_count = 0;
        sign = 1;
      }
    }
    // neither matching command character nor reading number:
    // try to find new command starting from last command. 
    // this allows to have commands starting with the same characters.
    // e.g. cmd = "print\n" with tmp='l' and i=5 would not match with '\n', but new command could be "println\n"
    // note: we ensure that new command "clearln\n" does not match.
    int old_cmd = cmd, j;
    for (++cmd; cmd < CMD_NUM; ++cmd) {
      if (i < commands_len[cmd]) {
        if (commands[cmd][i] == tmp) {
          // same character i: ensure equal starting characters
          for (j = 0; j < i; ++j) {
            if (commands[cmd][j] != commands[old_cmd][j]) break; // different character
          }
          if (j == i) {
            if (tmp == CMD_END) term = true; // expect -1 in next loop
            ++i; // new command with same starting characters found
            break;
          }
        }
        else if ( (((tmp >= '0') && (tmp <= '9')) || (tmp == '-')) && (commands[cmd][i] == CMD_INT) ) {
          ++i; // new command with reading integer found
          int_count = 1;
          if (tmp == '-') {
            sign = -1; 
            val = 0;
          }
          else {
            val = tmp - '0';
          }
          break;
        }
      }
    }
    if (cmd >= CMD_NUM) {
      // error: command not found
      cmd = -1;
      break;
    }
  }
  return cmd;
}
#endif

void setup(){
  int i,j;
#if USE_SERIAL == SERIAL_PROG
  Serial.begin(115200);
  //while(!Serial); // do not wait until Serial Monitor is connected
  Serial.println("init ...");
  //Serial.setTimeout(1000); // timeout in ms. default 1000. note: timeout might be affected by our change of SysTick
#elif USE_SERIAL == SERIAL_NATIVE
  SerialUSB.begin(0);
  //while(!SerialUSB); // do not wait until native USB is connected
#endif

  pinMode(LED,  OUTPUT);
  pinMode(AIN,  INPUT);
  pinMode(LASER_OUT, OUTPUT);
  pinMode(RAMP_OUT, OUTPUT);
  pinMode(TEST_OUT_0, OUTPUT);
  pinMode(TEST_OUT_1, OUTPUT);
  
  digitalWrite(LED, HIGH);
  digitalWrite(TEST_OUT_0, LOW);
  digitalWrite(TEST_OUT_1, LOW);
  
  analogReadResolution(AIN_BITS);
  analogWriteResolution(AOUT_BITS);
  analogWrite(LASER_OUT, laser_out);
  analogWrite(RAMP_OUT, ramp_out);

  pmc_enable_periph_clk(ID_ADC);
  //PMC->PMC_PCER1 |= PMC_PCER1_PID37; 
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  //adc_init(ADC, F_CPU, 100000, 0);
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);

  // ADC settings
  ADC->ADC_MR |=0x80; // free running
  //ADC->ADC_MR |=0x0001; // ADC mode register: use external trigger pin PA11 = D18/TX1

  // ADC channel selection
  ADC->ADC_CHER=0x0080; // ADC channel enable register: bit 7 = channel 7 = q0
  ADC->ADC_CHDR=0xff7f; // ADC channel disable register: disable all other channels 1-15

  // setup comparison window
  adc_set_comparison_channel(ADC, ADC_CHANNEL_7);
  adc_set_comparison_window(ADC, low_threshold, high_threshold);
  ADC->ADC_EMR = COMPARE_HIGH;

  // Peripheral DMA Controller (PDC)
  // description:
  // - each analog-to-digital conversion is saved into RPR and RCR is decremented.
  // - when RCR == 0 ENDRX IRQ is issued.
  // - and when RNCR is not 0 RNPR is used as the new buffer in RPR and RCR is reloaded with RNCR.
  // - on each ENDRX IRQ the ADC_Handler is called and reprograms RNPR and RNCR to next buffer (buf[2], buf[3], buf[0], etc.)
  ADC->ADC_RPR  = (uint32_t)buf[0];   // Receive Pointer Register = buffer 0
  ADC->ADC_RCR  = BUF_SIZE;           // Receive Counter Register = size of buffer 0
  ADC->ADC_RNPR = (uint32_t)buf[1];   // Receive Next Pointer Register = buffer 1
  ADC->ADC_RNCR = BUF_SIZE;           // Receive Next Counter Register = size of buffer 1
  ADC->ADC_PTCR = 1;                  // Transfer Control Register, bit 0 = RX enable

  // ADC interrupts (ADC_IER_ENDRX|ADC_IER_RXBUFF|ADC_IER_COMPE)
  ADC->ADC_IER = ADC_IER_ENDRX|ADC_IER_COMPE;     // ADC Interrupt Enable Register, bit 27 = ENDRX (End of Receive Buffer Interrupt Enable)
  ADC->ADC_IDR=~(ADC_IER_ENDRX|ADC_IER_COMPE);    // ADC Interrupt Disable Register, disable all other interrupts
  NVIC_ClearPendingIRQ(ADC_IRQn);
  //NVIC_SetPriority(ADC_IRQn, 0);
  NVIC_EnableIRQ(ADC_IRQn); // enable ADC interrupt

  /* test real-time timer. works for repeated IRQs but not for single IRQ?
  RTT->RTT_MR = RTT_MR_RTTRST
                | RTT_MR_RTPRES (0x8000) // T = 1 second
                | RTT_MR_ALMIEN;
  RTT->RTT_AR = RTT_AR_ALMV(0); //Defines the alarm value (ALMV+1) compared with the Real-time Timer
  NVIC_EnableIRQ(RTT_IRQn);  
  */

  // external trigger interrupt (not needed since we use DAC1 to generate ramp internally)
  //pinMode(TRIGGER_IN, INPUT);
  //attachInterrupt(digitalPinToInterrupt(TRIGGER_IN), trigger_ISR, RISING);
  //ADC->ADC_CR=2;                    // ADC control register: bit 0 = reset, bit 1 = start conversion

  // activate 24bit system timer with source = MCK = 84MHz
  // note: Arduino library uses this with LOAD = 84000 to get 1ms for millis() and delay()
  //       since we change this these functions will not work as expected!
  SysTick->CTRL = 0;
  SysTick->LOAD = TICK_RELOAD;
  SysTick->VAL  = 0;
  SysTick->CTRL = 0x5; // bit 0 = enable, bit 1 = IRQ, bit 2 = source (0=MCK/8, 1=MCK)

  // timer TC2 for controlled DACC (ramp) frequency
  pmc_enable_periph_clk (TC_INTERFACE_ID + 0*3 + 2);
  //PMC->PMC_PCER0 |= PMC_PCER0_PID29;                      // enable peripheral clock for TC0
  TC0->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK2  // MCK/8, clk on rising edge
                              | TC_CMR_WAVE               // Waveform mode
                              | TC_CMR_WAVSEL_UP_RC       // UP mode with automatic trigger on RC Compare
                              | TC_CMR_ACPA_CLEAR         // Clear TIOA on RA compare match
                              | TC_CMR_ACPC_SET;          // Set TIOA on RC compare match

  TC0->TC_CHANNEL[2].TC_RC = 84e6/8/(RAMP_BUFLEN*RAMP_FRAC)/RAMP_FREQ; // frequency = (MCK/8)/TC_RC = 328.125kHz; /(RAMP_FRAC*8192) = 120Hz for RAMP_FRAC=3 buffer 
  TC0->TC_CHANNEL[2].TC_RA = 15;                         // duty cycle 50%
  TC0->TC_CHANNEL[2].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG; // TC2 enable

  //TC0->TC_CHANNEL[2].TC_IER |= TC_IER_CPCS;             // enable TC2 IRQ
  //NVIC_EnableIRQ(TC2_IRQn);                             // enable TC2 IRQ
  
  //DACC Configuration
  pmc_enable_periph_clk (DACC_INTERFACE_ID); // start clocking DAC
  //PMC->PMC_PCER1 |= PMC_PCER1_PID38;
  dacc_reset(DACC);
  DACC->DACC_MR =   DACC_MR_TRGEN_EN                  // external trigger
                  | DACC_MR_TRGSEL(0b011)             // TIO Output of the Timer Counter Channel 0
                  | DACC_MR_WORD_HALF                 // half word = single channel
                  | DACC_MR_TAG_EN                    // enable channel selection tag
                  | DACC_MR_REFRESH (1)               // refresh period 1024/DACC Clock
                  | DACC_MR_STARTUP_8;                // startup time after sleep = 512 periods of DACClock
  DACC->DACC_CHER = DACC_CHER_CH0 | DACC_CHER_CH1;    // enable both channels

  DACC->DACC_TPR  = (uint32_t)(ramp_buffer+ramp_out); // buffer
  DACC->DACC_TCR  = RAMP_VPP_SAMPLES;                 // buffer length
  DACC->DACC_TNPR = (uint32_t)(ramp_buffer+RAMP_BUFLEN-ramp_out-ramp_Vpp); // next buffer
  DACC->DACC_TNCR = RAMP_VPP_SAMPLES;                 // next buffer length
  DACC->DACC_PTCR = DACC_PTCR_TXTEN;                  // Transfer Control Register, bit 1 = TX enable

  // enable buffer reload IRQ. in DACC_Handler we set next offset.
  DACC->DACC_IDR = ~DACC_IDR_ENDTX;
  DACC->DACC_IER = DACC_IER_ENDTX;
  //NVIC_SetPriority(DACC_IRQn, 0xFF);
  NVIC_EnableIRQ(DACC_IRQn);
  
  //DACC Interrupt NVIC Enable
  //NVIC_DisableIRQ(DACC_IRQn);
  //NVIC_ClearPendingIRQ(DACC_IRQn);
  //NVIC_SetPriority(DACC_IRQn, 0);
  //NVIC_EnableIRQ(DACC_IRQn);

#if NUM_AVG > 0
  // init averaging vectors
  for(i = 0; i < NUM_PEAKS; ++i) {
    t_peak_sum[i] = 0;
    for (j = 0; j < NUM_AVG; ++j) t_peak_avg[i][j] = 0;
  }
#endif

#if USE_SERIAL != SERIAL_NONE
  // init command length
  for(i = 0; i < CMD_NUM; ++i) {
    for(j = 0; commands[i][j] != '\0'; ++j);
    commands_len[i] = j;
  }
#endif
  
#if USE_SERIAL == SERIAL_PROG
  Serial.println("ok");
#endif
}
/*
void RTT_Handler () {
  // real time clock interrupt. works for continuous IRQ's but did not manage to get a single IRQ?
  digitalWrite(TEST_OUT_1, HIGH);
  RTT->RTT_MR &= ~RTT_MR_ALMIEN;
  RTT->RTT_SR;
  RTT->RTT_AR = RTT_AR_ALMV(0);
  RTT->RTT_MR |= RTT_MR_ALMIEN;
  RTT->RTT_MR |= RTT_MR_RTTRST;
  //flag = 0;
  digitalWrite(TEST_OUT_1, LOW);
}
*/

/* replaced by DACC_Handler
void trigger_ISR() {
  // external pin IRQ: reset peak count and start find peaks
  state = STATE_LOW;
  peak  = PEAK_NONE;

  // trigger time in system ticks (84MHz)
  t_trig = t_high = t_low = SysTick->VAL;
  
  //// test alarm at ~last peak value 
  //RTT->RTT_MR &= ~RTT_MR_ALMIEN;
  //RTT->RTT_AR = RTT_AR_ALMV(t_peak[0]>>11); //Defines the alarm value (ALMV+1) compared with the Real-time Timer
  //RTT->RTT_MR |= RTT_MR_ALMIEN;
  //RTT->RTT_MR |= RTT_MR_ALMIEN;
  //RTT->RTT_MR |= RTT_MR_RTTRST;
  ////NVIC_EnableIRQ(RTT_IRQn);  
  
}*/

void ADC_Handler() {                          // ENDRX|COMPE IRQ
  volatile int reg = ADC->ADC_ISR;            // ADC Interrupt Status Register. reset when read.
  uint32_t t_now;
  uint32_t dt;
  uint8_t next_buf;
  //NVIC_DisableIRQ(ADC_IRQn);
  //NVIC_DisableIRQ(DACC_IRQn);
  if (reg & ADC_IER_ENDRX) {
    n_buf = (n_buf + 1) & BUF_MASK;           // actual buffer used
    t_buf[n_buf] = SysTick->VAL;
    if (++cal_count == CAL_BUFS) {
      // calibration of ticks / ADC
      cal_ticks = TICK_DIFF(cal_tlast, t_buf[n_buf]);
      cal_tlast = t_buf[n_buf];
      cal_count = 0;
    }
    next_buf = (n_buf + 1) & BUF_MASK;  // next buffer to be loaded
    ADC->ADC_RNPR = (uint32_t)buf[next_buf];    // Receive Next Pointer Register = buffer n_buf + 2
    ADC->ADC_RNCR = BUF_SIZE;                   // Receive Next Counter Register = size of buffer n_buf + 2
  }
  if ((state == STATE_LOW) && (reg & ADC_IER_COMPE)) {
    // high threshold reached
    t_now = SysTick->VAL;
    dt = TICK_DIFF(t_low, t_now);
    if ( ((peak == PEAK_NONE) && (dt > ramp_delay)) ||
         ((peak != PEAK_NONE) && (dt > min_width )) ) {
      i_high = ADC->ADC_RCR;
      //n_high = ADC->ADC_RPR;
      n_high = ((i_high == BUF_SIZE) && (!(reg & ADC_IER_ENDRX))) ? ((n_buf + 1) & BUF_MASK) : n_buf; 
      t_high = t_now;
      ADC->ADC_EMR = COMPARE_LOW;
      state  = STATE_HIGH;
#if TEST_OUT_1_USE == TO1_THRESHOLD 
      digitalWrite(TEST_OUT_1, HIGH);
#endif
    }
  }
  else if ((state == STATE_HIGH) && (reg & ADC_IER_COMPE)) {
    // low threshold reached
    t_now = SysTick->VAL;
    dt = TICK_DIFF(t_high, t_now);
    if (dt > min_width) {
      i_low = ADC->ADC_RCR;
      //n_low = ADC->ADC_RPR;
      n_low = ((i_low == BUF_SIZE) && (!(reg & ADC_IER_ENDRX))) ? ((n_buf + 1) & BUF_MASK) : n_buf; 
      t_low = t_now;
      ADC->ADC_EMR = COMPARE_HIGH; 
      state = STATE_LOW;
      ++peak; // checked in loop() that peak is finished
#if TEST_OUT_1_USE == TO1_THRESHOLD 
      digitalWrite(TEST_OUT_1, LOW);
#endif
    }
  }
  //NVIC_ClearPendingIRQ(ADC_IRQn);
  //NVIC_EnableIRQ(ADC_IRQn);
  //NVIC_EnableIRQ(DACC_IRQn);
}

void DACC_Handler() {
  // set next buffer to new offset (ramp_out) and window
  volatile int reg = DACC->DACC_ISR; // we must read DACC_ISR to reset IRQ
  if ( reg & DACC_ISR_ENDTX ) { 
    if (ramp_up) {
      // ramping up is finished, i.e. we are ramping down and have to setup next ramping up
      DACC->DACC_TNPR = (uint32_t)(ramp_buffer+ramp_out); // next buffer start index
      DACC->DACC_TNCR = RAMP_VPP_SAMPLES;                 // next buffer length we do not change but we have to set
      ramp_up = false;
      digitalWrite(TEST_OUT_0, LOW);
    }
    else {
      // ramping down is finished, i.e. we are ramping up and have to setup next ramping down

      // reset peak count and start finding peaks. this replaces the external pin interrupt
      state = STATE_LOW;
      peak  = PEAK_NONE;

      // set comparator to high threshold and clear eventual low-threshold pending IRQs
      NVIC_DisableIRQ(DACC_IRQn);
      ADC->ADC_EMR = COMPARE_HIGH;
      NVIC_ClearPendingIRQ(DACC_IRQn);
      NVIC_EnableIRQ(DACC_IRQn);
      
      // trigger time in system ticks (84MHz)
      t_trig = t_high = t_low = SysTick->VAL;

      DACC->DACC_TNPR = (uint32_t)(ramp_buffer+RAMP_BUFLEN-ramp_out-ramp_Vpp); // next buffer start index
      DACC->DACC_TNCR = RAMP_VPP_SAMPLES;                       // next buffer length we do not change but we have to set
      
      ramp_up = true;
      digitalWrite(TEST_OUT_0, HIGH);
    }
  }
}

/* executed on systemtick interrupt, i.e. when it reloads
int sysTickHook(void) {
  static int state = 0;
  state = state ? 0 : 1;
  digitalWrite(TEST_OUT_1, state);
  //return false;
}*/

#define FLAGS_NONE    0x0000
#define FLAGS_STARTUP 0x0001
#define FLAGS_REPEAT  0x0002
#define FLAGS_FULL    0x0004
#define FLAGS_DERIV   0x0008
#define FLAGS_STEP_0  0x0010
#define FLAGS_STEP_1  0x0020
#define FLAGS_STEP    (FLAGS_STEP_0 | FLAGS_STEP_1)
#define FLAGS_LASER   0x0040
#define FLAGS_RAMP    0x0080
#define FLAGS_ERROR   0x0100
void loop() {
  int i, k, i_pf, i_q;
  static int16_t old_error = 0;
  int16_t error = 0;
  uint32_t _t_trig, _t_high;
  uint8_t n, _n_high, _n_low;
  uint16_t j, j_max, _i_high, _i_low;
  uint16_t samples, index;
#if NUM_FIT_POINTS == 2  
  int32_t q0, q1;
#elif NUM_FIT_POINTS == 4
  int32_t q0, q1, q2, q3;
#elif NUM_FIT_POINTS == 6
  int32_t q0, q1, q2, q3, q4, q5;
#elif NUM_FIT_POINTS == 8
  int32_t q0, q1, q2, q3, q4, q5, q6, q7;
#endif  
  int32_t k_error, tmp_i32;
  uint32_t tmp_u32;
  volatile uint16_t *p;
#if NUM_FILTER_POINTS == 5  
  uint16_t pf0, pf1, pf2, pf3, pf4;
#elif NUM_FILTER_POINTS == 9
  uint16_t pf0, pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8;
#endif
  static unsigned flags      = FLAGS_STARTUP;
  static uint32_t loop_count = STARTUP_COUNT; 
  bool _ramp_up;
  uint16_t i_peak = 0;

#if USE_SERIAL != SERIAL_NONE
  // check if peak data should be sent
  static int peak_to_send = PEAK_NONE;
  static int32_t value = 0;
  int cmd = CMD_NONE;
#if USE_SERIAL == SERIAL_NATIVE 
  int32_t *deriv = NULL, *d = NULL;
  uint32_t result;
#endif  
  if (SERIAL_TYPE.available() > 0) {
    peak_to_send = PEAK_NONE;
    flags        = FLAGS_NONE;
    loop_count   = 0;
    cmd = parse_command(&value);
    switch(cmd) {
      case CMD_PEAK_HALF:
        // get peak 0,1,2 individually or 3=all peaks
        if ((value >= 0) && (value <= NUM_PEAKS)) {
          peak_to_send = value; 
        }
        break;
      case CMD_PEAK_HALF_REP:
        // get peak 0,1,2 individually or 3=all peaks
        if ((value >= 0) && (value <= NUM_PEAKS)) {
          peak_to_send = value;
          flags = FLAGS_REPEAT;
        }
        break;
      case CMD_PEAK_FULL:
        // get peak 0,1,2 individually or 3=all peaks
        if ((value >= 0) && (value <= NUM_PEAKS)) {
          peak_to_send = value; 
#if USE_SERIAL == SERIAL_NATIVE
          flags = FLAGS_FULL;
#endif          
        }
        break;
      case CMD_PEAK_FULL_REP:
        // get peak 0,1,2 individually or 3=all peaks
        if ((value >= 0) && (value <= NUM_PEAKS)) {
          peak_to_send = value;
#if USE_SERIAL == SERIAL_NATIVE
          flags = FLAGS_REPEAT | FLAGS_FULL;
#else
          flags = FLAGS_REPEAT;
#endif
        }
        break;
#if USE_SERIAL == SERIAL_NATIVE
      case CMD_PEAK_DERIV_HALF:
        // get peak 0,1,2 individually or 3=all peaks
        if ((value >= 0) && (value <= NUM_PEAKS)) {
          peak_to_send = value; 
          flags = FLAGS_DERIV;
        }
        break;
      case CMD_PEAK_DERIV_HALF_REP:
        // get peak 0,1,2 individually or 3=all peaks
        if ((value >= 0) && (value <= NUM_PEAKS)) {
          peak_to_send = value; 
          flags = FLAGS_REPEAT | FLAGS_DERIV;
        }
        break;
      case CMD_PEAK_DERIV_FULL:
        // get peak 0,1,2 individually or 3=all peaks
        if ((value >= 0) && (value <= NUM_PEAKS)) {
          peak_to_send = value;
          flags = FLAGS_FULL | FLAGS_DERIV;
        }
        break;
      case CMD_PEAK_DERIV_FULL_REP:
        // get peak 0,1,2 individually or 3=all peaks
        if ((value >= 0) && (value <= NUM_PEAKS)) {
          peak_to_send = value;
          flags = FLAGS_REPEAT | FLAGS_FULL | FLAGS_DERIV;
        }
        break;
#endif
      case CMD_GET_LASER_SETPOINT:
        SERIAL_TYPE.println(laser_set);
        break;
      case CMD_SET_LASER_SETPOINT:
        laser_set = value;
        SERIAL_TYPE.println(laser_set);
        break;
      case CMD_GET_RAMP_SETPOINT:
        SERIAL_TYPE.println(ramp_set/MCK);
        break;
      case CMD_SET_RAMP_SETPOINT:
        ramp_set = value*MCK;
        break;
#if USE_SERIAL == SERIAL_PROG
      case CMD_STEP_LASER:
        // get all peaks with +/-step on laser setpoint after STEP_TIME_0 + STEP_TIME_1
        if ((value >= 0) && (value <= AOUT_MASK)) {
          peak_to_send = NUM_PEAKS;
          flags = FLAGS_LASER | FLAGS_REPEAT | FLAGS_STEP_0;
          loop_count = STEP_TIME_0;
          Serial.print("laser step ");
          Serial.print(value);
          Serial.println(" ...");
        }
        break;
      case CMD_STEP_RAMP:
        // get all peaks with +/-step on ramp setpoint after STEP_TIME_0 + STEP_TIME_1
        if ((value >= 0) && (value <= AOUT_MASK)) {
          peak_to_send = NUM_PEAKS;
          flags = FLAGS_RAMP | FLAGS_REPEAT | FLAGS_STEP_0;
          loop_count = STEP_TIME_0;
          Serial.print("ramp step ");
          Serial.print(value);
          Serial.println(" ...");
        }
        break;
#endif
      case CMD_GET_LASER_KP:
        SERIAL_TYPE.println(laser_kp);
        break;
      case CMD_SET_LASER_KP:
        laser_kp = value;
        break;
      case CMD_GET_RAMP_KP:
        SERIAL_TYPE.println(ramp_kp);
        break;
      case CMD_SET_RAMP_KP:
        ramp_kp = value;
        break;
      case CMD_GET_LASER_KI:
        SERIAL_TYPE.println(laser_ki);
        break;
      case CMD_SET_LASER_KI:
        laser_ki = value;
        break;
      case CMD_GET_RAMP_KI:
        SERIAL_TYPE.println(ramp_ki);
        break;
      case CMD_SET_RAMP_KI:
        ramp_ki = value;
        break;
      case CMD_GET_LASER_OUT:
        SERIAL_TYPE.println(laser_out);
        break;
      case CMD_SET_LASER_OUT:
        // we set output and integral part
        value = value << LASER_DIV;
        if      ( value < laser_min ) laser_i_out = laser_min;
        else if ( value > laser_max ) laser_i_out = laser_max;
        else                          laser_i_out = value;
        laser_out = laser_i_out >> LASER_DIV;
#if USE_SERIAL == SERIAL_PROG       
        Serial.println(laser_out);
#endif
        break;
      case CMD_GET_RAMP_OUT:
        SERIAL_TYPE.println(ramp_out);
        break;
      case CMD_SET_RAMP_OUT:
        // we set output and integral part
        value = value << RAMP_DIV;
        if      ( value < ramp_min ) ramp_i_out = ramp_min;
        else if ( value > ramp_max ) ramp_i_out = ramp_max;
        else                         ramp_i_out = value;
        ramp_out = ramp_i_out >> RAMP_DIV;
#if USE_SERIAL == SERIAL_PROG       
        Serial.println(ramp_out);
#endif
        break;
      case CMD_RST:
        laser_i_out = laser_offset<<LASER_DIV;
        laser_out   = laser_offset;
        ramp_i_out  = ramp_offset <<RAMP_DIV;
        ramp_out    = ramp_offset;
        break;
      case CMD_REP_OFF:
        peak_to_send = PEAK_NONE;
        flags        = FLAGS_NONE;
        loop_count   = 0;
        break;
      default:
#if USE_SERIAL == SERIAL_PROG      
        Serial.println(" cmd error!");
#endif
        break;
    }
  }
#endif

  for(i = 0; i < NUM_PEAKS; ++i) { 
    t_peak[i] = 0;
    i_peak = 0;
    // wait until peak data is available, i.e. high threshold and after this low threshold reached
    // if ramp_up flag changes during waiting we have missed peaks and exit loop
    // this allows to send also serial commands when no signal is present
    _ramp_up = ramp_up;
    while(peak != i) {
      if (ramp_up != _ramp_up) {
#if NUM_PEAKS <= 3
          if ((i == 0) || (_ramp_up && (i == 2)) ) {
            // for peak 0 we ignore ramp state since want to wait for it indefinitely.
            // for peak 2 and 5 ignore ramp change 1x since might be too close to change
            _ramp_up == ramp_up; // wait for peak 0 ignoring ramp state 
          }
          else {
            error = -10 -i;
            break;
          }
#else
          if ( (i == 0) || (_ramp_up && (i == 2)) || ((!_ramp_up) && (i == 5)) ) {
            // for peak 0 we ignore ramp state since want to wait for it indefinitely.
            // for peaks 2 and 5 ignore ramp change 1x since might be too close to change
            _ramp_up = ramp_up;
          }
          else {
            error = -10 - i;
            break;
          }
#endif
      }
    }
    if (error) {
      // peak not found
      break;
    }
    
    // saved for checking if buffer is overwritten during calculation
    _t_high = t_high;
    // we must save trigger time for the first peak since it might be overwritten for last peak
    if (i == 0) _t_trig = t_trig;

#if TEST_OUT_1_USE == TO1_CALC_TIME
    // high during calculation time
    digitalWrite(TEST_OUT_1, HIGH);
#endif

    if (false) {
      // TODO: this always fires!?
      // check consistency of t_high with i_high and n_high
      // in very rare cases it was inconsistent but this should be solved. we still keep the check.
      k_error = TICK_DIFF(t_buf[n_high], t_high) - i_high*ADC_TIME;
      if (k_error < 0) k_error = -k_error;
      if (k_error > (MCK<<1)) { 
#if USE_SERIAL == SERIAL_PROG
        Serial.print("|t_high - t_buf[n_high] - i_high*ADC_TIME| = "); 
        Serial.print(k_error/MCK); 
        Serial.println("us > 2us"); 
#endif
        // exit loop with error
        error = -2;
        break;
      }
      k_error = TICK_DIFF(t_buf[n_low], t_low) - i_low*ADC_TIME;
      if (k_error < 0) k_error = -k_error;
      if (k_error > (MCK<<1)) { 
#if USE_SERIAL == SERIAL_PROG      
        Serial.print("|t_low - t_buf[n_low] - i_low*ADC_TIME| = "); 
        Serial.print(k_error/MCK); 
        Serial.println(" > 2us"); 
#endif
        // exit loop with error
        error = -3;
        break;
      }
    }

    // invert buffer indices since they count down
    // note: this assumes 0 < i_high,i_low <= BUF_SIZE
    //       very rarely get i_high = 0. in this case assume i_high = 1. error is small.
#if USE_SERIAL == SERIAL_PROG
    if ((i_high == 0) || (i_high > BUF_SIZE)) { Serial.print("i_high "); Serial.print(i_high); Serial.println(" outside!"); }
    if ((i_low  == 0) || (i_low  > BUF_SIZE)) { Serial.print("i_low " ); Serial.print(i_low ); Serial.println(" outside!"); }
#endif
    i_high = (i_high == 0) ? BUF_SIZE - 1 : BUF_SIZE - i_high;
    i_low  = (i_low  == 0) ? BUF_SIZE - 1 : BUF_SIZE - i_low;
    // get starting buffer and index NUM_EDGE samples before i_high
    if (i_high >= NUM_EDGE) {
      _n_high = n_high;
      _i_high = i_high - NUM_EDGE;
    }
    else {
      _n_high = (n_high == 0) ? DMA_BUFFERS-1 : n_high-1;
      _i_high = BUF_SIZE + i_high - NUM_EDGE;      
    }
    // get ending buffer and index NUM_EDGE samples after i_low
    if ((i_low + NUM_EDGE) < BUF_SIZE) {
      _n_low = n_low;
      _i_low = i_low + NUM_EDGE;
    }
    else {
      _n_low = (n_low + 1) & BUF_MASK;
      _i_low = (i_low + NUM_EDGE) - BUF_SIZE;
    }
    // calculate number of samples from i_high (inclusive) .. i_low (exclusive)
    // additional 2xNUM_EDGE samples are added (before i_high and after i_low) for proper calculation of derivative.
    samples = 0;
    for (n = _n_high; n != _n_low; n = (n+1) & BUF_MASK, samples += BUF_SIZE);
    samples = (samples + _i_low) - _i_high;
#if USE_SERIAL == SERIAL_NATIVE
    if ((peak_to_send == NUM_PEAKS) || (peak_to_send == i)) {
      info.flags        =  i | (n_high << 4) | (n_low << 8) | (DMA_BUFFERS << 12) | (NUM_EDGE << 16) | (cal_div << 20) | 
                           ((flags & FLAGS_DERIV) ? 1 << 28 : 0) | 
                           ((flags & FLAGS_FULL ) ? 1 << 29 : 0);      
      info.cal_ticks    = cal_ticks;
      info.samples      = 0;
      info.i_high       = i_high;
      info.i_low        = i_low;
      info.i_peak       = 0;
      info.t_trig       = _t_trig;
      info.t_high       = t_high;
      info.t_low        = t_low;
      info.t_peak       = 0;
      info.laser_error  = 0;
      info.laser_out    = 0;
      info.ramp_error   = 0;
      info.ramp_low     = 0;
      info.ramp_high    = 0;
      for(k = 0; k < DMA_BUFFERS; ++k) {
        info.t_buf[k] = t_buf[k];
      }
      if (flags & FLAGS_DERIV) {
        deriv = d = (int32_t *) malloc(samples*sizeof(int32_t));
        if (deriv == NULL) {
          while(1); // out of memory: we cannot do anything!
        }
      }
    }
#endif
    n = _n_high;
    j = _i_high;
    p = buf[n] + j;
    i_pf = i_q = 0;
#if NUM_FILTER_POINTS == 5
    pf0 = pf1 = pf2 = pf3 = pf4 = 0;
#elif NUM_FILTER_POINTS == 9
    pf0 = pf1 = pf2 = pf3 = pf4 = pf5 = pf6 = pf7 = pf8 = 0;
#endif  
#if NUM_FIT_POINTS == 2  
    q0 = q1 = 0;
#elif NUM_FIT_POINTS == 4
    q0 = q1 = q2 = q3 = 0;
#elif NUM_FIT_POINTS == 6
    q0 = q1 = q2 = q3 = q4 = q5 = 0;
#elif NUM_FIT_POINTS == 8
    q0 = q1 = q2 = q3 = q4 = q5 = q6 = q7 = 0;
#endif  
    for (index = 0; index < samples; ++index) {
      //pf[NUM_FILTER_POINTS-1] = *p;
      switch (i_pf) {
        case 0: pf0 = *p; break;
        case 1: pf1 = *p; break;
        case 2: pf2 = *p; break;
        case 3: pf3 = *p; break;
        case 4: pf4 = *p; break;
#if NUM_FILTER_POINTS == 9
        case 5: pf5 = *p; break;
        case 6: pf6 = *p; break;
        case 7: pf7 = *p; break;
        case 8: pf8 = *p; break;
#endif        
      }
#if FILTER == SG5
      // 5-point Savitzky-Golay Filter
      // notes: 
      // - this is a very efficient filter but requires good choice of the high threshold.
      //   if the high threshold is too low we might get zero crossings of the derivative due to residual noise spikes.
      // - according to scipy.signal.savgol_coeffs the coefficients should be [3,1,-1,-3] and not [2,1,-1,-2].
      //   the result is however nearly the same and *2 as <<1 is much more efficient than *3.
      // - for this filter NUM_EDGE = 2. first 2*NUM_EDGE samples are invalid and must be skipped!
      // - this filter produces some ripples when noise is in certain frequency range
      // - original implementation:
      //   q1 = (*(p+2) << 1) + *(p+1) - *(p-1) - (*(p-2) << 1);
      // - we use sum_k pk-nk to easily bridge buffer boundaries. efficiency should be similar.
      //q[NUM_FIT_POINTS-1] = ((pf[4] - pf[0]) << 1) + pf[3] - pf[1];
      switch (i_pf){
        case 0: tmp_i32 = ((INT32(pf0) - INT32(pf1)) << 1) + INT32(pf4) - INT32(pf2); break;
        case 1: tmp_i32 = ((INT32(pf1) - INT32(pf2)) << 1) + INT32(pf0) - INT32(pf3); break;
        case 2: tmp_i32 = ((INT32(pf2) - INT32(pf3)) << 1) + INT32(pf1) - INT32(pf4); break;
        case 3: tmp_i32 = ((INT32(pf3) - INT32(pf4)) << 1) + INT32(pf2) - INT32(pf0); break;
        case 4: tmp_i32 = ((INT32(pf4) - INT32(pf0)) << 1) + INT32(pf3) - INT32(pf1); break;
      }
#elif FILTER == SG9
      // notes: 
      // - this smoothens the curve considerably better than SG5 and makes choice of high threshold less critical.
      // - the coefficients from scipy.signal.savgol_coeffs are: [7,5,3,1,-1,-3,-5,-7].
      //   as for SG5 we use power of 2 for faster calculation:  [8,4,2,1,-1,-2,-4,-8].
      // - for this filter NUM_EDGE = 4. first 2*NUM_EDGE samples are invalid and must be skipped!
      // - this has fewer - but still visible - ripples than SG5, but this might depend on noise.
      // - we use sum_k pk-nk to easily bridge buffer boundaries
      //q[NUM_FIT_POINTS-1] = (pf[5] - pf[3]) + (pf[6] - pf[2]) * 3 + (pf[7] - pf[1]) * 5 + (pf[8] - pf[0]) * 7;
      switch (i_pf){
        case 0: tmp_i32 = (INT32(pf6)-INT32(pf4))+((INT32(pf7)-INT32(pf3))<<1)+((INT32(pf8)-INT32(pf2))<<2)+((INT32(pf0)-INT32(pf1))<<3); break;
        case 1: tmp_i32 = (INT32(pf7)-INT32(pf5))+((INT32(pf8)-INT32(pf4))<<1)+((INT32(pf0)-INT32(pf3))<<2)+((INT32(pf1)-INT32(pf2))<<3); break;
        case 2: tmp_i32 = (INT32(pf8)-INT32(pf6))+((INT32(pf0)-INT32(pf5))<<1)+((INT32(pf1)-INT32(pf4))<<2)+((INT32(pf2)-INT32(pf3))<<3); break;
        case 3: tmp_i32 = (INT32(pf0)-INT32(pf7))+((INT32(pf1)-INT32(pf6))<<1)+((INT32(pf2)-INT32(pf5))<<2)+((INT32(pf3)-INT32(pf4))<<3); break;
        case 4: tmp_i32 = (INT32(pf1)-INT32(pf8))+((INT32(pf2)-INT32(pf7))<<1)+((INT32(pf3)-INT32(pf6))<<2)+((INT32(pf4)-INT32(pf5))<<3); break;
        case 5: tmp_i32 = (INT32(pf2)-INT32(pf0))+((INT32(pf3)-INT32(pf8))<<1)+((INT32(pf4)-INT32(pf7))<<2)+((INT32(pf5)-INT32(pf6))<<3); break;
        case 6: tmp_i32 = (INT32(pf3)-INT32(pf1))+((INT32(pf4)-INT32(pf0))<<1)+((INT32(pf5)-INT32(pf8))<<2)+((INT32(pf6)-INT32(pf7))<<3); break;
        case 7: tmp_i32 = (INT32(pf4)-INT32(pf2))+((INT32(pf5)-INT32(pf1))<<1)+((INT32(pf6)-INT32(pf0))<<2)+((INT32(pf7)-INT32(pf8))<<3); break;
        case 8: tmp_i32 = (INT32(pf5)-INT32(pf3))+((INT32(pf6)-INT32(pf2))<<1)+((INT32(pf7)-INT32(pf1))<<2)+((INT32(pf8)-INT32(pf0))<<3); break;
      }
#endif

#if USE_SERIAL == SERIAL_NATIVE
      // save derivative
      if (flags & FLAGS_DERIV) {
        //*d++ = q[NUM_FIT_POINTS-1];
        *d++ = tmp_i32;
      }
#endif

      // update new entry in ring-buffer of derivatives
      switch(i_q) {
        case 0: q0 = tmp_i32; break;
        case 1: q1 = tmp_i32; break;
#if NUM_FIT_POINTS > 2        
        case 2: q2 = tmp_i32; break;
        case 3: q3 = tmp_i32; break;
#if NUM_FIT_POINTS > 4        
        case 4: q4 = tmp_i32; break;
        case 5: q5 = tmp_i32; break;
#if NUM_FIT_POINTS > 6
        case 6: q6 = tmp_i32; break;
        case 7: q7 = tmp_i32; break;
#endif
#endif
#endif
      }
      
      if (i_peak == 0) {
        //if (q[NUM_FIT_POINTS-1] < 0) {
        if (tmp_i32 < 0) {

          // ensure index is not too close to edge
          // this might happen when peak is too low or threshold too high!
          if (index < ((NUM_EDGE<<1)-1)) {
            error = -20;
            break;
          }
          else {
            // memorize peak position        
            i_peak = index;

#if NUM_FIT_POINTS == 2
            // calculate peak position in ticks
            // notes: 
            // - the factor q0/(q0-q1) = 0..1 linear interpolates between the ADC samples at index-1 and index
            // - we multiply q0*cal_ticks before dividing by (q0+q1) in order to have a good resolution in ticks.
            // - caution! this part might overflow uint32_t! 
            //   ensure that q0*cal_ticks < 2^31 by choosing CAL_BUFS small enough.
            //   (index*(q0-q1) + q0)*cal_ticks would require only 1x multiplication but would overflow! /2^cal_div
            //t_peak[i] = TICK_DIFF(_t_trig, _t_high) + (index-(NUM_EDGE<<1)-1)*ADC_TIME + (q[0]*ADC_TIME)/(q[0]-q[1]);
            t_peak[i] = TICK_DIFF(_t_trig, _t_high) + (index-(NUM_EDGE<<1)-1)*ADC_TIME + ((i_q == 0) ? (q1*ADC_TIME)/(q1-q0) : (q0*ADC_TIME)/(q0-q1));

#if USE_SERIAL == SERIAL_NONE
            // stop after fit done
            break;
#elif USE_SERIAL == SERIAL_PROG
            // keep loop running for full samples to check for error -60
#elif USE_SERIAL == SERIAL_NATIVE
            // stop if not taking full peak
            if (!(flags & FLAGS_FULL)) break;
#endif

#endif // NUM_FIT_POINTS == 2
          }
        } // end peak found
      }
      else { // i_peak > 0

#if NUM_FIT_POINTS > 2
        if ( index == (i_peak + NUM_FIT_HALF - 1) ) {
          // NUM_FIT/2 samples after peak found: linear regression of zero crossing from last NUM_FIT datapoints
          t_peak[i]  = TICK_DIFF(_t_trig, _t_high) + (i_peak-(NUM_EDGE<<1))*ADC_TIME;
#if NUM_FIT_POINTS == 4
          //t_peak[i] += ((q[0]+2*q[1]+3*q[2]+4*q[3])*ADC_TIME)/(3*(q[0]-q[3])+(q[1]-q[2]));
          switch (i_q) {
            case 0: t_peak[i] += ((q1+2*q2+3*q3+4*q0)*ADC_TIME)/(3*(q1-q0)+(q2-q3)); break;
            case 1: t_peak[i] += ((q2+2*q3+3*q0+4*q1)*ADC_TIME)/(3*(q2-q1)+(q3-q0)); break;
            case 2: t_peak[i] += ((q3+2*q0+3*q1+4*q2)*ADC_TIME)/(3*(q3-q2)+(q0-q1)); break;
            case 3: t_peak[i] += ((q0+2*q1+3*q2+4*q3)*ADC_TIME)/(3*(q0-q3)+(q1-q2)); break;
          }
#elif NUM_FIT_POINTS == 6        
          // t_peak[i] += ((10*q[0]+13*q[1]+16*q[2]+19*q[3]+22*q[4]+25*q[5])*ADC_TIME)/(15*(q[0]-q[5])+9*(q[1]-q[4])+3*(q[2]-q[3])); break;
          switch (i_q) {
            case 0: t_peak[i] += ((10*q1+13*q2+16*q3+19*q4+22*q5+25*q0)*ADC_TIME)/(15*(q1-q0)+9*(q2-q5)+3*(q3-q4)); break;
            case 1: t_peak[i] += ((10*q2+13*q3+16*q4+19*q5+22*q0+25*q1)*ADC_TIME)/(15*(q2-q1)+9*(q3-q0)+3*(q4-q5)); break;
            case 2: t_peak[i] += ((10*q3+13*q4+16*q5+19*q0+22*q1+25*q2)*ADC_TIME)/(15*(q3-q2)+9*(q4-q1)+3*(q5-q0)); break;
            case 3: t_peak[i] += ((10*q4+13*q5+16*q0+19*q1+22*q2+25*q3)*ADC_TIME)/(15*(q4-q3)+9*(q5-q2)+3*(q0-q1)); break;
            case 4: t_peak[i] += ((10*q5+13*q0+16*q1+19*q2+22*q3+25*q4)*ADC_TIME)/(15*(q5-q4)+9*(q0-q3)+3*(q1-q2)); break;
            case 5: t_peak[i] += ((10*q0+13*q1+16*q2+19*q3+22*q4+25*q5)*ADC_TIME)/(15*(q0-q5)+9*(q1-q4)+3*(q2-q3)); break;
          }
#elif NUM_FIT_POINTS == 8
          // t_peak[i] += ((7*q[0]+8*q[1]+9*q[2]+10*q[3]+11*q[4]+12*q[5]+13*q[6]+14*q[7])*ADC_TIME)/(7*(q[0]-q[7])+5*(q[1]-q[6])+3*(q[2]-q[5])+(q[3]-q[4]));
          switch (i_q) {
            case 0: t_peak[i] += ((7*q1+8*q2+9*q3+10*q4+11*q5+12*q6+13*q7+14*q0)*ADC_TIME)/(7*(q1-q0)+5*(q2-q7)+3*(q3-q6)+(q4-q5)); break;
            case 1: t_peak[i] += ((7*q2+8*q3+9*q4+10*q5+11*q6+12*q7+13*q0+14*q1)*ADC_TIME)/(7*(q2-q1)+5*(q3-q0)+3*(q4-q7)+(q5-q6)); break;
            case 2: t_peak[i] += ((7*q3+8*q4+9*q5+10*q6+11*q7+12*q0+13*q1+14*q2)*ADC_TIME)/(7*(q3-q2)+5*(q4-q1)+3*(q5-q0)+(q6-q7)); break;
            case 3: t_peak[i] += ((7*q4+8*q5+9*q6+10*q7+11*q0+12*q1+13*q2+14*q3)*ADC_TIME)/(7*(q4-q3)+5*(q5-q2)+3*(q6-q1)+(q7-q0)); break;
            case 4: t_peak[i] += ((7*q5+8*q6+9*q7+10*q0+11*q1+12*q2+13*q3+14*q4)*ADC_TIME)/(7*(q5-q4)+5*(q6-q3)+3*(q7-q2)+(q0-q1)); break;
            case 5: t_peak[i] += ((7*q6+8*q7+9*q0+10*q1+11*q2+12*q3+13*q4+14*q5)*ADC_TIME)/(7*(q6-q5)+5*(q7-q4)+3*(q0-q3)+(q1-q2)); break;
            case 6: t_peak[i] += ((7*q7+8*q0+9*q1+10*q2+11*q3+12*q4+13*q5+14*q6)*ADC_TIME)/(7*(q7-q6)+5*(q0-q5)+3*(q1-q4)+(q2-q3)); break;
            case 7: t_peak[i] += ((7*q0+8*q1+9*q2+10*q3+11*q4+12*q5+13*q6+14*q7)*ADC_TIME)/(7*(q0-q7)+5*(q1-q6)+3*(q2-q5)+(q3-q4)); break;
          }
#endif // NUM_FIT_POINTS individual code
        
#if USE_SERIAL == SERIAL_NONE
          // stop after fit done
          break;
#elif USE_SERIAL == SERIAL_PROG
          // keep loop running for full samples to check for error -60
#elif USE_SERIAL == SERIAL_NATIVE
          // stop if not taking full peak
          if (!(flags & FLAGS_FULL)) break;
#endif
        
        } // index == (i_peak + NUM_FIT_HALF - 1)
#endif // NUM_FIT_POINTS > 2
      } // i_peak > 0

      // increment filter and fit ring buffer counter
      if (++i_pf >= NUM_FILTER_POINTS) i_pf = 0;
      if (++i_q  >= NUM_FIT_POINTS   ) i_q  = 0;

      // next buffer index
      if (++j == BUF_SIZE) {
        j = 0;
        n = (n + 1) & BUF_MASK;
        p = buf[n];
      }
      else ++p;
      
    } // next sample index 

    // check for error and if peak found
    if (error) break;
    else if (i_peak == 0) {
      error = -30 - i;
      break;
    }
    else if (_t_high != t_high) {
      // ok only if buffer was not overwritten
      //error = -40;
      //break;
    }
    
#if USE_SERIAL == SERIAL_PROG
    if (t_peak[i] > (6000*MCK)) {
      Serial.print("error peak position ");
      Serial.print(i);
      Serial.print(" ");
      Serial.print(t_peak[i]);
      Serial.print(" ");
      Serial.print(_t_trig);
      Serial.print(" ");
      Serial.print(_t_high);
      Serial.print(" ");
      Serial.print(index);
      Serial.print(" ");
      Serial.print(i_q);
      Serial.print(" ");
      Serial.print(q0);
      Serial.print(" ");
      Serial.print(q1);
      Serial.print(" ");
#if NUM_FIT_POINTS > 2      
      Serial.print(q2);
      Serial.print(" ");
      Serial.print(q3);
      Serial.print(" ");
#if NUM_FIT_POINTS > 4      
      Serial.print(q4);
      Serial.print(" ");
      Serial.print(q5);
      Serial.print(" ");
#if NUM_FIT_POINTS > 6      
      Serial.print(q6);
      Serial.print(" ");
      Serial.print(q7);
      Serial.print(" ");
#endif
#endif
#endif
      Serial.println(error);          
      error = -70;
    }
#endif    

#if NUM_AVG > 0
    // average NUM_AVG peak positions
    tmp_u32 = t_peak_avg[i][avg_ptr];
    t_peak_avg[i][avg_ptr] = t_peak[i];
    t_peak_sum[i] += t_peak[i] - tmp_u32;
    if (i == (NUM_PEAKS-1)) {
      if (++avg_ptr >= NUM_AVG) avg_ptr = 0;
    }
#endif

#if NUM_PEAKS == 6
    if ( (i == ((NUM_PEAKS>>1)-1)) || (i == (NUM_PEAKS-1)) )
#else
    if ( i == (NUM_PEAKS-1) )
#endif
    { // feedback on laser and ramp
      
      if ( (i == (NUM_PEAKS-1)) && (loop_count > 0) ) {
        if (--loop_count == 0) {
          if (flags & FLAGS_STARTUP) {
            // during startup do not feedback on laser or ramp
            // this allows user to manually set ramp offset
            // note: startup time counts only when all peaks have been found
#if USE_SERIAL == SERIAL_PROG
            Serial.println("PI run!");
#endif                
            flags = 0;
          }
#if USE_SERIAL == SERIAL_PROG
          else if ((flags & FLAGS_STEP) == FLAGS_STEP_0) {
            // STEP_TIME_0: jump setpoint and wait STEP_TIME_1
            if (flags & FLAGS_LASER) {
              laser_set += value;
              Serial.print("#laser step +");
              Serial.println(value);
            }
            else if (flags & FLAGS_RAMP) {
              ramp_set += value;
              Serial.print("#ramp step +");
              Serial.println(value);
            }
            loop_count = STEP_TIME_1;
            flags = (flags & (~FLAGS_STEP)) | FLAGS_STEP_1;
          }
          else if ((flags & FLAGS_STEP) == FLAGS_STEP_1) {
            // STEP_1_TIME: jump -setpoint and continue sending data until "\n"
            if (flags & FLAGS_LASER) {
              laser_set -= value;
              Serial.print("#laser step -");
              Serial.println(value);
              flags = (flags & (~(FLAGS_STEP | FLAGS_LASER)));
            }
            else if (flags & FLAGS_RAMP) {
              ramp_set -= value;
              Serial.print("#ramp step -");
              Serial.println(value);
              flags = (flags & (~(FLAGS_STEP | FLAGS_RAMP)));
            }
          }
#endif
        }
      }

      if (!(flags & FLAGS_STARTUP)) {

        // calculate error signal for laser
        // note: when NUM_PEAKS == 6 we feedback 2x during ramp on laser (using last values on the not measured ramp), otherwise only 1x.
        //       since the error is calculated from relative position of peaks it does not matter if there is a hysteresis.
        // note: when I write this as a single equation the sign-bit is lost and get large positive values when it should be negative!?
#if NUM_PEAKS == 6

#if NUM_AVG > 0 // use average peak position
        laser_error  = (((int32_t)((t_peak_sum[5]-t_peak_sum[4])>>NUM_AVG_P2)) - ((int32_t)(((t_peak_sum[5]-t_peak_sum[3])>>NUM_AVG_P2)*laser_set)/laser_ref));
        laser_error += (((int32_t)((t_peak_sum[1]-t_peak_sum[0])>>NUM_AVG_P2)) - ((int32_t)(((t_peak_sum[2]-t_peak_sum[0])>>NUM_AVG_P2)*laser_set)/laser_ref));
        laser_error >>= 1; // ticks
#else // do not average
        laser_error  = ((int32_t)(t_peak[5] - t_peak[4]) - ((int32_t)((t_peak[5] - t_peak[3])*laser_set)/laser_ref));
        laser_error += ((int32_t)(t_peak[1] - t_peak[0]) - ((int32_t)((t_peak[2] - t_peak[0])*laser_set)/laser_ref));
        laser_error >>= 1; // ticks
#endif // NUM_AVG(int32_t)

#else // NUM_PEAKS == 3

#if NUM_AVG > 0 // use average peak position
        laser_error = ((t_peak_sum[1]-t_peak_sum[0])>>NUM_AVG_P2) - (((t_peak_sum[2]-t_peak_sum[0])>>NUM_AVG_P2)*laser_set)/laser_ref; // ticks
#else // do not average
        laser_error = (t_peak[1]-t_peak[0]) - ((t_peak[2]-t_peak[0])*laser_set)/laser_ref; // ticks
#endif // NUM_AVG

#endif // NUM_PEAKS == 6

        // feedback on laser 
        // laser_i_out += laser_ki*ramp_error;
        // laser_out    = (laser_i_out + laser_kp*laser_error)/2^LASER_DIV;
        // integral part limited to [laser_min, laser_max]
        k_error = laser_ki*laser_error;
        int64_t tmp_i64 = ((int64_t)laser_i_out) + k_error;
        if (k_error > 0) {
          tmp_i32 = laser_max - k_error;
          //laser_i_out = (tmp_i32 <= laser_min) ? laser_max : ((laser_i_out < tmp_i32) ? laser_i_out + k_error : laser_max);
          laser_i_out = (tmp_i32 <= 0) ? laser_max : ((laser_i_out < tmp_i32) ? laser_i_out + k_error : laser_max);
        }
        else if (k_error < 0) {
          tmp_i32 = laser_min - k_error;
          laser_i_out = (tmp_i32 >= laser_max) ? laser_min : (laser_i_out > tmp_i32) ? laser_i_out + k_error : laser_min;
        }

#if USE_SERIAL == SERIAL_PROG
        if      (tmp_i64 > laser_max) tmp_i64 = laser_max;
        else if (tmp_i64 < laser_min) tmp_i64 = laser_min;
        if ((uint32_t)(tmp_i64 & 0xffffffff) != laser_i_out) {
          Serial.print("error laser_i_out ");
          Serial.print(laser_i_out);
          Serial.print(" != ");
          Serial.print((uint32_t)(tmp_i64>>32));
          Serial.print("_");
          Serial.println((uint32_t)(tmp_i64 & 0xffffffff));
          // send all peaks
          for (k=0; k < NUM_PEAKS; ++k) {
            Serial.print(t_peak[k]);
            Serial.print(" ");
          }
          Serial.println("");
        }
#endif
      
        // add proportional part limted to [laser_min, laser_max]
        k_error = laser_kp*laser_error; 
        tmp_i64 = ((int64_t)laser_i_out) + k_error;
        if (k_error > 0) {
          tmp_i32 = laser_max - k_error;
          //tmp_u32 = (tmp_i32 <= laser_min) ? laser_max : ((laser_i_out < tmp_i32) ? laser_i_out + k_error : laser_max);
          tmp_u32 = (tmp_i32 <= 0) ? laser_max : ((laser_i_out < tmp_i32) ? laser_i_out + k_error : laser_max);
        }
        else if (k_error < 0) {
          tmp_i32 = laser_min - k_error;
          tmp_u32 = (tmp_i32 >= laser_max) ? laser_min : ((laser_i_out > tmp_i32) ? laser_i_out + k_error : laser_min);
        }
        else tmp_u32 = laser_i_out;

        // write DAC value into conversion FIFO without waiting to finish
        // notes:
        // - this writes the channel half-word directly into conversion FIFO with the channel tag indicating the LASER_OUT channel
        // - do not use high-level code here: 
        //   analogWrite would probably change settings and wait for conversion to finish
        //   dacc_set_channel_selection should return with error since we use channel selection tag in data
        //   dacc_write_conversion_data should do the same as we do here.
        // - AOUT_MASK is only for security here, laser_min and laser_max ensure that laser_out does not over- or underflow. 
        laser_out = (tmp_u32 >> LASER_DIV) & AOUT_MASK;
        DACC->DACC_CDR = laser_out | LASER_CHANNEL_TAG;

#if USE_SERIAL == SERIAL_PROG
        if      (tmp_i64 > laser_max) tmp_i64 = laser_max;
        else if (tmp_i64 < laser_min) tmp_i64 = laser_min;
        if ((uint32_t)(tmp_i64 & 0xffffffff) != tmp_u32) {
          Serial.print("error laser_out ");
          Serial.print(tmp_u32);
          Serial.print(" != ");
          Serial.print((uint32_t)(tmp_i64>>32));
          Serial.print("_");
          Serial.print((uint32_t)(tmp_i64 & 0xffffffff));
          Serial.print(" ");
          Serial.print(laser_i_out);
          Serial.print(" ");
          Serial.print(k_error);
          Serial.print(" ");
          Serial.println(tmp_i32);              
        }
        else if (laser_out != (tmp_u32 >> LASER_DIV)) {
          Serial.print("error AOT_MASK! ");
          Serial.println(tmp_u32);
        }
#endif
        if (i == (NUM_PEAKS-1)) {
          // calculate error signal for ramp
          // note: when NUM_PEAKS == 6 we still feedback only 1x during ramp on cavity.
          //       there might be a hysteresis which affects peak 5 absolute position.

#if NUM_AVG > 0 // use average peak position
          ramp_error = (t_peak_sum[0]>>NUM_AVG_P2) - ramp_set; // ticks
#else
          ramp_error = t_peak[0] - ramp_set; // ticks
#endif // NUM_AVG

          // feedback on cavity ramp generator offset = ramp low value
          // ramp_i_out += ramp_ki*ramp_error;
          // ramp_out    = (ramp_i_out + ramp_kp*ramp_error)/2^RAMP_DIV;
          // integral part limited to [ramp_min, ramp_max=(2^12-ramp_Vpp)<<RAMP_DIV]
          k_error = ramp_ki*ramp_error;
          if (k_error > 0) {
            tmp_i32 = ramp_max - k_error;
            //ramp_i_out = (tmp_i32 <= ramp_min) ? ramp_max : ((ramp_i_out < tmp_i32) ? ramp_i_out + k_error : ramp_max);
            ramp_i_out = (tmp_i32 <= 0) ? ramp_max : ((ramp_i_out < tmp_i32) ? ramp_i_out + k_error : ramp_max);
          }
          else {
            tmp_i32 = ramp_min - k_error;
            ramp_i_out = (tmp_i32 >= ramp_max) ? ramp_min : ((ramp_i_out > tmp_i32) ? ramp_i_out + k_error : ramp_min);
          }
          // add proportional part limted to [ramp_min, ramp_max=(2^12-ramp_Vpp)<<RAMP_DIV]
          k_error = ramp_kp*ramp_error;
          if (k_error > 0) {
            tmp_i32 = ramp_max - k_error;
            tmp_u32 = (tmp_i32 <= ramp_min) ? ramp_max : ((ramp_i_out < tmp_i32) ? ramp_i_out + k_error : ramp_max);
          }
          else if (k_error < 0) {
            tmp_i32 = ramp_min - k_error;
            tmp_u32 = (tmp_i32 >= ramp_max) ? ramp_min : ((ramp_i_out > tmp_i32) ? ramp_i_out + k_error : ramp_min);
          }
          else tmp_u32 = ramp_i_out;
          // set ramp low value
          // notes:
          // - ramp_out = ramp low value, i.e. the triangular ramp goes from ram_out to ramp_out + ramp_Vpp and back
          // - ramp_out is used in DACC_Handler called by next DACC_IRQ when new buffer loaded.
          //   ramp_out defines start and end of next DMA output buffer.
          //   this will take into effect between 1x and 2x BUF_SIZE*ADC_TIME_MU after ramp_out is updated here
          // - AOUT_MASK is only for security here, ramp_max and ramp_mix ensure that ramp_out does not over- or underflow.
          ramp_out = (tmp_u32 >> RAMP_DIV) & AOUT_MASK;
        }
      }
    } // feedback on laser

#if USE_SERIAL == SERIAL_PROG
    if (i == (NUM_PEAKS-1)) {
      // check if j is as expected, otherwise we have serious trouble.
      if (j != _i_low) {
        Serial.print("error -60! j = ");
        Serial.print(j);
        Serial.print(" _i_low = ");
        Serial.print(_i_low);
        Serial.print(" i_low = ");
        Serial.print(i_low);
        Serial.print(" i_high = ");
        Serial.print(i_high);
        Serial.print(" index = ");
        Serial.println(index);
        error = -60;
        break;
      }
    }

    if ( (peak_to_send == i) || ((peak_to_send == NUM_PEAKS) && (i == (NUM_PEAKS-1))) ) {
      if (peak_to_send == NUM_PEAKS) {
        // send all peaks
        for (k=0; k < NUM_PEAKS; ++k) {
          Serial.print(t_peak[k]);
          Serial.print(" ");
        }
      }
      else {
        // send peak 0,1,2 individually
        Serial.print(t_peak[peak_to_send]);
        Serial.print(" ");
      }
      // send laser error and output
      Serial.print(laser_error);
      if (0) {
        tmp_i32 = ((int32_t)(t_peak[1] - t_peak[0]) - ((int32_t)((t_peak[2] - t_peak[0])*laser_set)/laser_ref));
        Serial.print(" ");
        Serial.print(tmp_i32);
        tmp_i32 = ((int32_t)(t_peak[5] - t_peak[4]) - ((int32_t)((t_peak[5] - t_peak[3])*laser_set)/laser_ref));
        Serial.print(" ");
        Serial.print(tmp_i32);        
        tmp_i32 += ((int32_t)(t_peak[1] - t_peak[0]) - ((int32_t)((t_peak[2] - t_peak[0])*laser_set)/laser_ref));
        Serial.print(" ");
        Serial.print(tmp_i32);        
        Serial.print(" ");
        Serial.print(tmp_i32>>1);        
      }
      Serial.print(" ");
      Serial.print(laser_out);
      Serial.print(" ");
      // send cavity ramp error and output
      Serial.print((int)ramp_error);
      Serial.print(" ");
      if (0) {
        Serial.print(ramp_out);
        Serial.print(" ");
        Serial.print(_n_high);
        Serial.print(" ");
        Serial.print(_n_low);
        Serial.print(" ");
        Serial.print(i_high);
        Serial.print(" ");
        Serial.println(_i_high);
      }
      else {
        Serial.println(ramp_out);
        /*tmp_i32 = ((int32_t)(t_peak[5] - t_peak[4]) - ((int32_t)((t_peak[5] - t_peak[3])*laser_set)/laser_ref));
        Serial.print(" ");
        Serial.print(tmp_i32);
        tmp_i32 = ((int32_t)(t_peak[1] - t_peak[0]) - ((int32_t)((t_peak[2] - t_peak[0])*laser_set)/laser_ref));
        Serial.print(" ");
        Serial.println(tmp_i32);
        */
      }

      // command is done if not repeat flag is set
      if (!(flags & FLAGS_REPEAT)) {
        peak_to_send = PEAK_NONE;
        flags        = FLAGS_NONE;
      }
    }
#elif USE_SERIAL == SERIAL_NATIVE
    // send data even if not ok
    if ( (peak_to_send == i) || (peak_to_send == NUM_PEAKS) ) {

      // for half peak send data only until index = i_peak + 1
      if (!(flags & FLAGS_FULL)) {
        _n_low  = n;
        _i_low  = j; 
        samples = index;
        info.flags = (info.flags & (~0xf00)) | (_n_low << 8); // update n_low in flags
      }
      
      // for full peak we can check if j is as expected
      if ((flags & FLAGS_FULL) && (j != _i_low)) {
        if (!error) error = -80;
        j = _i_low;
      }
      
      // write header
      info.samples      = samples;
      info.i_peak       = i_peak;
      info.t_peak       = t_peak[i];
      info.laser_error  = laser_error;
      info.ramp_error   = ramp_error;
      info.laser_out    = laser_out;
      info.ramp_low     = ramp_out;
      info.ramp_high    = ramp_out + ramp_Vpp;
      SerialUSB.write((char*)&info, sizeof(struct header));
      if (_n_high == _n_low) {
        // write single buffer
        SerialUSB.write((char*)(buf[_n_high] + _i_high), (j - _i_high)*sizeof(uint16_t));
        //SerialUSB.write((char*)buf[_n_high], BUF_SIZE*sizeof(uint16_t));
        //if ((!error) && (n_buf == _n_high)) error = -100; // buffer overwritten! probably same as error -32.
      }
      else {
        // write first buffer
        n = _n_high;
        SerialUSB.write((char*)(buf[n] + _i_high), (BUF_SIZE - _i_high)*sizeof(uint16_t));
        if ((!error) && (n_buf == n)) error = -110; // buffer overwritten!
        // write full buffers
        n = (n + 1) & BUF_MASK;
        for (; n != _n_low; n = (n + 1) & BUF_MASK) {
          SerialUSB.write((char*)buf[n], BUF_SIZE*sizeof(uint16_t));
          if ((!error) && (n_buf == n)) error = -120; // buffer overwritten!
        }
        if (j > 0) {
          // write last buffer
          SerialUSB.write((char*)buf[n], j*sizeof(uint16_t));
          //if ((!error) && (n_buf == n)) error = -130; // buffer overwritten! -> cannot check this since buffer is always active when we stop scanning.
        }
      }
      // send derivative
      // note: the first 2*NUM_EDGE samples are invalid and should be skipped.
      //       deriv[2*NUM_EDGE] corresponds to data[NUM_EDGE] at n_high, i_high.
      if (flags & FLAGS_DERIV) {
        SerialUSB.write((char*)deriv, samples*sizeof(int32_t));
        free(deriv);
        deriv = NULL;
      }
      // write again number of samples and high word 0 if ok, error code if not ok.
      result = samples | (error << 16);
      SerialUSB.write((char*)&result, sizeof(uint32_t));
      
      // command is done if not repeat flag is set
      if (!(flags & FLAGS_REPEAT)) {
        if (peak_to_send == i) peak_to_send = PEAK_NONE;
        else if ( (peak_to_send == NUM_PEAKS) && (i == (NUM_PEAKS-1)) ) peak_to_send = PEAK_NONE;
        // reset flags. if FLAGS_FULL would be set the calculation time remaings for full peak.
        flags = FLAGS_NONE;
      }
      
    }
#endif // USE_SERIAL

#if TEST_OUT_1_USE == TO1_CALC_TIME
    // we are done
    digitalWrite(TEST_OUT_1, LOW);
#endif
  } // next peak

  // on error switch LED on for a given time
  if (error) {
    digitalWrite(LED, HIGH);
#if TEST_OUT_1_USE == TO1_CALC_TIME
    digitalWrite(TEST_OUT_1, LOW); // on error this is not reset
#endif
#if USE_SERIAL == SERIAL_PROG
    if ((old_error == 0) || (!(flags & FLAGS_ERROR))) {
      Serial.print("error ");
      Serial.println(error);
    }
    else if (old_error != error) {
      Serial.print("new error ");
      Serial.println(error);
    }
#endif
    flags |= FLAGS_ERROR;
    old_error = error;
    loop_count = LED_ERROR_COUNT;
  }
  else if (old_error) {
    if (--loop_count == 0) {
#if USE_SERIAL == SERIAL_PROG
      Serial.print("error resolved.");
#endif
      flags &= ~FLAGS_ERROR;
      old_error = 0;
      digitalWrite(LED, LOW);
    }
  }
}
