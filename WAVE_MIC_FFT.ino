#include <DMAChannel.h>
#include <Audio.h>
#include "memcpy16.h"

#define SPEED_OF_SOUND_25 (346.45)

// AUDIO_SAMPLE_RATE_EXACT
#define AUDIO_SAMPLE_RATE_KHZ (int)(AUDIO_SAMPLE_RATE_EXACT / 1000)

#define PDB_CONFIG     (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE | PDB_SC_CONT | PDB_SC_DMAEN)

// DMA for wave out to DAC
DMAChannel wave_dma(false);

#define USE_FFT1024
//#define USE_FFT256

#if defined(USE_FFT1024)
  #include "buf_analyze_fft1024.h"
  BufferAnalyzeFFT1024     bufFFT;
  #define FFT_BLOCK_CNT 8
  #define FFT_READ_MAX 512
  #define FFT_FRQ_ACC (AUDIO_SAMPLE_RATE_EXACT / 1024.)
#elif defined(USE_FFT256)
  #include "buf_analyze_fft256.h"
  BufferAnalyzeFFT256     bufFFT;
  #define FFT_BLOCK_CNT 2
  #define FFT_READ_MAX 128
  #define FFT_FRQ_ACC (AUDIO_SAMPLE_RATE_EXACT / 256.)
#endif


#if defined(__MK20DX256__)  // Teensy 3.2
  #define BUTTON_PIN 8
#elif defined(__MK64FX512__) // Teensy 3.5
  #define BUTTON_PIN 32
#endif

/*
#define WAVE_20KHz_START_PIN 6
#define WAVE_10KHz_START_PIN 5
#define WAVE_5KHz_START_PIN 4
#define WAVE_2KHz_START_PIN 3
*/
/*
#if defined(__MK20DX256__)  // Teensy 3.2
  #define DAC_PIN A14
#elif defined(__MK64FX512__) // Teensy 3.5
  #define DAC_PIN A21
#endif
*/
/*
#define TONE_PIN 35
*/
#define CAP_TIME_MAX 700 // unit is msec
//#define CAP_TIME_MAX 300 // unit is msec

#define CAP_TIME_DIV 4
//#define CAP_TIME_DIV 2
//#define CAP_TIME_DIV 1

#define CAP_BLOCK_SIZE ((AUDIO_SAMPLE_RATE_KHZ * CAP_TIME_MAX / AUDIO_BLOCK_SAMPLES) + 1) // approx. 263

#define WAVE_FFT_LEVEL_MAX 600
#define WAVE_FFT_LEVEL_MIN 300

#define WAVE_IN_GAIN_MAX 65
#define WAVE_IN_GAIN_MIN 0

#define WAVE_OUT_VOL_MAX 100
#define WAVE_OUT_VOL_MIN 5

//#define WAVW_IN_GAIN_DEFAULT 0;
#define WAVW_IN_GAIN_DEFAULT 35;
//#define WAVW_IN_GAIN_DEFAULT 100;

#define WAVW_OUT_VOL_DEFAULT 50;

#define WAVE_AUTO_SELECT
//#define WAVE_COMBINED_USE

#define WAVE_DURATION 1 // unit is msec
//#define WAVE_DURATION 2 // unit is msec

typedef struct {
  int cnt;
  int size;
  int16_t *buf;
  int in_gain;
  int out_vol;
  int level;
} wave_data_t;


volatile int wave_cnt = 0;
volatile int wave_end = 0;
int wave_cnt_max;

/**/
#define WAVE_INFO_CNT 4

int wave_freq[WAVE_INFO_CNT] =
{
  20000,
  10000,
  5000,
  2000
};
/**/
/*
#define WAVE_INFO_CNT 7

int wave_freq[WAVE_INFO_CNT] =
{
  20000,
  15000,
  10000,
  5000,
  4000,
  3000,
  2000
};
*/

#if defined(WAVE_COMBINED_USE)
  // array for waves data and combined wave data
  wave_data_t wave_data[WAVE_INFO_CNT + 1];
  int wave_sel = WAVE_INFO_CNT; // for select combined wave
#else
  // array for waves data
  wave_data_t wave_data[WAVE_INFO_CNT];
  wave_data_t wave_data_act;
  #if defined(WAVE_AUTO_SELECT)
    int wave_sel = -1; // for auto select
  #else
    int wave_sel = 0;
    //int wave_sel = 1;
    //int wave_sel = 2;
    //int wave_sel = 3;
    //int wave_sel = 4;
  #endif
#endif

// Pin 13 has the LED on Teensy 3.1 - will use this pin also as the square wave TTL output
//int ledpin = 13;

volatile int out_index = 0;
volatile int out_start = 0;
volatile int out_stop = 0;
volatile int out_end = 0;

volatile int cap_blk_cnt = 0;

/*
// the setup routine runs once when you press reset:
IntervalTimer timer0;
IntervalTimer timer1;
*/

// GUItool: begin automatically generated code
AudioInputI2S            i2s1;           //xy=105,63
AudioRecordQueue         queue1;         //xy=281,63
AudioConnection          patchCord1(i2s1, 0, queue1, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=265,212
// GUItool: end automatically generated code

// which input on the audio shield will be used?
//const int audio_input = AUDIO_INPUT_LINEIN;
const int audio_input = AUDIO_INPUT_MIC;

int16_t cap_buf[CAP_BLOCK_SIZE * AUDIO_BLOCK_SAMPLES];

#if defined(WAVE_COMBINED_USE)
  int distance[WAVE_INFO_CNT] = {0}; // unit is centi-meter
  int msec_index_first[WAVE_INFO_CNT] = {0};
  int msec_index_highest[WAVE_INFO_CNT] = {0};
#else
  int distance = 0; // unit is centi-meter
  int msec_index_first = 0;
  int msec_index_highest = 0;
#endif

// Remember which mode we're doing
int mode = 0;  // 0=stopped, 1=recording

// Setup DAC0
void dac_setup()
{
  // setup DAC 
  SIM_SCGC2 |= SIM_SCGC2_DAC0;
  DAC0_C0 = DAC_C0_DACEN; // 1.2V VDDA is DACREF_2
  DAC0_C0 |= DAC_C0_DACRFS; // 3.3V
  
  // slowly ramp up to DC voltage, approx 1/4 second
  for (int16_t i = 0; i <= 2048; i += 8)
  {
    *(int16_t *)&(DAC0_DAT0L) = i;
    delay(1);
  }
}

// Setup PDB freq at n usec
void pdb_setup(int usec)
{
  uint32_t mod = ((double)F_BUS / 1000000.) * (double)usec;

  //Serial.printf("usec = %d\n", usec);
  //Serial.printf("F_BUS = %d\n", F_BUS);
  //Serial.printf("(F_BUS / 1000000) * usec = %lf\n", ((double)F_BUS / 1000000.) * (double)usec);

  SIM_SCGC6 |= SIM_SCGC6_PDB; // enable PDB clock

  PDB0_MOD = (uint16_t)(mod-1);  
  PDB0_IDLY = 0;
  PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
  PDB0_SC = PDB_CONFIG | PDB_SC_SWTRIG;
  PDB0_CH0C1 = 0x0101;
}

// Setup & start DMA for DAC transfer - ref sin 
void wave_dma_start(int16_t *lut, int lut_size)
{
  wave_dma.disable();
  wave_dma.sourceBuffer(lut, lut_size*sizeof(uint16_t));
  wave_dma.transferSize(2);
  wave_dma.destination(*(volatile uint16_t *)&(DAC0_DAT0L));
  //wave_dma.interruptAtHalf();
  wave_dma.interruptAtCompletion();
  wave_dma.triggerAtHardwareEvent(DMAMUX_SOURCE_PDB);
  
  wave_dma.attachInterrupt(wave_dma_isr_completion);
  wave_dma.enable();
}

void wave_dma_isr_completion(void)
{
  wave_dma.clearInterrupt();
  wave_cnt ++;
  if (wave_cnt >= wave_cnt_max)
  {
    wave_dma.disable();
    wave_end = 1;
  }
}

FASTRUN void isr_button_push()
{
  wave_sel ++;
  if (wave_sel >= WAVE_INFO_CNT + 1)
  {
    wave_sel = 0;
  }
  //Serial.print("Button pushed!");
  //Serial.println(wave_sel);
}


void setup()
{
  int i;

  for (i = 0; i < 5; i ++)
  {
    Serial.println("wait...");
    delay(1000);
  }

  // Make waves data
  for (i = 0; i < WAVE_INFO_CNT; i ++)
  {
    wave_data[i].cnt = wave_freq[i] / 1000 * WAVE_DURATION;
    wave_data[i].in_gain = WAVW_IN_GAIN_DEFAULT;
    wave_data[i].out_vol = WAVW_OUT_VOL_DEFAULT;
    wave_data[i].buf = make_sine_wave(wave_freq[i] / 1000, &wave_data[i].size);
  }

#if defined(WAVE_COMBINED_USE)
  make_combined_wave();
#else
  wave_data_act.buf = NULL;
#endif

  pinMode(BUTTON_PIN, INPUT_PULLUP); // sets the digital pin as input
  attachInterrupt(BUTTON_PIN, isr_button_push, RISING);

  dac_setup();

  pdb_setup(1); // 1 usec

  // Allocate the wave_dma channels
  wave_dma.begin(true);

  /*
  pinMode(WAVE_20KHz_START_PIN,OUTPUT);
  digitalWrite(WAVE_20KHz_START_PIN, HIGH);

  pinMode(WAVE_10KHz_START_PIN,OUTPUT);
  digitalWrite(WAVE_10KHz_START_PIN, HIGH);

  pinMode(WAVE_5KHz_START_PIN,OUTPUT);
  digitalWrite(WAVE_5KHz_START_PIN, HIGH);

  pinMode(WAVE_2KHz_START_PIN,OUTPUT);
  digitalWrite(WAVE_2KHz_START_PIN, HIGH);
  */

  /*
  analogWriteResolution(12);
  pinMode(DAC_PIN,OUTPUT);
  analogWrite(DAC_PIN,0x800);
  timer0.priority(0);
  timer1.priority(1);
  */

  // Audio connections require memory, and the record queue
  // uses this memory to buffer incoming audio.
  AudioMemory(CAP_BLOCK_SIZE / 2);

  // Enable the audio shield, select input, and enable output
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(audio_input);
  // Set mic gain
  //sgtl5000_1.micGain(70);
  //sgtl5000_1.micGain(40);
  //sgtl5000_1.micGain(wave_data[wave_sel].in_gain);
  //sgtl5000_1.micGain(0);

  //bufFFT.windowFunction(AudioWindowHanning1024);
}

void loop() {
  if (mode == 0) {
    //Serial.println("mode=0");
    /*
    // Plott wave data buffer
    if (wave_data[wave_sel].size < 500)
    {
      for (int i = 0; i < wave_data[wave_sel].size * 500 / wave_data[wave_sel].size; i ++)
      {
        Serial.printf("-2048 %d 2047\n", wave_data[wave_sel].buf[i % wave_data[wave_sel].size] - 2048);
      }
    }
    else
    {
      for (int i = 0; i < wave_data[wave_sel].size; i += wave_data[wave_sel].size / 500)
      {
        int j;
        int avg = 0;
        for (j = 0; j < wave_data[wave_sel].size / 500; j ++)
        {
          avg += wave_data[wave_sel].buf[i + j];
        }
        avg = avg / j;
        Serial.printf("-2048 %d 2047\n", avg - 2048);
      }
    }
    //delay(1000);
    */
#if defined(WAVE_AUTO_SELECT)
    wave_sel ++;
    if (wave_sel >= WAVE_INFO_CNT)
    {
      wave_sel = 0;
    }
#endif
    make_active_wave();

    sgtl5000_1.micGain(wave_data_act.in_gain);

    wave_end = 0;
    wave_cnt = 0;
    wave_cnt_max = wave_data_act.cnt;
    wave_dma_start(wave_data_act.buf, wave_data_act.size);
    /*
    digitalWrite(freq_pin_num[wave_sel], LOW);
    delay(10);
    digitalWrite(freq_pin_num[wave_sel], HIGH);
    */
    startMicCapture();
    // play a 11KHz tone on pin 5 for 1 ms:
    //tone(TONE_PIN, 11000, 1);
    if (out_start == 0) {
/*
      Serial.println("startWave");
      out_start = 1;
      out_stop = 0;
      out_end = 0;
      //timer1.begin(timer1_callback, 1000);
      timer1.begin(timer1_callback, 1000*1000);
      timer0.begin(timer0_callback, 1);
      startMicCapture();
*/
    }
    else {
/*
      if (out_end)
      {
        Serial.println("endWave");
        startMicCapture();
      }
*/
    }

  }
  else if (mode == 1) {
    //Serial.println("mode=1");
    continueMicCapture();
    if (cap_blk_cnt >= CAP_BLOCK_SIZE) {
      stopMicCapture();
/*
      while (!out_end)
      {
        delay(100);
      }
      out_start = 0;
      Serial.println("endWave");
*/
      int level_max;

      level_max = check_capture_data();

#if defined(WAVE_COMBINED_USE)
      int wave_index;

      if (level_max < WAVE_FFT_LEVEL_MIN)
      {
          if (WAVE_FFT_LEVEL_MIN - level_max > WAVE_FFT_LEVEL_MIN / 2)
          {
            wave_data[wave_sel].in_gain += 10;
          }
          else if (WAVE_FFT_LEVEL_MIN - level_max > WAVE_FFT_LEVEL_MIN / 10)
          {
            wave_data[wave_sel].in_gain += 5;
          }
          else
          {
            wave_data[wave_sel].in_gain += 3;
          }
          if (wave_data[wave_sel].in_gain > WAVE_IN_GAIN_MAX)
            wave_data[wave_sel].in_gain = WAVE_IN_GAIN_MAX;
      }
      else if (level_max > WAVE_FFT_LEVEL_MAX)
      {
          if (level_max - WAVE_FFT_LEVEL_MAX > WAVE_FFT_LEVEL_MIN / 2)
          {
            wave_data[wave_sel].in_gain -= 10;
          }
          else if (level_max - WAVE_FFT_LEVEL_MAX > WAVE_FFT_LEVEL_MIN / 10)
          {
            wave_data[wave_sel].in_gain -= 5;
          }
          else
          {
            wave_data[wave_sel].in_gain -= 3;
          }
          if (wave_data[wave_sel].in_gain < WAVE_IN_GAIN_MIN)
            wave_data[wave_sel].in_gain = WAVE_IN_GAIN_MIN;
      }
      else
      {
        int out_vol_adjust[WAVE_INFO_CNT];
        int out_vol_adjust_sum = 0;
        volatile int out_vol_adjust_flag = 0;

        for (wave_index = 0; wave_index < WAVE_INFO_CNT; wave_index ++)
        {
          Serial.printf("wave_data[%d].level=%d\n", wave_index, wave_data[wave_index].level);
          out_vol_adjust[wave_index] = 0;
          if (wave_data[wave_index].level < WAVE_FFT_LEVEL_MIN)
          {
            out_vol_adjust_flag = 1;
            out_vol_adjust_sum += 5;
            out_vol_adjust[wave_index] = 5;
            if (WAVE_FFT_LEVEL_MIN - wave_data[wave_index].level > WAVE_FFT_LEVEL_MIN / 2)
            {
              out_vol_adjust_sum += 5;
              out_vol_adjust[wave_index] += 5;
            }
          }
          else if (wave_data[wave_index].level > WAVE_FFT_LEVEL_MAX)
          {
            out_vol_adjust_flag = 1;
            out_vol_adjust_sum -= 5;
            out_vol_adjust[wave_index] = -5;
            if (wave_data[wave_index].level - WAVE_FFT_LEVEL_MAX > WAVE_FFT_LEVEL_MIN / 2)
            {
              out_vol_adjust_sum -= 5;
              out_vol_adjust[wave_index] -= 5;
            }
          }
          Serial.printf("out_vol_adjust[%d]=%d\n", wave_index, out_vol_adjust[wave_index]);
        }
        Serial.printf("out_vol_adjust_sum=%d\n", out_vol_adjust_sum);
        if (out_vol_adjust_flag)
        {
          for (wave_index = 0; wave_index < WAVE_INFO_CNT; wave_index ++)
          {
            Serial.printf("out_vol_adjust[%d]=%d, wave_data[%d].out_vol=%d\n", wave_index, out_vol_adjust[wave_index], wave_index, wave_data[wave_index].out_vol);
            if ((out_vol_adjust[wave_index] > 0)
                &&
                (wave_data[wave_index].out_vol == WAVE_OUT_VOL_MAX))
              break;
            if ((out_vol_adjust[wave_index] < 0)
                &&
                (wave_data[wave_index].out_vol == WAVE_OUT_VOL_MIN))
              break;
          }
          if (wave_index != WAVE_INFO_CNT)
          {
            out_vol_adjust_flag = 0;
            if (out_vol_adjust_sum > 0)
            {
              wave_data[wave_sel].in_gain += 3;
              if (wave_data[wave_sel].in_gain > WAVE_IN_GAIN_MAX)
                wave_data[wave_sel].in_gain = WAVE_IN_GAIN_MAX;
            }
            else if (out_vol_adjust_sum <= 0)
            {
              wave_data[wave_sel].in_gain -= 3;
              if (wave_data[wave_sel].in_gain < WAVE_IN_GAIN_MIN)
                wave_data[wave_sel].in_gain = WAVE_IN_GAIN_MIN;
            }
          }
          else
          {
            for (wave_index = 0; wave_index < WAVE_INFO_CNT; wave_index ++)
            {
              wave_data[wave_index].out_vol += out_vol_adjust[wave_index];
              if (wave_data[wave_index].out_vol > WAVE_OUT_VOL_MAX)
                wave_data[wave_index].out_vol = WAVE_OUT_VOL_MAX;
              if (wave_data[wave_index].out_vol < WAVE_OUT_VOL_MIN)
                wave_data[wave_index].out_vol = WAVE_OUT_VOL_MIN;
              Serial.printf("wave_data[%d].out_vol=%d\n", wave_index, wave_data[wave_index].out_vol);
            }
          }
        }
        if (out_vol_adjust_flag)
        {
          make_combined_wave();
        }
      }
#else // defined(WAVE_COMBINED_USE)
      if (level_max < WAVE_FFT_LEVEL_MIN)
      {
        if (wave_data[wave_sel].out_vol == WAVE_OUT_VOL_MAX)
        {
          if (WAVE_FFT_LEVEL_MIN - level_max > WAVE_FFT_LEVEL_MIN / 2)
          {
            wave_data[wave_sel].in_gain += 10;
          }
          else if (WAVE_FFT_LEVEL_MIN - level_max > WAVE_FFT_LEVEL_MIN / 10)
          {
            wave_data[wave_sel].in_gain += 5;
          }
          else
          {
            wave_data[wave_sel].in_gain += 3;
          }
          if (wave_data[wave_sel].in_gain > WAVE_IN_GAIN_MAX)
            wave_data[wave_sel].in_gain = WAVE_IN_GAIN_MAX;
        }
        else
        {
          if (WAVE_FFT_LEVEL_MIN - level_max > WAVE_FFT_LEVEL_MIN / 2)
          {
            wave_data[wave_sel].out_vol += 10;
          }
          else if (WAVE_FFT_LEVEL_MIN - level_max > WAVE_FFT_LEVEL_MIN / 10)
          {
            wave_data[wave_sel].out_vol += 5;
          }
          else
          {
            wave_data[wave_sel].out_vol += 3;
          }
          if (wave_data[wave_sel].out_vol > WAVE_OUT_VOL_MAX)
            wave_data[wave_sel].out_vol = WAVE_OUT_VOL_MAX;
        }
      }
      else if (level_max > WAVE_FFT_LEVEL_MAX)
      {
        if (wave_data[wave_sel].out_vol == WAVE_OUT_VOL_MIN)
        {
          if (level_max - WAVE_FFT_LEVEL_MAX > WAVE_FFT_LEVEL_MIN / 2)
          {
            wave_data[wave_sel].in_gain -= 10;
          }
          else if (level_max - WAVE_FFT_LEVEL_MAX > WAVE_FFT_LEVEL_MIN / 10)
          {
            wave_data[wave_sel].in_gain -= 5;
          }
          else
          {
            wave_data[wave_sel].in_gain -= 3;
          }
          if (wave_data[wave_sel].in_gain < WAVE_IN_GAIN_MIN)
            wave_data[wave_sel].in_gain = WAVE_IN_GAIN_MIN;
        }
        else
        {
          if (level_max - WAVE_FFT_LEVEL_MAX > WAVE_FFT_LEVEL_MIN / 2)
          {
            wave_data[wave_sel].out_vol -= 10;
          }
          else if (level_max - WAVE_FFT_LEVEL_MAX > WAVE_FFT_LEVEL_MIN / 10)
          {
            wave_data[wave_sel].out_vol -= 5;
          }
          else
          {
            wave_data[wave_sel].out_vol -= 3;
          }
          if (wave_data[wave_sel].out_vol < WAVE_OUT_VOL_MIN)
            wave_data[wave_sel].out_vol = WAVE_OUT_VOL_MIN;
        }
      }
#endif // else defined(WAVE_COMBINED_USE)
/*
      Serial.printf("\nlevel_max=%d,in_gain=%d", level_max, wave_data[wave_sel].in_gain);
#if defined(WAVE_COMBINED_USE)
      for (wave_index = 0; wave_index < WAVE_INFO_CNT; wave_index ++)
      {
        Serial.printf(",out_vol[%d]=%d", wave_index, wave_data[wave_index].out_vol);
      }
#else
      Serial.printf(",out_vol=%d", wave_data[wave_sel].out_vol);
#endif
      Serial.printf("\n\n");
      //delay(1000);
*/
    }
  }
  else {
    //Serial.print("mode=");
    //Serial.println(mode);
  }
}

int check_capture_data()
{
  int i;
  int16_t buffer[FFT_BLOCK_CNT * AUDIO_BLOCK_SAMPLES];
  uint16_t peak2peak[CAP_TIME_MAX];
#if defined(WAVE_COMBINED_USE)
  uint16_t amplitude[WAVE_INFO_CNT][CAP_TIME_MAX * CAP_TIME_DIV];
  int min[WAVE_INFO_CNT], max[WAVE_INFO_CNT], avg[WAVE_INFO_CNT];
  uint16_t high[WAVE_INFO_CNT];
  int peaks[WAVE_INFO_CNT];
  int wave_index;
#else
  uint16_t amplitude[CAP_TIME_MAX * CAP_TIME_DIV];
  int min, max, avg;
  uint16_t high;
  int peaks;
#endif

  // Calculate peak to peak values of capruted buffer
  for (i = 0; i < CAP_TIME_MAX; i ++)
  {
    int pp_min, pp_max, pp_index;

    pp_min = 32767;
    pp_max = -32768;
    for (pp_index = 0; pp_index < AUDIO_SAMPLE_RATE_KHZ; pp_index ++)
    {
      int16_t pp_data;
      pp_data = cap_buf[i * AUDIO_SAMPLE_RATE_KHZ + pp_index];
      if (pp_data < pp_min) pp_min = pp_data;
      if (pp_data > pp_max) pp_max = pp_data;
    }
    peak2peak[i] = pp_max - pp_min + 1;
  }

  // Calcurate FFT for 1msec/CAP_TIME_DIV of capture buffer
  // And, Calcurate max min average of FFT amplitude 
#if defined(WAVE_COMBINED_USE)
  /*
  for (wave_index = 0; wave_index < WAVE_INFO_CNT; wave_index ++)
  {
    min[wave_index] = 65535;
    max[wave_index] = 0;
    avg[wave_index] = 0;
  }
  */
#else
  //min = 65535;
  //max = 0;
  //avg = 0;
#endif
  for (i = 0; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
  {
    int sample_index;

    for (sample_index = 0; sample_index < FFT_BLOCK_CNT * AUDIO_BLOCK_SAMPLES / AUDIO_SAMPLE_RATE_KHZ; sample_index ++)
    {
      memcpy16(buffer + sample_index * AUDIO_SAMPLE_RATE_KHZ, cap_buf + i * AUDIO_SAMPLE_RATE_KHZ / CAP_TIME_DIV, AUDIO_SAMPLE_RATE_KHZ);
    }
    //Serial.printf("sample_index(%d) * AUDIO_SAMPLE_RATE_KHZ = %d\n", sample_index, sample_index * AUDIO_SAMPLE_RATE_KHZ);
    memcpy16(buffer + sample_index * AUDIO_SAMPLE_RATE_KHZ, cap_buf + i * AUDIO_SAMPLE_RATE_KHZ / CAP_TIME_DIV, (FFT_BLOCK_CNT * AUDIO_BLOCK_SAMPLES) - (sample_index * AUDIO_SAMPLE_RATE_KHZ));

    // Start FFT from buffer
    bufFFT.update(buffer);

    double n;

#if defined(WAVE_COMBINED_USE)
    for (wave_index = 0; wave_index < WAVE_INFO_CNT; wave_index ++)
    {
      //n = bufFFT.read((wave_freq[wave_index] - 500) / FFT_FRQ_ACC, (wave_freq[wave_index] + 500)/ FFT_FRQ_ACC);
      n = bufFFT.read((double)(wave_freq[wave_index] - 250.) / FFT_FRQ_ACC, (double)(wave_freq[wave_index] + 250.) / FFT_FRQ_ACC);
      amplitude[wave_index][i] = (int)(n * 1000.);
      //avg[wave_index] += amplitude[wave_index][i];
      //if (amplitude[wave_index][i] < min[wave_index]) min[wave_index] = amplitude[wave_index][i];
      //if (amplitude[wave_index][i] > max[wave_index]) max[wave_index] = amplitude[wave_index][i];
    }
#else
    //n = bufFFT.read((wave_freq[wave_sel] - 500) / FFT_FRQ_ACC, (wave_freq[wave_sel] + 500)/ FFT_FRQ_ACC);
    n = bufFFT.read((double)(wave_freq[wave_sel] - 250.) / FFT_FRQ_ACC, (double)(wave_freq[wave_sel] + 250.) / FFT_FRQ_ACC);
    amplitude[i] = (int)(n * 1000.);
    //avg += amplitude[i];
    //if (amplitude[i] < min) min = amplitude[i];
    //if (amplitude[i] > max) max = amplitude[i];
#endif
  }
#if defined(WAVE_COMBINED_USE)
  /*
  for (wave_index = 0; wave_index < WAVE_INFO_CNT; wave_index ++)
  {
    avg[wave_index] = avg[wave_index] / CAP_TIME_MAX / CAP_TIME_DIV;
  }
  */
#else
  //avg = avg / CAP_TIME_MAX / CAP_TIME_DIV;
#endif

#if defined(WAVE_COMBINED_USE)
  for (wave_index = 0; wave_index < WAVE_INFO_CNT; wave_index ++)
  {
    // Calcurate max min average of FFT amplitude 
    //  at early 3msec approx. 50 cm
    min[wave_index] = 65535;
    max[wave_index] = 0;
    avg[wave_index] = 0;
    for (i = 0 ; i < (3 + 1) * CAP_TIME_DIV; i ++)
    {
      avg[wave_index] += amplitude[wave_index][i];
      if (amplitude[wave_index][i] < min[wave_index]) min[wave_index] = amplitude[wave_index][i];
      if (amplitude[wave_index][i] > max[wave_index]) max[wave_index] = amplitude[wave_index][i];
    }
    avg[wave_index] = avg[wave_index] / CAP_TIME_MAX / CAP_TIME_DIV;

    // Search first peak
    high[wave_index] = 0;
    for (i = 0; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
    {
      if (amplitude[wave_index][i] > max[wave_index] / 2)
      {
        if (amplitude[wave_index][i] <= high[wave_index])
        {
          break;
        }
        high[wave_index] = amplitude[wave_index][i];
        msec_index_first[wave_index] = i;
      }
      else
      {
        if (high[wave_index])
        {
          break;
        }
      }
    }

    // Calcurate max min average of FFT amplitude 
    //  after skip 3msec approx. 50 cm
    min[wave_index] = 65535;
    max[wave_index] = 0;
    avg[wave_index] = 0;
    for (i = msec_index_first[wave_index] + (WAVE_DURATION + 3) * CAP_TIME_DIV; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
    {
      avg[wave_index] += amplitude[wave_index][i];
      if (amplitude[wave_index][i] < min[wave_index]) min[wave_index] = amplitude[wave_index][i];
      if (amplitude[wave_index][i] > max[wave_index]) max[wave_index] = amplitude[wave_index][i];
    }
    avg[wave_index] = avg[wave_index] / CAP_TIME_MAX / CAP_TIME_DIV;

    // Calcurate peak count of FFT amplitude 
    peaks[wave_index] = 0;
    high[wave_index] = 0;
    //  after skip 3msec approx. 50 cm
    for (i = msec_index_first[wave_index] + (WAVE_DURATION + 3) * CAP_TIME_DIV; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
    //for (i = 0; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
    {
      if (amplitude[wave_index][i] > (max[wave_index] / 3))
      {
        if (amplitude[wave_index][i] <= high[wave_index])
        {
          peaks[wave_index] ++;
          high[wave_index] = 0;
        }
        else
        {
          high[wave_index] = amplitude[wave_index][i];
        }
      }
      else
      {
        if (high[wave_index])
        {
          peaks[wave_index] ++;
          high[wave_index] = 0;
        }
      }
    }

    // Search highest amplitude after skip 3msec approx. 50 cm
    high[wave_index] = 0;
    for (i = msec_index_first[wave_index] + (WAVE_DURATION + 3) * CAP_TIME_DIV; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
    {
      if (amplitude[wave_index][i] > high[wave_index])
      {
        high[wave_index] = amplitude[wave_index][i];
        msec_index_highest[wave_index] = i;
      }
    }

    if ((max[wave_index] - min[wave_index]) > WAVE_FFT_LEVEL_MIN
        &&
        (max[wave_index] - min[wave_index]) < WAVE_FFT_LEVEL_MAX
        && peaks[wave_index] < 30)
    {
      // t = m*2/SoS -> m = t*SoS/2, SoS = Speed_of_SOUND
      distance[wave_index] = SPEED_OF_SOUND_25 * (msec_index_highest[wave_index] - msec_index_first[wave_index]) / 2 / CAP_TIME_DIV / 10; 
    }
    else
    {
      distance[wave_index] = 0;
    }
  }
#else // defined(WAVE_COMBINED_USE)
  // Calcurate max min average of FFT amplitude 
  //  at early 3msec approx. 50 cm
  min = 65535;
  max = 0;
  avg = 0;
  for (i = 0 ; i < (3 + 1) * CAP_TIME_DIV; i ++)
  {
    avg += amplitude[i];
    if (amplitude[i] < min) min = amplitude[i];
    if (amplitude[i] > max) max = amplitude[i];
  }
  avg = avg / CAP_TIME_MAX / CAP_TIME_DIV;

  // Search first peak
  high = 0;
  for (i = 0; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
  {
    if (amplitude[i] > max / 2)
    {
      if (amplitude[i] <= high)
      {
        break;
      }
      high = amplitude[i];
      msec_index_first = i;
    }
    else
    {
      if (high)
      {
        break;
      }
    }
  }

  // Calcurate max min average of FFT amplitude 
  //  after skip 3msec approx. 50 cm
  min = 65535;
  max = 0;
  avg = 0;
  for (i = msec_index_first + (WAVE_DURATION + 3) * CAP_TIME_DIV; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
  {
    avg += amplitude[i];
    if (amplitude[i] < min) min = amplitude[i];
    if (amplitude[i] > max) max = amplitude[i];
  }
  avg = avg / CAP_TIME_MAX / CAP_TIME_DIV;

  // Calcurate peak count of FFT amplitude 
  peaks = 0;
  high = 0;
  //  after skip 3msec approx. 50 cm
  for (i = msec_index_first + (WAVE_DURATION + 3) * CAP_TIME_DIV; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
  //for (i = 0; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
  {
    if (amplitude[i] > (max / 3))
    {
      if (amplitude[i] <= high)
      {
        peaks ++;
        high = 0;
      }
      else
      {
        high = amplitude[i];
      }
    }
    else
    {
      if (high)
      {
        peaks ++;
        high = 0;
      }
    }
  }

  // Search highest amplitude after skip 3msec approx. 50 cm
  high = 0;
  for (i = msec_index_first + (WAVE_DURATION + 3) * CAP_TIME_DIV; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
  {
    if (amplitude[i] > high)
    {
      high = amplitude[i];
      msec_index_highest = i;
    }
  }

  if ((max - min) >= WAVE_FFT_LEVEL_MIN
      &&
      (max - min) <= WAVE_FFT_LEVEL_MAX
      &&
      peaks < 30)
  {
    // t = m*2/SoS -> m = t*SoS/2, SoS = Speed_of_SOUND
    distance = SPEED_OF_SOUND_25 * (msec_index_highest - msec_index_first) / 2 / CAP_TIME_DIV / 10; 
  }
  else
  {
    distance = 0;
  }
#endif // else defined(WAVE_COMBINED_USE)

#if 0
  // Plotte some of capture data
  int plot_size = FFT_BLOCK_CNT * AUDIO_BLOCK_SAMPLES * 2;
  for(i = 0; i < plot_size; i += plot_size / 500)
  {
    int j;
    int cap_avg = 0;
    for (j = 0; j < plot_size / 500; j ++)
    {
      cap_avg += cap_buf[i + j];
    }
    cap_avg = cap_avg / j;
    Serial.printf("32767 %d -32768\n", cap_avg);
    //Serial.printf("%d 32767 %d -32768\n",i , cap_buf[i + 128]);
    //Serial.printf("32767 %d -32768\n", cap_buf[i * 2 + 128]);
    //Serial.printf("32767 %d -32768\n", (cap_buf[i * 2 + 128] + cap_buf[i * 2 + 1 + 128]) / 2);
    //Serial.printf("%d\n", cap_buf[i]);
  }
  Serial.println();
  delay(1000);
#endif

#if 0
  // Plotte some of capture data and peak to peak
  int plot_size = FFT_BLOCK_CNT * AUDIO_BLOCK_SAMPLES * 20;
  for(i = 0; i < plot_size; i += plot_size / 500)
  {
    int j;
    int p2p_avg = 0;
    int cap_avg = 0;

    if (plot_size / 500 / AUDIO_SAMPLE_RATE_KHZ > 0)
    {
      for (j = 0; j < plot_size / 500 / AUDIO_SAMPLE_RATE_KHZ; j ++)
      {
        p2p_avg += peak2peak[i / AUDIO_SAMPLE_RATE_KHZ + j];
      }
      p2p_avg = p2p_avg / j;
    }
    else
    {
      p2p_avg = peak2peak[i / AUDIO_SAMPLE_RATE_KHZ];
    }
    p2p_avg -= 32768;

    for (j = 0; j < plot_size / 500; j ++)
    {
      cap_avg += cap_buf[i + j];
    }
    cap_avg = cap_avg / j;

    Serial.printf("32767 %d %d %d -32768\n", cap_avg, p2p_avg, wave_data[wave_sel].in_gain * 1000 - 32768);
  }
  Serial.println();
  //delay(1000);
#endif

#if 0
  // Plotte all of capture data
  for (i = 0; i < CAP_BLOCK_SIZE * AUDIO_BLOCK_SAMPLES ; i += (CAP_BLOCK_SIZE * AUDIO_BLOCK_SAMPLES / 500))
  {
    int j;
    int cap_avg = 0;
    for (j = 0; j < CAP_BLOCK_SIZE * AUDIO_BLOCK_SAMPLES / 500; j ++)
    {
      cap_avg += cap_buf[i + j];
    }
    cap_avg = cap_avg / (CAP_BLOCK_SIZE * AUDIO_BLOCK_SAMPLES / 500);
    Serial.printf("32767 %d -32768\n", cap_avg);
  }
  if (i < CAP_BLOCK_SIZE * AUDIO_BLOCK_SAMPLES)
  {
    int j;
    int cap_avg = 0;
    for (j = 0; j < (CAP_BLOCK_SIZE * AUDIO_BLOCK_SAMPLES) - i; j ++)
    {
      cap_avg += cap_buf[i + j];
    }
    cap_avg = cap_avg / j;
    Serial.printf("32767 %d -32768\n", cap_avg);
  }
  Serial.printf("\n");
  delay(2000);
#endif

#if 0
  // Plotte all of capture data and peak to peak
  for (i = 0; i < CAP_BLOCK_SIZE * AUDIO_BLOCK_SAMPLES ; i += (CAP_BLOCK_SIZE * AUDIO_BLOCK_SAMPLES / 500))
  {
    int j;
    int cap_avg = 0;
    int p2p_avg = 0;
    for (j = 0; j < CAP_BLOCK_SIZE * AUDIO_BLOCK_SAMPLES / 500 / AUDIO_SAMPLE_RATE_KHZ; j ++)
    {
      p2p_avg += peak2peak[i / AUDIO_SAMPLE_RATE_KHZ + j];
    }
    p2p_avg = p2p_avg / j;
    for (j = 0; j < CAP_BLOCK_SIZE * AUDIO_BLOCK_SAMPLES / 500; j ++)
    {
      cap_avg += cap_buf[i + j];
    }
    cap_avg = cap_avg / j;
    Serial.printf("-32768 %d %d 32767\n", cap_avg, p2p_avg);
  }
  if (i < CAP_BLOCK_SIZE * AUDIO_BLOCK_SAMPLES)
  {
    int j;
    int cap_avg = 0;
    int p2p_avg = 0;
    for (j = 0; j < ((CAP_BLOCK_SIZE * AUDIO_BLOCK_SAMPLES) - i) / AUDIO_SAMPLE_RATE_KHZ; j ++)
    {
      p2p_avg += peak2peak[i / AUDIO_SAMPLE_RATE_KHZ + j];
    }
    p2p_avg = p2p_avg / j;
    for (j = 0; j < (CAP_BLOCK_SIZE * AUDIO_BLOCK_SAMPLES) - i; j ++)
    {
      cap_avg += cap_buf[i + j];
    }
    cap_avg = cap_avg / j;
    Serial.printf("-32768 %d %d 32767\n", cap_avg, p2p_avg);
  }
  Serial.printf("\n");
  delay(2000);
#endif

#if 0
  // Plotte peak2peak
  for (i = 0; i < CAP_TIME_MAX; i += (CAP_TIME_MAX / 500))
  {
    int j;
    int pp_avg = 0;
    for (j = 0; j < CAP_TIME_MAX / 500; j ++)
    {
      pp_avg += peak2peak[i + j];
    }
    pp_avg = pp_avg / (CAP_TIME_MAX / 500);
    Serial.printf("0 %d %d %d\n", i, pp_avg, wave_data[wave_sel].in_gain);
  }
  if (i < CAP_TIME_MAX)
  {
    int j;
    int cap_avg = 0;
    int pp_avg = 0;
    for (j = 0; j < CAP_TIME_MAX - i; j ++)
    {
      pp_avg += peak2peak[i + j];
    }
    pp_avg = pp_avg / j;
    Serial.printf("0 %d %d %d\n", i, pp_avg, wave_data[wave_sel].in_gain);
  }
  Serial.printf("\n");
  //delay(2000);
#endif

#if 1
  #if defined(WAVE_COMBINED_USE)
    for (wave_index = 0; wave_index < WAVE_INFO_CNT; wave_index ++)
    {
      // Plotte amplitude
      for (i = 0; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
      {
        if (i >= 500)
        {
          break;
        }
        if ((msec_index_first[wave_index] != 0 && i >= msec_index_first[wave_index] && i < msec_index_first[wave_index] + (WAVE_DURATION + 3) * CAP_TIME_DIV)
            ||
            (msec_index_highest[wave_index] != 0 && i == msec_index_highest[wave_index]))
        {
          Serial.printf("%d %d %d %d %d %d\n", i, amplitude[wave_index][i], avg[wave_index], 0, distance[wave_index], wave_data[wave_index].in_gain * 10);
        }
        else
        {
          Serial.printf("%d %d %d %d %d %d\n", i, amplitude[wave_index][i], avg[wave_index], 189, distance[wave_index], wave_data[wave_index].in_gain * 10);
        }
      }
      //delay(2000);
    }
  #else
    // Plotte amplitude
    for (i = 0; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
    {
      if (i >= 500)
      {
        break;
      }
      if ((msec_index_first != 0 && i >= msec_index_first && i < msec_index_first + (WAVE_DURATION + 3) * CAP_TIME_DIV)
          ||
          (msec_index_highest != 0 && i == msec_index_highest))
      {
        Serial.printf("%d %d %d %d %d %d\n", i, amplitude[i], avg, 0, distance, wave_data[wave_sel].in_gain * 10);
      }
      else
      {
        Serial.printf("%d %d %d %d %d %d\n", i, amplitude[i], avg, 189, distance, wave_data[wave_sel].in_gain * 10);
      }
    }
  #endif
#endif

#if 1
  #if defined(WAVE_COMBINED_USE)
    Serial.printf("\n");
    for (wave_index = 0; wave_index < WAVE_INFO_CNT; wave_index ++)
    {
      Serial.printf("%d:frq=%d,out_vol=%d,min=%d,max=%d,avg=%d,peaks=%d,i_f=%d,i_h=%d,dis=%d\n", wave_index, wave_freq[wave_index], wave_data[wave_index].out_vol, min[wave_index], max[wave_index], avg[wave_index], peaks[wave_index], msec_index_first[wave_index], msec_index_highest[wave_index], distance[wave_index]);
    }
    Serial.printf("\n");
  #else
    Serial.printf("\n%d:frq=%d,in_gain=%d,out_vol=%d,min=%d,max=%d,avg=%d,peaks=%d,i_f=%d,i_h=%d,dis=%d\n\n", wave_sel, wave_freq[wave_sel], wave_data[wave_sel].in_gain, wave_data[wave_sel].out_vol, min, max, avg, peaks, msec_index_first, msec_index_highest, distance);
  #endif
#endif

#if defined(WAVE_COMBINED_USE)
  int maxmax = 0;
  for (wave_index = 0; wave_index < WAVE_INFO_CNT; wave_index ++)
  {
    wave_data[wave_index].level = max[wave_index];
    if (max[wave_index] > maxmax) maxmax = max[wave_index];
  }
  return maxmax;
#else
  wave_data[wave_sel].level = max;
  return max;
#endif
}

void startMicCapture() {
  //Serial.println("startMicCapture");
  queue1.begin();
  mode = 1;
}

void continueMicCapture() {
  int q_cnt = queue1.available();
  //Serial.print("continueMicCapture: q_cnt=");
  //Serial.println(q_cnt);
  if (q_cnt >= 1) {
    //Serial.print("continueMicCapture: q_cnt=");
    //Serial.println(q_cnt);
    for (int i = 0; i < q_cnt; i ++) {
      memcpy16(cap_buf + (cap_blk_cnt + i) * AUDIO_BLOCK_SAMPLES, queue1.readBuffer(), AUDIO_BLOCK_SAMPLES);
      queue1.freeBuffer();
    }
    cap_blk_cnt += q_cnt;
  }
}

void stopMicCapture() {
  int q_cnt;
  //Serial.println("stopMicCapture");
  queue1.end();
  q_cnt = queue1.available();
  //Serial.print("stopMicCapture: q_cnt=");
  //Serial.println(q_cnt);
  while (q_cnt > 1) {
    //Serial.print("stopMicCapture: q_cnt=");
    //Serial.println(q_cnt);
    queue1.freeBuffer();
    q_cnt = queue1.available();
  }
  mode = 0;
  cap_blk_cnt = 0;
}

/*
FASTRUN void timer0_callback() {
  analogWrite(DAC_PIN, waveTable[out_index]);  // write the selected waveform on DAC
  out_index++;
  if (out_index>=waveTableMax) {
    out_index=0;
    if (out_stop) {
      timer0.end();
      out_end = 1;
    }
  }
}

FASTRUN void timer1_callback() {
  timer1.end();
  out_stop = 1;
}
*/

#define PI2 (PI * 2.0)

int16_t *make_sine_wave(int target_KHz, int *cnt)
{
  int16_t *buf;
  int16_t tmp[1000]; // max. = 1000 / target_KHz
  int i;
  double val;

  Serial.printf("make_sine_wave(%d, ...)\n", target_KHz);

  if (target_KHz > 20 || target_KHz < 1)
  {
    Serial.printf("make_sine_wave: target_KHz(%d) error!\n", target_KHz);
    return NULL;
  }

  *cnt = 1000 / target_KHz; // usec
  //Serial.printf("*cnt=%d\n", *cnt);

  buf = (int16_t *)malloc(sizeof(int16_t) * (*cnt));
  if (!buf)
  {
    Serial.printf("make_sine_wave: malloc(%d) error!\n", sizeof(int16_t) * (*cnt));
    return NULL;
  }

  int min, max;
  min = 2047;
  max = -2048;
  for (i = 0; i < *cnt; i ++)
  {
    val = cos(PI2 * (double)i / (double)*cnt) * 4095.0/2.0 + 4095.0/2.0;
    buf[i] = (int16_t)val;
    if (buf[i] > max) max = buf[i];
    if (buf[i] < min) min = buf[i];
  }
  Serial.printf("min=%d, max=%d, vol=%d\n", min, max, max - min);

  /*
  for (i = 0; i < *cnt; i ++)
  {
    Serial.println(buf[i]);
    //Serial.print(",");
  }
  //Serial.println();
  delay(1000);
  */

  for (i = *cnt - 1; i >= 0; i --)
  {
    if (buf[i] < 2048)
    {
      i ++;
      break;
    }
  }

  /*
  Serial.print("i = ");
  Serial.println(i);
  */

  memcpy16(tmp, buf, i);
  memcpy16(buf, buf + i, *cnt - i + 1);
  memcpy16(buf + (*cnt - i), tmp, i);

  /*
  for (i = 0; i < *cnt; i ++)
  {
    Serial.println(buf[i]);
    //Serial.print(",");
  }
  //Serial.println();
  delay(3000);
  */

  return buf;
}

#if defined(WAVE_COMBINED_USE)
void make_combined_wave()
{
  int i, j;
  int32_t min, max;
  int32_t val;
  int32_t vol;
  int32_t mid;

  // Make combined wave data
  wave_data[WAVE_INFO_CNT].cnt = WAVE_DURATION;
  wave_data[WAVE_INFO_CNT].in_gain = WAVW_IN_GAIN_DEFAULT;
  wave_data[WAVE_INFO_CNT].out_vol = 100;
  wave_data[WAVE_INFO_CNT].size = 1000; // 1000 * 1usec = 1msec
  wave_data[WAVE_INFO_CNT].buf = (int16_t *)malloc(sizeof(int16_t) * 1000);

  for (j = 0; j < WAVE_INFO_CNT; j ++)
  {
    Serial.printf("make_combined_wave: out_vol[%d]=%d\n", j, wave_data[j].out_vol);
  }

  min = 2047;
  max = -2048;
  for (i = 0; i < wave_data[WAVE_INFO_CNT].size; i ++)
  {
    val = 0;
    for (j = 0; j < WAVE_INFO_CNT; j ++)
    {
      val += (int32_t)((((double)wave_data[j].buf[i % wave_data[j].size] - 2048.) * (double)wave_data[j].out_vol / 100.) + 2048.);
    }
    val = val / WAVE_INFO_CNT;
    wave_data[WAVE_INFO_CNT].buf[i] = (int16_t)val;
    if (val > max) max = val;
    if (val < min) min = val;
  }
  vol = max - min;
  mid = vol / 2 + min;
  Serial.printf("make_combined_wave: init min=%d(%d), max=%d(%d), vol=%d, mid=%d\n", min, (int)(min - mid), max, (int)(max - mid), vol, mid);

  min = 2047;
  max = -2048;
  for (i = 0; i < wave_data[WAVE_INFO_CNT].size; i ++)
  {
    wave_data[WAVE_INFO_CNT].buf[i] = (int16_t)((((double)wave_data[WAVE_INFO_CNT].buf[i] - (double)mid) * 4095. / (double)vol) + 2048.);
    //wave_data[WAVE_INFO_CNT].buf[i] = (int16_t)(((wave_data[WAVE_INFO_CNT].buf[i] - mid) * 4095 / vol) + 2048);
    if (wave_data[WAVE_INFO_CNT].buf[i] > max) max = wave_data[WAVE_INFO_CNT].buf[i];
    if (wave_data[WAVE_INFO_CNT].buf[i] < min) min = wave_data[WAVE_INFO_CNT].buf[i];
  }
  Serial.printf("make_combined_wave: final min=%d(%d), max=%d(%d), vol=%d\n", min, min - 2048, max, max - 2048, max - min);
}
#else
void make_active_wave()
{
  int i;
  int32_t min, max;
  int32_t val;
  int32_t vol;
  int32_t mid;

  // Make active wave data
  wave_data_act.cnt = wave_data[wave_sel].cnt;
  wave_data_act.in_gain = wave_data[wave_sel].in_gain;
  wave_data_act.out_vol = wave_data[wave_sel].out_vol;
  wave_data_act.size = wave_data[wave_sel].size;
  if (wave_data_act.buf) free(wave_data_act.buf);
  wave_data_act.buf = (int16_t *)malloc(sizeof(int16_t) * wave_data_act.size);

  //Serial.printf("make_active_wave: out_vol[%d]=%d\n", wave_sel, wave_data[wave_sel].out_vol);

  min = 2047;
  max = -2048;
  for (i = 0; i < wave_data[wave_sel].size; i ++)
  {
    val = (int32_t)((((double)wave_data[wave_sel].buf[i] - 2048.) * (double)wave_data[wave_sel].out_vol / 100.) + 2048.);
    wave_data_act.buf[i] = (int16_t)val;
    if (val > max) max = val;
    if (val < min) min = val;
  }
  vol = max - min;
  mid = vol / 2 + min;
  //Serial.printf("make_active_wave: min=%d(%d), max=%d(%d), vol=%d, mid=%d\n", min, (int)(min - mid), max, (int)(max - mid), vol, mid);
}
#endif
