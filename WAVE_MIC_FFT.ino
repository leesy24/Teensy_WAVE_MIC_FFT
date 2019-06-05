#include <DMAChannel.h>
#include <Audio.h>

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

#define CAP_BLOCK_SIZE ((AUDIO_SAMPLE_RATE_KHZ * CAP_TIME_MAX / AUDIO_BLOCK_SAMPLES) + 1) // approx. 263

#define WAVE_DURATION 1 // unit is msec

#define WAVE_INFO_CNT 4

int wave_freq[WAVE_INFO_CNT] =
{
  20000,
  10000,
  5000,
  2000
};

typedef struct {
  int cnt;
  int size;
  int16_t *buf;
  int gain;
} wave_info_t;

wave_info_t wave_data[WAVE_INFO_CNT];


volatile int wave_cnt = 0;
volatile int wave_end = 0;
int wave_cnt_max;

//int wave_sel = 0;
int wave_sel = 2;

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

#define AUDIO_INPUT_GAIN_MAX 100
//int audio_input_gain_default = 0;
int audio_input_gain_default = 35;
//int audio_input_gain_default = 100;

int16_t cap_buf[CAP_BLOCK_SIZE * AUDIO_BLOCK_SAMPLES];

int distance = 0; // unit is centi-meter
int msec_index_first = 0;
int msec_index_highest = 0;

// Remember which mode we're doing
int mode = 0;  // 0=stopped, 1=recording

// Setup DAC0
void dac_setup() {
  // setup DAC 
  SIM_SCGC2 |= SIM_SCGC2_DAC0;
  DAC0_C0 = DAC_C0_DACEN; // 1.2V VDDA is DACREF_2
  DAC0_C0 |= DAC_C0_DACRFS; // 3.3V
  
  // slowly ramp up to DC voltage, approx 1/4 second
  for (int16_t i=0; i<=2048; i+=8) {
    *(int16_t *)&(DAC0_DAT0L) = i;
    delay(1);
  }
}

// Setup PDB freq at n usec
void pdb_setup(int usec) {
  uint32_t mod = ((double)F_BUS / 1000000.) * (double)usec;

  Serial.printf("usec = %d\n", usec);
  Serial.printf("F_BUS = %d\n", F_BUS);
  Serial.printf("(F_BUS / 1000000) * usec = %lf\n", ((double)F_BUS / 1000000.) * (double)usec);

  SIM_SCGC6 |= SIM_SCGC6_PDB; // enable PDB clock

  PDB0_MOD = (uint16_t)(mod-1);  
  PDB0_IDLY = 0;
  PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
  PDB0_SC = PDB_CONFIG | PDB_SC_SWTRIG;
  PDB0_CH0C1 = 0x0101;
}

// Setup & start DMA for DAC transfer - ref sin 
void wave_dma_start(int16_t *lut, int lut_size) {
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


void setup()
{
  int i;

  for (i = 0; i < 5; i ++)
  {
    Serial.println("wait...");
    delay(1000);
  }

  // Make wave informations
  for (i = 0; i < WAVE_INFO_CNT; i ++)
  {
    wave_data[i].cnt = wave_freq[i] / 1000 * WAVE_DURATION;
    wave_data[i].buf = make_sine_wave(wave_freq[i] / 1000, &wave_data[i].size);
    wave_data[i].gain = audio_input_gain_default;
  }

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
  //sgtl5000_1.micGain(wave_data[wave_sel].gain);
  //sgtl5000_1.micGain(0);

  //bufFFT.windowFunction(AudioWindowHanning1024);
}

FASTRUN void isr_button_push()
{
  wave_sel ++;
  if (wave_sel >= WAVE_INFO_CNT)
  {
    wave_sel = 0;
  }
  //Serial.print("Button pushed!");
  //Serial.println(wave_sel);
}

void loop() {
  if (mode == 0) {
    //Serial.println("mode=0");
    sgtl5000_1.micGain(wave_data[wave_sel].gain);

    wave_end = 0;
    wave_cnt = 0;
    wave_cnt_max = wave_data[wave_sel].cnt;
    wave_dma_start(wave_data[wave_sel].buf, wave_data[wave_sel].size);
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
      int level;
      level = check_capture_data();

      if (level < 500)
      {
          wave_data[wave_sel].gain += 5;
          if (500 - level > 250)
          {
            wave_data[wave_sel].gain += 5;
          }
          if (wave_data[wave_sel].gain >= AUDIO_INPUT_GAIN_MAX)
            wave_data[wave_sel].gain = AUDIO_INPUT_GAIN_MAX;
          sgtl5000_1.micGain(wave_data[wave_sel].gain);
      }
      else if (level > 1000)
      {
          wave_data[wave_sel].gain -= 5;
          if (level - 1000 > 250)
          {
            wave_data[wave_sel].gain -= 5;
          }
          if (wave_data[wave_sel].gain < 0)
            wave_data[wave_sel].gain = 0;
          sgtl5000_1.micGain(wave_data[wave_sel].gain);
      }
      Serial.printf("\nlvl=%d,dis=%d,gain=%d\n\n", level, distance, wave_data[wave_sel].gain);
      delay(1000);
    }
  }
  else {
    //Serial.print("mode=");
    //Serial.println(mode);
  }
}

#define CAP_TIME_DIV 4

int check_capture_data()
{
  int i;
  int16_t buffer[FFT_BLOCK_CNT * AUDIO_BLOCK_SAMPLES];
  uint16_t amplitude[CAP_TIME_MAX * CAP_TIME_DIV];
  int sample_index;

  /*
  for(sample_index = 0; sample_index < (((FFT_BLOCK_CNT * AUDIO_BLOCK_SAMPLES) < 500) ? (FFT_BLOCK_CNT * AUDIO_BLOCK_SAMPLES) : 500) ; sample_index ++)
  {
    //Serial.printf("%d 32767 %d -32768\n",sample_index , cap_buf[sample_index + 128]);
    Serial.printf("32767 %d -32768\n", (cap_buf[sample_index * 2 + 128] + cap_buf[sample_index * 2 + 1 + 128]) / 2);
    //Serial.printf("%d\n", cap_buf[sample_index]);
  }
  Serial.println();
  delay(1000);
  */

  for (i = 0; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
  {
    for (sample_index = 0; sample_index < FFT_BLOCK_CNT * AUDIO_BLOCK_SAMPLES / AUDIO_SAMPLE_RATE_KHZ; sample_index ++)
    {
      memcpy(buffer + sample_index * AUDIO_SAMPLE_RATE_KHZ, cap_buf + i * AUDIO_SAMPLE_RATE_KHZ / CAP_TIME_DIV, AUDIO_SAMPLE_RATE_KHZ * 2);
    }
    //Serial.printf("sample_index(%d) * AUDIO_SAMPLE_RATE_KHZ = %d\n", sample_index, sample_index * AUDIO_SAMPLE_RATE_KHZ);
    memcpy(buffer + sample_index * AUDIO_SAMPLE_RATE_KHZ, cap_buf + i * AUDIO_SAMPLE_RATE_KHZ / CAP_TIME_DIV, ((FFT_BLOCK_CNT * AUDIO_BLOCK_SAMPLES) - (sample_index * AUDIO_SAMPLE_RATE_KHZ)) * 2);

    // Start FFT from buffer
    bufFFT.update(buffer);

    double n;

    //n = bufFFT.read((wave_freq[wave_sel] - 500) / FFT_FRQ_ACC, (wave_freq[wave_sel] + 500)/ FFT_FRQ_ACC);
    n = bufFFT.read((double)(wave_freq[wave_sel] - 250) / FFT_FRQ_ACC, (double)(wave_freq[wave_sel] + 250) / FFT_FRQ_ACC);
    amplitude[i] = (int)(n * 1000);
  }

  // Calcurate max min average of FFT amplitude 
  int min, max, avg;
  min = 65536;
  max = 0;
  avg = 0;
  for (i = 0; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
  {
    avg += amplitude[i];
    if (amplitude[i] < min) min = amplitude[i];
    if (amplitude[i] > max) max = amplitude[i];
  }
  avg = avg / CAP_TIME_MAX / CAP_TIME_DIV;

  uint16_t high;

  // Calcurate peak count of FFT amplitude 
  int peaks;
  peaks = 0;
  high = 0;
  for (i = 0; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
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

  // Search first peak
  high = 0;
  for (i = 0; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
  {
    if (amplitude[i] > avg * 2)
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

  if ((max - min) > 500 && peaks < 20)
  {
    // SoS = 346.45 m/s @ 25`C, t = m*2/SoS, m = t*SoS/2
    distance = 346.45 * (msec_index_highest - msec_index_first) / 2 / CAP_TIME_DIV / 10; 
  }
  else
  {
    distance = 0;
  }

  // Plotte
  for (i = 0; i < CAP_TIME_MAX * CAP_TIME_DIV; i ++)
  {
    if (i >= 500)
    {
      break;
    }
    if ((msec_index_first != 0 && i == msec_index_first)
        ||
        (msec_index_highest != 0 && i == msec_index_highest))
    {
      Serial.printf("%d %d %d %d %d\n", i, amplitude[i], avg, 0, 189);
    }
    else
    {
      Serial.printf("%d %d %d %d %d\n", i, amplitude[i], avg, distance, 189);
    }
  }

  Serial.printf("\nmin=%d,max=%d,avg=%d,peaks=%d,i_f=%d,i_h=%d,dis=%d\n\n", min, max, avg, peaks, msec_index_first, msec_index_highest, distance);

  return max;
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
      memcpy(cap_buf + (cap_blk_cnt + i) * AUDIO_BLOCK_SAMPLES, queue1.readBuffer(), AUDIO_BLOCK_SAMPLES * 2);
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
  int16_t *tmp;
  int i;
  double val;

  Serial.printf("make_sine_wave(%d, ...)\n", target_KHz);

  if (target_KHz > 20 || target_KHz < 1)
  {
    Serial.printf("make_sine_wave: targer_KHz(%d) error!\n", target_KHz);
    return NULL;
  }

  *cnt = 1000 / target_KHz; // usec
  Serial.print("*cnt=");
  Serial.println(*cnt);

  buf = (int16_t *)malloc(sizeof(int16_t) * (*cnt));

  for (i = 0; i < *cnt; i ++)
  {
    val = cos(PI2 * (double)i / (double)*cnt) * 4095.0/2.0 + 4095.0/2.0;
    buf[i] = (int16_t)val;
  }

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

  tmp = (int16_t *)malloc(sizeof(int16_t) * i);
  memcpy(tmp, buf, sizeof(int16_t) * i);
  memcpy(buf, buf + i, sizeof(int16_t) * (*cnt - i + 1));
  memcpy(buf + (*cnt - i), tmp, sizeof(int16_t) * i);
  free(tmp);

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
