//ECE 301 Final Project Ted Fritzenmeier, Alex Zimmermann, Evan Gould, Carlos Juarez, Wesley Sauder
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "uart.h"


//Fs = 360 Hz
// LPF: H(z) = (1 - z^-6)^2 / (1 - z^-1)^2
//   y(n) = 2y(n-1) - y(n-2) + x(n) - 2x(n-6) + x(n-12)

// HPF: y(n) = y(n-1) - (1/32)x(n) + x(n-16) - x(n-17) + (1/32)x(n-32)

// Derivative (5-point):
//  y(n) = (1/8)[2x(n) + x(n-1) - x(n-3) - 2x(n-4)]
 
// MWI: 150 ms rectangular window = 54 samples at 360 Hz
// Refractory: 200 ms = 72 samples
// Bootstrap:    2 s  = 720 samples

#define FS               360
#define REFRACTORY_SAMP  72
#define BOOT_SAMPLES     720

//parameters

// Low-pass filter
#define LPF_N   13
static int32_t lpf_x[LPF_N];
static int32_t lpf_y1 = 0, lpf_y2 = 0;
static uint8_t lpf_head = 0;

// High-pass filter
#define HPF_N   33
static int32_t hpf_x[HPF_N];
static int32_t hpf_y = 0;
static uint8_t hpf_head = 0;

// Derivative
static int32_t drv_x[5];
static uint8_t drv_head = 0;

// Moving-window integrator
#define MWI_N   54
static int32_t mwi_x[MWI_N];
static int32_t mwi_sum = 0;
static uint8_t mwi_head = 0;

//Low pass filter
static int32_t lpf(int32_t x)
{
    lpf_x[lpf_head] = x;

    uint8_t i6  = (uint8_t)((lpf_head + LPF_N - 6)  % LPF_N);
    uint8_t i12 = (uint8_t)((lpf_head + LPF_N - 12) % LPF_N);

    int32_t y = (lpf_y1 << 1) - lpf_y2
              + lpf_x[lpf_head]
              - (lpf_x[i6] << 1)
              + lpf_x[i12];

    lpf_y2   = lpf_y1;
    lpf_y1   = y;
    lpf_head = (uint8_t)((lpf_head + 1) % LPF_N);

    return y >> 5;
}

// High-pass filter
static int32_t hpf(int32_t x)
{
    hpf_x[hpf_head] = x;

    uint8_t i16 = (uint8_t)((hpf_head + HPF_N - 16) % HPF_N);
    uint8_t i17 = (uint8_t)((hpf_head + HPF_N - 17) % HPF_N);
    uint8_t i32 = (uint8_t)((hpf_head + HPF_N - 32) % HPF_N);

    hpf_y = hpf_y
          - (x              >> 5)
          + hpf_x[i16]
          - hpf_x[i17]
          + (hpf_x[i32]    >> 5);

    hpf_head = (uint8_t)((hpf_head + 1) % HPF_N);

    return hpf_y;
}

//derivative
static int32_t derivative(int32_t x)
{
    drv_x[drv_head] = x;

    uint8_t n1 = (uint8_t)((drv_head + 4) % 5);   /* x(n-1) */
    uint8_t n3 = (uint8_t)((drv_head + 2) % 5);   /* x(n-3) */
    uint8_t n4 = (uint8_t)((drv_head + 1) % 5);   /* x(n-4) */

    int32_t y = ( (drv_x[drv_head] << 1)
                +  drv_x[n1]
                -  drv_x[n3]
                - (drv_x[n4] << 1) ) >> 3;

    drv_head = (uint8_t)((drv_head + 1) % 5);
    return y;
}

//Squaring: limit input to +/-1000 to avoid overflow, then square
static int32_t squarer(int32_t x)
{
    if (x >  1000) x =  1000;
    if (x < -1000) x = -1000;
    return x * x;
}

// Moving-window integrator (150 ms = 54 samples)
static int32_t mwi(int32_t x)
{
    mwi_sum          -= mwi_x[mwi_head];
    mwi_x[mwi_head]   = x;
    mwi_sum          += x;
    mwi_head          = (uint8_t)((mwi_head + 1) % MWI_N);
    return mwi_sum / MWI_N;
}

// Adaptive thresholding and QRS detection state
static int32_t  spki        = 0;
static int32_t  npki        = 0;
static int32_t  thresh1     = 0;
static bool     initialized = false;

static int32_t  boot_max    = 0;
static uint16_t boot_count  = 0;

static uint16_t refractory  = 0;

static int32_t  prev_val    = 0;
static bool     is_rising   = false;

//process a single sample through the pipeline and return true if a QRS complex is detected
static bool process_sample(int16_t raw)
{
    int32_t s1 = lpf((int32_t)raw);
    int32_t s2 = hpf(s1);
    int32_t s3 = derivative(s2);
    int32_t s4 = squarer(s3);
    int32_t s5 = mwi(s4);

    // During the initial bootstrapping phase, we just look for the maximum value to set our initial thresholds. (2s)
    if (!initialized)
    {
        if (s5 > boot_max) boot_max = s5;
        boot_count++;

        if (boot_count >= BOOT_SAMPLES)
        {
            spki    = boot_max;
            npki    = boot_max >> 2;                    // noise = 25% of max    
            thresh1 = npki + ((spki - npki) >> 2);     // 25% between N and S   

            initialized = true;
            refractory  = REFRACTORY_SAMP;
            prev_val    = 0;
            is_rising   = false;

            hpf_y = 0;
        }
        return false;
    }

    if (refractory > 0)
    {
        refractory--;
        prev_val = s5;
        return false;
    }

    // Peak detection with adaptive thresholding
    bool qrs = false;

    if (s5 > prev_val)
    {
        is_rising = true;
    }
    else if (is_rising && (s5 < prev_val))
    {
        if (prev_val >= thresh1)
        {
            qrs        = true;
            spki       = spki + ((prev_val - spki) >> 3);
            refractory = REFRACTORY_SAMP;
        }
        else
        {
            npki = npki + ((prev_val - npki) >> 3);
        }
        is_rising = false;
    }

    prev_val = s5;

    // Update adaptive threshold
    thresh1 = npki + ((spki - npki) >> 2);
    if (thresh1 < 1) thresh1 = 1;

    return qrs;
}


int main(void)
{
    uart_init();

    memset(lpf_x, 0, sizeof(lpf_x));
    memset(hpf_x, 0, sizeof(hpf_x));
    memset(drv_x, 0, sizeof(drv_x));
    memset(mwi_x, 0, sizeof(mwi_x));

    lpf_y1 = 0;  lpf_y2 = 0;
    hpf_y  = 0;

    // Tell MATLAB the device is ready to receive samples
    uart_puts("READY\r\n");

    char    rx_buf[12];
    uint8_t rx_idx = 0;

    for (;;)
    {
        if (!uart_rx_ready()) continue;

        // Read characters until we get a newline, then process the line as a sample value
        char c = uart_getc();

        if (c == '\n')
        {
            if (rx_idx > 0)
            {
                rx_buf[rx_idx] = '\0';
                rx_idx = 0;

                int16_t sample = (int16_t)atoi(rx_buf);
                bool    qrs    = process_sample(sample);

                uart_puts(qrs ? "Q\r\n" : "N\r\n");
            }
        }
        else if (c != '\r' && rx_idx < (uint8_t)(sizeof(rx_buf) - 1))
        {
            rx_buf[rx_idx++] = c;
        }
    }

    return 0;
}