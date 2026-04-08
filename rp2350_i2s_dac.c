#include <math.h>
#include <complex.h>

#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

/* promise compiler there is no such thing as inf/nan in complex numbers so that
 multiplying them can be done purely in hardware */
#pragma GCC optimize "-fcx-limited-range"

#include "rp2350_i2s_out_16bit_2ch.pio.h"
#define PIO_CLOCK_PER_BIT 2
#define BIT_PER_SAMPLE 16

void yield(void) {
    /* we could do context switching here for cooperative multitasking if we wanted */
    __dsb();
    __wfe();
}

#define BUFFER_WRAP_BITS 13
#define BYTES_PER_CHUNK 4096
#define CHANNELS 2

#define SAMPLES_PER_CHUNK (BYTES_PER_CHUNK / (CHANNELS * sizeof(int16_t)))
__attribute((aligned(sizeof(int16_t) * 2 * SAMPLES_PER_CHUNK * CHANNELS)))
static int16_t buffer[2][SAMPLES_PER_CHUNK * CHANNELS];
_Static_assert(1U << BUFFER_WRAP_BITS == sizeof(buffer), "wtf");

static float cmagsquaredf(const float complex x) {
    return crealf(x) * crealf(x) + cimagf(x) * cimagf(x);
}

static uint64_t xorshift64star(void) {
    /* marsaglia et al., yields 64 bits, most significant are most random */
    static uint64_t x = 1; /* must be nonzero */
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    return x * 0x2545F4914F6CDD1DULL;
}

static float frand_minus_frand(void) {
    /* generate 64 random bits, of which we will use the most significant 46, in two groups of 23 */
    const uint64_t bits = xorshift64star();

    /* generate two random numbers each uniformly distributed on [1.0f, 2.0f) */
    const union { uint32_t u; float f; } x = { .u = 0x3F800000U | ((bits >> 41) & 0x7FFFFFU) };
    const union { uint32_t u; float f; } y = { .u = 0x3F800000U | ((bits >> 18) & 0x7FFFFFU) };

    /* and subtract them, yielding a triangular distribution on (-1.0f, +1.0f) */
    return x.f - y.f;
}

int main() {
    /* enable sevonpend, so that we don't need nearly-empty ISRs */
    scb_hw->scr |= M33_SCR_SEVONPEND_BITS;

    set_sys_clock_48mhz();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    PIO pio = pio0;
    const unsigned sm = pio_claim_unused_sm(pio, true);
    const unsigned offset = pio_add_program(pio, &i2s_out_16bit_2ch_program);

    /* gp5 for bck, gp6 for wsel, gp7 for din */
    pio_gpio_init(pio, 5);
    pio_gpio_init(pio, 6);
    pio_gpio_init(pio, 7);

    pio_sm_set_consecutive_pindirs(pio, sm, 5, 3, true);

    const unsigned long sys_cycle_256ths_per_pio_clock = 4096;

    pio_sm_config sm_config = i2s_out_16bit_2ch_program_get_default_config(offset);
    sm_config_set_sideset_pins(&sm_config, 5);
    sm_config_set_out_pins(&sm_config, 7, 1);
    sm_config_set_clkdiv_int_frac8(&sm_config, sys_cycle_256ths_per_pio_clock / 256U, sys_cycle_256ths_per_pio_clock % 256U);

    /* restore original config */
    pio_sm_init(pio, sm, offset, &sm_config);

#define IDMA_I2S 0
    dma_channel_claim(IDMA_I2S);
    dma_channel_config cfg = dma_channel_get_default_config(IDMA_I2S);
    channel_config_set_dreq(&cfg, pio_get_dreq(pio, sm, true));
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_ring(&cfg, false, BUFFER_WRAP_BITS);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);

    dma_channel_configure(IDMA_I2S,
                          &cfg,
                          &pio->txf[sm],
                          &buffer[0],
                          SAMPLES_PER_CHUNK | (1U << 28),
                          false);

    /* enable interrupt for dma, but leave it disabled in nvic */
    dma_channel_acknowledge_irq0(IDMA_I2S);
    dma_channel_set_irq0_enabled(IDMA_I2S, true);
    __dsb();
    irq_set_enabled(DMA_IRQ_0, false);
    dma_channel_start(IDMA_I2S);

    const float sample_rate = 48e6f * 256.0f / (sys_cycle_256ths_per_pio_clock * PIO_CLOCK_PER_BIT * BIT_PER_SAMPLE * CHANNELS);

    /* this can be any value between dc and fs/2, does not need to be an integer */
    const float tone_frequency = 900.0f;

    /* multiplier relative to full scale */
    const float tone_amplitude = 0.1f;

    const float complex advance = cexpf(I * 2.0f * (float)M_PI * tone_frequency / sample_rate);

    /* this will evolve along the unit circle */
    float complex carrier = -1.0f;

    for (size_t ichunk = 0;; ichunk++) {
        int16_t * const dst = buffer[ichunk % 2];
        for (size_t ival = 0; ival < SAMPLES_PER_CHUNK; ival++) {
            const float sample = crealf(carrier) * tone_amplitude;

            for (size_t ic = 0; ic < CHANNELS; ic++)
            /* map [-1.0, 1.0] to [0, TOP] with triangular pdf dither */
                /* TODO: validate full scale and no clipping */
                dst[ic + CHANNELS * ival] = (sample * 32766) + 0.5f + frand_minus_frand();

            /* rotate complex sinusoid at the desired frequency */
            carrier *= advance;

            /* renormalize carrier to unity */
            carrier = carrier * (3.0f - cmagsquaredf(carrier)) / 2.0f;
        }

        /* if we just filled the first chunk and have not enabled the pio yet, enable it,
         and immediately fill the next chunk without waiting for an interrupt */
        if (0 == ichunk && !(pio->ctrl & (1U << sm)))
            pio_sm_set_enabled(pio, sm, true);
        else {
            /* run other tasks or low power sleep until next dma interrupt */
            while (!(dma_hw->intr & 1U << IDMA_I2S)) yield();

            /* acknowledge and clear the interrupt in both dma and nvic */
            dma_hw->ints0 = 1U << IDMA_I2S;
            irq_clear(DMA_IRQ_0);
        }
    }
}
