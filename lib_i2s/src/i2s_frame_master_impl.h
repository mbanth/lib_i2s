// Copyright (c) 2016-2018, XMOS Ltd, All rights reserved
#if defined(__XS2A__)

#include <xs1.h>
#include <xclib.h>
#include "i2s.h"
#include "xassert.h"

#define NUM_IO_PER_I2S_FRAME    8   //The number of input/outputs to 4b port over one I2S frame
#define NUM_CHANS_PER_WIRE      4
#define I2S_WIRES               1

//-----------------------------------------------------------------------------
// Description: Helper function to zip up output samples to 4b buffered ports
//     Returns: none
//      Inputs: samplesOut - array of input PCM samples
//      Outputs: zipped_outs - array of 32b values to be output to buffered port
//-----------------------------------------------------------------------------
#pragma unsafe arrays
static inline void zip_samplesOut_to_output_buffer_left(unsigned samplesOut[], unsigned zipped_outs[NUM_IO_PER_I2S_FRAME]){
    unsigned long long temp64_outs[2];

    /* Packs of 1 bit */
    temp64_outs[0] = zip(samplesOut[0], samplesOut[2], 0);      // Interleave L1 & L2
    temp64_outs[1] = zip(samplesOut[4], samplesOut[6], 0);      // Interleave L3 & L4
    zipped_outs[0] = (unsigned int)(temp64_outs[0] >> 32);      // MSB of [L1,L2] into 32bits
    zipped_outs[1] = (unsigned int)(temp64_outs[0] >>  0);      // LSB of [L1,L2] into 32bits
    zipped_outs[2] = (unsigned int)(temp64_outs[1] >> 32);      // MSB of [L3,L4] into 32bits
    zipped_outs[3] = (unsigned int)(temp64_outs[1] >>  0);      // LSB of [L3,L4] into 32bits

    /* Packs of 2 bits */
    temp64_outs[0] = zip(zipped_outs[0], zipped_outs[2], 1);    // Interleave MSB's of [L1,L2] with [L3,L4]
    temp64_outs[1] = zip(zipped_outs[1], zipped_outs[3], 1);    // Interleave LSB's of [L1,L2] with [L3,L4]
    zipped_outs[0] = (unsigned int)(temp64_outs[0] >> 32);      // MSBs of [L1,L2,L3,L4] into 32bits
    zipped_outs[1] = (unsigned int)(temp64_outs[0]      );      // MSBs of [L1,L2,L3,L4] into 32bits
    zipped_outs[2] = (unsigned int)(temp64_outs[1] >> 32);      // LSBs of [L1,L2,L3,L4] into 32bits
    zipped_outs[3] = (unsigned int)(temp64_outs[1]      );      // LSBs of [L1,L2,L3,L4] into 32bits
}

//-----------------------------------------------------------------------------
// Description: Helper function to zip up output samples to 4b buffered ports
//     Returns: none
//      Inputs: samplesOut - array of input PCM samples
//      Outputs: zipped_outs - array of 32b values to be output to buffered port
//-----------------------------------------------------------------------------
#pragma unsafe arrays
static inline void zip_samplesOut_to_output_buffer_right(unsigned samplesOut[], unsigned zipped_outs[NUM_IO_PER_I2S_FRAME]){
    unsigned long long temp64_outs[2];

    /* Packs of 1 bit */
    temp64_outs[0] = zip(samplesOut[1], samplesOut[3], 0);      // Interleave R1 & R2
    temp64_outs[1] = zip(samplesOut[5], samplesOut[7], 0);      // Interleave R3 & R4
    zipped_outs[0] = (unsigned int)(temp64_outs[0] >> 32);      // MSB of [R1,R2] into 32bits
    zipped_outs[1] = (unsigned int)(temp64_outs[0] >>  0);      // LSB of [R1,R2] into 32bits
    zipped_outs[2] = (unsigned int)(temp64_outs[1] >> 32);      // MSB of [L3,L4] into 32bits
    zipped_outs[3] = (unsigned int)(temp64_outs[1] >>  0);      // LSB of [L3,L4] into 32bits


    /* Packs of 2 bits */
    temp64_outs[0] = zip(zipped_outs[0], zipped_outs[2], 1);    // Interleave MSB's of [R1,R2] with [R3,R4]
    temp64_outs[1] = zip(zipped_outs[1], zipped_outs[3], 1);    // Interleave LSB's of [R1,R2] with [R3,R4]
    zipped_outs[4] = (unsigned int)(temp64_outs[0] >> 32);      // MSBs of [R1,R2,R3,R4] into 32bits
    zipped_outs[5] = (unsigned int)(temp64_outs[0]      );      // MSBs of [R1,R2,R3,R4] into 32bits
    zipped_outs[6] = (unsigned int)(temp64_outs[1] >> 32);      // LSBs of [R1,R2,R3,R4] into 32bits
    zipped_outs[7] = (unsigned int)(temp64_outs[1]      );      // LSBs of [R1,R2,R3,R4] into 32bits
}

static void i2s_frame_init_ports(
        out buffered port:32 (&?p_dout)[num_out],
        static const size_t num_out,
        in buffered port:32 (&?p_din)[num_in],
        static const size_t num_in,
        out port p_bclk0,
        out port p_bclk1,
        out port p_bclk2,
        out port p_bclk3,
        out buffered port:32 p_lrclk,
        clock bclk,
        in port p_mclk,
        unsigned mclk_bclk_ratio){

    set_clock_on(bclk);
    configure_clock_src_divide(bclk, p_mclk, mclk_bclk_ratio >> 1);
    configure_port_clock_output(p_bclk0, bclk);
    configure_port_clock_output(p_bclk1, bclk);
    configure_port_clock_output(p_bclk2, bclk);
    configure_port_clock_output(p_bclk3, bclk);
    configure_out_port(p_lrclk, bclk, 1);
    for (size_t i = 0; i < num_out; i++)
        configure_out_port(p_dout[i], bclk, 0);
    for (size_t i = 0; i < num_in; i++)
        configure_in_port(p_din[i], bclk);
}

#pragma unsafe arrays
static i2s_restart_t i2s_frame_ratio_n(client i2s_frame_callback_if i2s_i,
        out buffered port:32 (&?p_dout)[num_out],
        static const size_t num_out,
        in buffered port:32 (&?p_din)[num_in],
        static const size_t num_in,
        out port p_bclk0,
        out port p_bclk1,
        out port p_bclk2,
        out port p_bclk3,
        clock bclk,
        out buffered port:32 p_lrclk,
        unsigned ratio,
        i2s_mode_t mode){

    int32_t in_samps[16]; //Workaround: should be (num_in << 1) but compiler thinks that isn't const,
    int32_t out_samps[16];//so setting to 16 which should be big enough for most cases
    unsigned int zipped_outs[I2S_WIRES][NUM_IO_PER_I2S_FRAME];

    // Since #pragma unsafe arrays is used need to ensure array won't overflow.
    assert((num_in << 1) <= 16);

    unsigned lr_mask = 0xF0000000; // Was lr_mask = 0

    for (size_t i=0;i<num_out;i++)
        clearbuf(p_dout[i]);
    for (size_t i=0;i<num_in;i++)
        clearbuf(p_din[i]);
    clearbuf(p_lrclk);

    if (num_out) i2s_i.send(num_out << 1, out_samps);

    //Start outputting evens (0,2,4..) data at correct point relative to the clock
    int offset = 0;
    if (mode==I2S_MODE_I2S) {
        offset = 1;
    }

#pragma loop unroll
    for (size_t i=0; i<num_out; i++){
        zip_samplesOut_to_output_buffer_left((unsigned int*)&out_samps[i * NUM_CHANS_PER_WIRE], zipped_outs[i]); // Turn our samples into port vals
        p_dout[i] @ (1 + offset) <: bitrev(zipped_outs[i][0]);
        p_dout[i] <: bitrev(zipped_outs[i][1]);
        p_dout[i] <: bitrev(zipped_outs[i][2]);
        p_dout[i] <: bitrev(zipped_outs[i][3]);
    }

    p_lrclk @ 1 <: lr_mask;

    start_clock(bclk);

    //And pre-load the odds (1,3,5..)
#pragma loop unroll
    for (size_t i=0, idx=1; i<num_out; i++, idx+=2){
        p_dout[i] <: bitrev(zipped_outs[i][0 + 4]);
        p_dout[i] <: bitrev(zipped_outs[i][1 + 4]);
        p_dout[i] <: bitrev(zipped_outs[i][2 + 4]);
        p_dout[i] <: bitrev(zipped_outs[i][3 + 4]);
    }

    lr_mask = ~lr_mask;
    p_lrclk <: lr_mask;

    for (size_t i=0;i<num_in;i++) {
        asm("setpt res[%0], %1"::"r"(p_din[i]), "r"(32 + offset));
    }

    while(1) {
        // Check for restart
        i2s_restart_t restart = i2s_i.restart_check();

        if (restart == I2S_NO_RESTART) {
            if (num_out) i2s_i.send(num_out << 1, out_samps);

            //Output i2s evens (0,2,4..)
#pragma loop unroll
            for (size_t i=0; i<num_out; i++){
                zip_samplesOut_to_output_buffer_left((unsigned int*)&out_samps[i * NUM_CHANS_PER_WIRE], zipped_outs[i]); // Turn our samples into port vals
                p_dout[i] @ (1 + offset) <: bitrev(zipped_outs[i][0]);
                p_dout[i] <: bitrev(zipped_outs[i][1]);
                p_dout[i] <: bitrev(zipped_outs[i][2]);
                p_dout[i] <: bitrev(zipped_outs[i][3]);
            }
        }

        //Input i2s evens (0,2,4..)
#pragma loop unroll
        for (size_t i=0, idx=0; i<num_in; i++, idx+=2){
            int32_t data;
            asm volatile("in %0, res[%1]":"=r"(data):"r"(p_din[i]):"memory");
            in_samps[idx] = bitrev(data);
        }

        lr_mask = ~lr_mask;
        p_lrclk <: lr_mask;

        if (restart == I2S_NO_RESTART) {
            //Output i2s odds (1,3,5..)
#pragma loop unroll
            for (size_t i=0; i<num_out; i++){
                p_dout[i] <: bitrev(zipped_outs[i][0 + 4]);
                p_dout[i] <: bitrev(zipped_outs[i][1 + 4]);
                p_dout[i] <: bitrev(zipped_outs[i][2 + 4]);
                p_dout[i] <: bitrev(zipped_outs[i][3 + 4]);
            }

            lr_mask = ~lr_mask;
            p_lrclk <: lr_mask;
        }

        //Input i2s odds (1,3,5..)
#pragma loop unroll
        for (size_t i=0, idx=1; i<num_in; i++, idx+=2){
            int32_t data;
            asm volatile("in %0, res[%1]":"=r"(data):"r"(p_din[i]):"memory");
            in_samps[idx] = bitrev(data);
        }

        if (num_in) i2s_i.receive(num_in << 1, in_samps);

        if (restart != I2S_NO_RESTART) {
            if (!num_in) {
                // Prevent the clock from being stopped before the last word
                // has been sent if there are no RX ports.
                sync(p_dout[0]);
            }
            stop_clock(bclk);
            return restart;
        }
    }
    return I2S_RESTART;
}

#define i2s_frame_master i2s_frame_master0

static void i2s_frame_master0(client i2s_frame_callback_if i2s_i,
                out buffered port:32 (&?p_dout)[num_out],
                static const size_t num_out,
                in buffered port:32 (&?p_din)[num_in],
                static const size_t num_in,
                out port p_bclk0,
                out port p_bclk1,
                out port p_bclk2,
                out port p_bclk3,
                out buffered port:32 p_lrclk,
                in port p_mclk,
                clock bclk){
    xassert( num_in ==0 );

    while(1){
        i2s_config_t config;
        unsigned mclk_bclk_ratio_log2;
        i2s_i.init(config, null);

        if (isnull(p_dout) && isnull(p_din)) {
            fail("Must provide non-null p_dout or p_din");
        }

        mclk_bclk_ratio_log2 = clz(bitrev(config.mclk_bclk_ratio));

        //This ensures that the port time on all the ports is at 0
        i2s_frame_init_ports(p_dout, num_out, p_din, num_in, p_bclk0, p_bclk1, p_bclk2, p_bclk3, p_lrclk, bclk,
            p_mclk, config.mclk_bclk_ratio);

        i2s_restart_t restart =
          i2s_frame_ratio_n(i2s_i, p_dout, num_out, p_din,
                      num_in, p_bclk0, p_bclk1, p_bclk2, p_bclk3, bclk, p_lrclk,
                      mclk_bclk_ratio_log2, config.mode);

        if (restart == I2S_SHUTDOWN)
          return;
    }
}

// This function is just to avoid unused static function warnings for i2s_frame_master0,
// it should never be called.
inline void i2s_frame_master1(client interface i2s_frame_callback_if i,
        out buffered port:32 i2s_dout[num_i2s_out],
        static const size_t num_i2s_out,
        in buffered port:32 i2s_din[num_i2s_in],
        static const size_t num_i2s_in,
        out port i2s_bclk0,
        out port i2s_bclk1,
        out port i2s_bclk2,
        out port i2s_bclk3,
        out buffered port:32 i2s_lrclk,
        in port p_mclk,
        clock clk_bclk) {
    i2s_frame_master0(i, i2s_dout, num_i2s_out, i2s_din, num_i2s_in,
                i2s_bclk0, i2s_bclk1, i2s_bclk2, i2s_bclk3, i2s_lrclk, p_mclk, clk_bclk);
}

#endif // __XS2A__
