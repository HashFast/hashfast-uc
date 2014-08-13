/** @file hf_pid.h
 * @brief Structure maintained for each instance of PID control
 *
 * @copyright
 * Copyright (c) 2014, HashFast Technologies LLC
 * All rights reserved.
 *
 * @page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   1.  Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *   2.  Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *   3.  Neither the name of HashFast Technologies LLC nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL HASHFAST TECHNOLOGIES LLC BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _HF_PID_H_
#define _HF_PID_H_

/**
 * HF PID structure
 */
typedef struct {
    /* user settings */
    float setpoint;
    float Kp;
    float Ki;
    float Kd;
    bool reverse;               //!< true for reverse direction
    bool manual;                //!< true for manual mode
    float output_limit[2];      //!< Low, high
    unsigned sample_time;       //!< Sample time to use in msec
    /* in and out */
    float input;
    float output;
    /* internal variables */
    float last_input;
    float i_term;
    float kp, ki, kd;           //!< Scaled and direction corrected
    uint16_t last_calc_time;    //!< Last calculation time in msec
} hf_pid_t;

#define OL_LOW 0
#define OL_HIGH 1

#define OUTPUT_LIMIT(x) \
        if ((x) > p->output_limit[OL_HIGH]) \
            (x) = p->output_limit[OL_HIGH]; \
        else if ((x) < p->output_limit[OL_LOW]) \
            (x) = p->output_limit[OL_LOW]

void hf_pid_init(hf_pid_t *, uint16_t, uint16_t, float, float, float, float, bool);
bool hf_pid_compute(hf_pid_t *, uint16_t);
void hf_pid_set_tuning(hf_pid_t *, float, float, float, bool);
void hf_pid_change_sample_time(hf_pid_t *, int);
void hf_pid_set_output_limits(hf_pid_t *, float, float);
void hf_pid_set_manual(hf_pid_t *);
void hf_pid_set_auto(hf_pid_t *, float, float);

#endif /* _HF_PID_H_ */
