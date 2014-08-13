/** @file hf_pid.c
 * @brief A PID controller
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

#include "main.h"

/**
 * Initialize PID
 * @param p
 * @param setpoint_x10
 * @param input_x10
 * @param output
 * @param Kp
 * @param Ki
 * @param Kd
 * @param reverse
 */
void hf_pid_init(hf_pid_t *p, uint16_t setpoint_x10, uint16_t input_x10, float output, float Kp, float Ki, float Kd, bool reverse) {
    p->setpoint = (float) setpoint_x10 / 10.0;
    p->input = (float) input_x10 / 10.0;
    p->output = output;
    p->manual = true;
    p->reverse = reverse;

    p->output_limit[OL_LOW] = 0;
    p->output_limit[OL_HIGH] = 300;
    p->sample_time = 100;
    p->last_calc_time = msec_ticker;

    hf_pid_set_tuning(p, Kp, Ki, Kd, reverse);
}

/**
 * Compute PID
 * @param p
 * @param input_x10
 * @return
 */
bool hf_pid_compute(hf_pid_t *p, uint16_t input_x10) {
    float d_input;
    float error;

    if (p->manual)
        return false;

    if (elapsed_since(p->last_calc_time) >= p->sample_time) {
        p->input = (float) input_x10 / 10.0;

        error = p->setpoint - p->input;

        p->i_term += (p->ki * error);

        OUTPUT_LIMIT(p->i_term);

        d_input = (p->input - p->last_input);
        p->output = p->kp * error + p->i_term - p->kd * d_input;

        OUTPUT_LIMIT(p->output);

        p->last_input = p->input;
        p->last_calc_time = msec_ticker;
        return true;
    } else
        return false;
}

/**
 * Set PID tuning
 * @param p
 * @param Kp
 * @param Ki
 * @param Kd
 * @param reverse
 */
void hf_pid_set_tuning(hf_pid_t *p, float Kp, float Ki, float Kd, bool reverse) {
    float sample_time_seconds = ((float) p->sample_time) / 1000.0;

    if (Kp < 0 || Ki < 0 || Kd < 0 || p->sample_time == 0)
        return;

    p->Kp = Kp;
    p->Ki = Ki;
    p->Kd = Kd;

    p->kp = Kp;
    p->ki = Ki * sample_time_seconds;
    p->kd = Kd / sample_time_seconds;

    if (reverse) {
        p->kp = (0 - p->kp);
        p->ki = (0 - p->ki);
        p->kd = (0 - p->kd);
    }
}

/**
 * Change sample time of PID
 * @param p
 * @param new_sample_time
 */
void hf_pid_change_sample_time(hf_pid_t *p, int new_sample_time) {
    if (new_sample_time > 0) {
        if (p->sample_time > 0) {
            float correction = (float) new_sample_time / (float) p->sample_time;

            p->ki *= correction;
            p->kd /= correction;
        }
        p->sample_time = new_sample_time;
    }
}

/**
 * Set output limits of PID
 * @param p
 * @param min
 * @param max
 */
void hf_pid_set_output_limits(hf_pid_t *p, float min, float max) {
    if (min >= max)
        return;

    p->output_limit[OL_LOW] = min;
    p->output_limit[OL_HIGH] = max;

    if (p->manual == false) {
        OUTPUT_LIMIT(p->output);
        OUTPUT_LIMIT(p->i_term);
    }
}

/**
 * Set a PID as manual
 * @param p
 */
void hf_pid_set_manual(hf_pid_t *p) {
    p->manual = true;
}

/**
 * Set a PID as automatic
 * @param p
 * @param input
 * @param output
 */
void hf_pid_set_auto(hf_pid_t *p, float input, float output) {
    if (p->manual == true) {
        p->manual = false;
        p->i_term = output;
        p->output = output;
        p->last_input = input;

        OUTPUT_LIMIT(p->i_term);
    }
}
