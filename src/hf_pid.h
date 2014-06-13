//
// Structure maintained for each instance of PID control
//



#ifndef _HF_PID_H_
#define _HF_PID_H_

typedef struct {
    // User settings
    float setpoint;
    float Kp;                             
    float Ki;
    float Kd;
    bool reverse;                           // true for reverse direction
    bool manual;                            // true for manual mode
    float output_limit[2];                 // Low, high
    unsigned sample_time;                   // Sample time to use in msec

    // In and out
    float input;
    float output;

    // Internal variables
    float last_input;
    float i_term;
    float kp, ki, kd;                      // Scaled and direction corrected
    uint16_t last_calc_time;                // Last calculation time in msec
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

#endif
