/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ma40h1s.cpp
 * @author KeChaofan
 *
 * Driver for the ma40h1s sonar range finders connected via GPIO.
 */

#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_defines.h>
#include <board_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <lib/mathlib/mathlib.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

//#include <systemlib/perf_counter.h>
#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/stm32/drv_io_timer.h>
#include <drivers/drv_pwm_output.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position_setpoint.h> 

#include <modules/px4iofirmware/protocol.h>

#include <arch/board/board.h>

#include <stm32_adc.h>
#include <stm32_gpio.h>
#include <stm32_dma.h>
#include <float.h>
#include <getopt.h>


#include "sonar_decoder_c.c"
/*
 * Register accessors.
 */
#define REG(_reg)   (*(volatile uint32_t *)(STM32_ADC2_BASE + _reg))

#define rSR         REG(STM32_ADC_SR_OFFSET)
#define rCR1        REG(STM32_ADC_CR1_OFFSET)
#define rCR2        REG(STM32_ADC_CR2_OFFSET)
#define rCSR        REG(STM32_ADC_CSR_OFFSET)
#define rCCR        REG(STM32_ADC_CCR_OFFSET) 
#define rSMPR1      REG(STM32_ADC_SMPR1_OFFSET)
#define rSMPR2      REG(STM32_ADC_SMPR2_OFFSET)
#define rJOFR1      REG(STM32_ADC_JOFR1_OFFSET)
#define rJOFR2      REG(STM32_ADC_JOFR2_OFFSET)
#define rJOFR3      REG(STM32_ADC_JOFR3_OFFSET)
#define rJOFR4      REG(STM32_ADC_JOFR4_OFFSET)
#define rHTR        REG(STM32_ADC_HTR_OFFSET)
#define rLTR        REG(STM32_ADC_LTR_OFFSET)
#define rSQR1       REG(STM32_ADC_SQR1_OFFSET)
#define rSQR2       REG(STM32_ADC_SQR2_OFFSET)
#define rSQR3       REG(STM32_ADC_SQR3_OFFSET)
#define rJSQR       REG(STM32_ADC_JSQR_OFFSET)
#define rJDR1       REG(STM32_ADC_JDR1_OFFSET)
#define rJDR2       REG(STM32_ADC_JDR2_OFFSET)
#define rJDR3       REG(STM32_ADC_JDR3_OFFSET)
#define rJDR4       REG(STM32_ADC_JDR4_OFFSET)
#define rDR         REG(STM32_ADC_DR_OFFSET)


#define MA40H1S_CONVERSION_INTERVAL 43500 //us

#define MID_LENGTH  7

#define ADC_BUFFER_SIZE 10000

enum MA40H1S_ID
{
    MA40H1S_ID_ALL          = 0,
    MA40H1S_ID_EXPANSION    = 1,
    MA40H1S_ID_EXPANSION1   = 2,
    MA40H1S_ID_EXPANSION2   = 3,
    MA40H1S_ID_EXPANSION3   = 4, 
    MA40H1S_ID_PRIMARY      = 5,       
};


#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class MA40H1S : public device::CDev
{
public:
    MA40H1S();

    virtual ~MA40H1S();

    virtual int init();

    virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);
    virtual int     ioctl(struct file *filp, int cmd, unsigned long arg);

    /**
    * Diagnostics - print some basic information about the driver.
    */
    void    print_info();
    void    trig();
    void    interrupt(hrt_abstime time);

    void    test_high();
    void    test_low();

protected:
    virtual int probe();

private:
    float _min_distance;
    float _max_distance;
    float distance_orginal;

    int _class_instance;
    int _orb_class_instance;
    int _measure_ticks;
    int _cycling_rate;

    static bool _echo_valid;
    static uint8_t trig_state;
	//static bool voltage_state;
    static uint8_t  _echo_count;

    bool new_value;
    
    orb_advert_t    _distance_sensor_topic;
    work_s          _work;
	
	struct actuator_armed_s	 _armed = {};	
	struct vehicle_land_detected_s	 _vehicle_land_detected = {};
	
    ringbuffer::RingBuffer  *_reports;
    static hrt_abstime _start_time; 
    hrt_abstime _end_time;
    uint16_t _end_index;
    hrt_abstime _last_inte;

    struct hrt_call     _call;

    // DMA_HANDLE      _tx1_dma;
    DMA_HANDLE      _adc_dma;

    // static struct stm32_tim_dev_s * _tim8; 
    struct stm32_tim_dev_s * _tim5; 

    static bool _time_up;

    // uint32_t     _dma_buffer[2];
    static uint16_t     adc_buffer[ADC_BUFFER_SIZE];
    // uint16_t     adc_buffer[ADC_BUFFER_SIZE];

    uint16_t average_sample;
    uint16_t thr_value;

    bool single_test_mode;

    // struct GPIOConfig {
    //     uint32_t        dr_a_port;
    //     uint32_t        dr_b_port;
    // };
    // uint32_t        _dr_a_port;
    // uint32_t        _dr_b_port;
    
    // uint32_t _GPIOx_BSRR_addr;
    // uint8_t _ADC_Channel; 
    enum MA40H1S_ID _ultrasonic_id;

    struct dev_config {
        enum MA40H1S_ID id;
        uint8_t pwm1_ch;
        uint8_t pwm2_ch;
        uint8_t timer_index;
        uint8_t adc_ch;
    };
    static const dev_config _ultrasonic_config[NUM_OF_ULTRASONIC_DEV];
    // static const GPIOConfig _gpio_tab;
    /**
    * Initialise the automatic measurement state machine and start it.
    */
    void start();
    
    /**
    * Stop the automatic measurement state machine.
    */
    void stop();

    /**
    * Perform a poll cycle; collect from the previous measurement
    * and start a new one.
    */
    void cycle();
    int  measure();
    int  collect();

    /**
    * Set the min and max distance thresholds if you want the end points of the sensors
    * range to be brought in at all, otherwise it will use the defaults MB12XX_MIN_DISTANCE
    * and MB12XX_MAX_DISTANCE
    */
    void  set_minimum_distance(float min);
    void  set_maximum_distance(float max);
    float get_minimum_distance();
    float get_maximum_distance();

    /**
    * Static trampoline from the workq context; because we don't have a
    * generic workq wrapper yet.
    *
    * @param arg        Instance pointer for the driver that is polling.
    */
    static void cycle_trampoline(void *arg);

    static void tick_trampoline(void *arg);

    static int timer5_interrupt(int irq, void *context, void *arg);

    static void  _dma_callback(DMA_HANDLE handle, uint8_t status, void *arg);

    void        _do_adc_dma_callback(unsigned status);
};

// const MA40H1S::GPIOConfig MA40H1S::_gpio_tab = {
//     GPIO_DR_A,
//     GPIO_DR_B
// };

const MA40H1S::dev_config MA40H1S::_ultrasonic_config[NUM_OF_ULTRASONIC_DEV] = {
    {MA40H1S_ID_PRIMARY, 6, 5, 1, 4}
    #if (NUM_OF_ULTRASONIC_DEV > 1)
    ,{MA40H1S_ID_EXPANSION, 8, 7, 2, 14}
    #endif
};


hrt_abstime MA40H1S::_start_time = 0;
bool MA40H1S::_echo_valid = false;
bool MA40H1S::_time_up = false;
uint8_t MA40H1S::trig_state = 5;
//bool MA40H1S::voltage_state = false;
uint8_t MA40H1S::_echo_count = 0;
// struct stm32_tim_dev_s MA40H1S::_tim5 = NULL; 

 /*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ma40h1s_main(int argc, char *argv[]);
// static int sonar_isr(int irq, void *context);

//uint32_t MA40H1S::dma_buffer[2] = {0x04000010,0x00100400};
//uint32_t MA40H1S::dma_buffer[2] = {0x00100002,0x00020010};
uint16_t MA40H1S::adc_buffer[ADC_BUFFER_SIZE] = {};
// struct stm32_tim_dev_s * _tim8 = nullptr;
//struct stm32_tim_dev_s * _tim5 = nullptr;

MA40H1S::MA40H1S():
    CDev("MA40H1S", MA40H1S_DEVICE_PATH),
    _min_distance(0.28f),
    _max_distance(2.0f),
    _class_instance(-1),
    _orb_class_instance(-1),
    _measure_ticks(0),
    _cycling_rate(0),
    new_value(false),
    _distance_sensor_topic(nullptr),
    _reports(nullptr),
    _end_time(0),
    _end_index(0),
    _last_inte(0),
    // _tx1_dma(nullptr),
    _adc_dma(nullptr)
{
    _armed.armed = false;
    _vehicle_land_detected.landed = true;
    single_test_mode = false;
    memset(&_work, 0, sizeof(_work));
    memset(&_call, 0, sizeof(_call));
}

MA40H1S::~MA40H1S()
{
    stop();

    /* free any existing reports */
    if (_reports != nullptr) {
        delete _reports;
    }

    // if (_tim5 != nullptr) {
    // 	stm32_tim_deinit(_tim5);
    // 	_tim5 = nullptr;
    // }

    if (_class_instance != -1) {
        unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
    }   
}

int MA40H1S::init()
{
    int ret = ERROR;

    /* do super class device init first*/
    if(CDev::init() != OK){
        return ret;
    }

    /* allocate basic report buffers */
    _reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

    if (_reports == nullptr) {
        return ret;
    }

    _class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

    if (_class_instance == CLASS_DEVICE_PRIMARY) {
        /* get a publish handle on the range finder topic */
        struct distance_sensor_s ds_report = {};

        _distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
                                 &_orb_class_instance, ORB_PRIO_LOW);

        if (_distance_sensor_topic == nullptr) {
            DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
        }
    }
    // stm32_configgpio(_dr_a_port);
    // stm32_configgpio(_dr_b_port);
    //stm32_configgpio(_gpio_tab.sw_a_port); 
    // stm32_configgpio(_gpio_tab.sw_b_port);
    //stm32_configgpio(GPIO_BAK_DR_B);
    // printf("dma init start\n");
    // _tx1_dma = stm32_dmachannel(PX4FMU_SONAR_TX4_DMAMAP); 

    _ultrasonic_id = _ultrasonic_config[0].id;
    io_timer_channel_init(_ultrasonic_config[0].pwm2_ch, IOTimerChanMode_PWMOut, NULL, NULL); // init PWM CH7/CH5
    io_timer_channel_init(_ultrasonic_config[0].pwm1_ch, IOTimerChanMode_PWMOut, NULL, NULL); // init PWM CH8/CH6
    io_timer_set_rate(_ultrasonic_config[0].timer_index, 40000); //timer_index 1: TIM4   timer_index 2: TIM12
    io_timer_set_ccr(_ultrasonic_config[0].pwm2_ch, 12);
    io_timer_set_ccr(_ultrasonic_config[0].pwm1_ch, 12);

    // if(_tx1_dma == nullptr || _tx1_dma == NULL){
    //     // printf("dma init failed\n");
    //     return ret;
    // }
	
    /*
     stm32_dmasetup(
        _tx1_dma, 
        _GPIOx_BSRR_addr, // Pb 0x40020818 0x40021000
        reinterpret_cast<uint32_t>(&_dma_buffer),
        2,
        DMA_SCR_DIR_M2P |\
        DMA_SCR_MINC |\
        DMA_SCR_PSIZE_32BITS |\
        DMA_SCR_MSIZE_32BITS |\
        DMA_SCR_PBURST_SINGLE |\
        DMA_SCR_MBURST_SINGLE |\
	    DMA_SCR_CIRC);*/
    // stm32_dmastart(_tx1_dma, nullptr, nullptr, false);
    // printf("tx1 dma\n");
	
 
    // _tim8 = stm32_tim_init(8);
    // if(_tim8 == NULL){
    //     // printf("timer8 init failed\n");
    //     return ret;
    // }
    // // printf("timer8 init success\n");
    // STM32_TIM_SETPERIOD(_tim8, 24);
    // STM32_TIM_SETCLOCK(_tim8,2000000);
    // STM32_TIM_SETMODE(_tim8,STM32_TIM_MODE_UP);
    // //STM32_TIM_SETCOMPARE(_tim8,3,11);
    // //STM32_TIM_SETCOMPARE(_tim8,2,5);
    // usleep(200000);

    _tim5 = stm32_tim_init(5);
    if(_tim5 == NULL){
        // printf("timer5 init failed\n");
        PX4_WARN("Timer5 init failed");
        return ret;
    }
    // printf("timer5 init success\n");
    enum MA40H1S_ID * pdev_id =  &_ultrasonic_id;
    STM32_TIM_SETISR(_tim5, MA40H1S::timer5_interrupt, pdev_id, 0);
    putreg16(0x0101,STM32_TIM5_DIER);//  putreg16(0x0101,0x40000c0c);  //STM32_TIM5_BASE:0x40000c00  STM32_GTIM_DIER_OFFSET:0x000c
    STM32_TIM_SETPERIOD(_tim5, 4);
    STM32_TIM_SETCLOCK(_tim5,1000000);
    STM32_TIM_SETMODE(_tim5,STM32_TIM_MODE_UP);
    // printf("CNT:%d\n",getreg16(0x40000c24));
    // STM32_TIM_SETCOMPARE();

    _cycling_rate = MA40H1S_CONVERSION_INTERVAL;

    /* init gpio ports */

	// stm32_gpiowrite(_dr_a_port,false);
 //    stm32_gpiowrite(_dr_b_port,false);
	//stm32_gpiowrite(_gpio_tab.sw_a_port,false);
	//stm32_gpiowrite(_gpio_tab.sw_b_port,true);
	// stm32_gpiosetevent(_gpio_tab.adc_port, true, false, false, sonar_isr);

    _adc_dma = stm32_dmachannel(DMAMAP_ADC2_1);
    if(_adc_dma == nullptr || _adc_dma == NULL){
        printf("adc dma init failed\n");
        return ret;
    }
    // PX4_INFO("ADC_DMA channel");

    stm32_dmasetup(
        _adc_dma, 
        STM32_ADC2_DR, // adc2 DR  
        reinterpret_cast<uint32_t>(&adc_buffer),
        ADC_BUFFER_SIZE,
        DMA_SCR_DIR_P2M |\
        DMA_SCR_MINC |\
        DMA_SCR_PSIZE_16BITS |\
        DMA_SCR_MSIZE_16BITS |\
        DMA_SCR_PBURST_SINGLE |\
        DMA_SCR_MBURST_SINGLE);//DMA_SCR_CIRC
    //PX4_INFO("ADC_DMA setup");

    // stm32_dmastart(_adc_dma, _dma_callback, this, false);
    //PX4_WARN("_ADC_dma start");
    //return ret;
    // printf("adc dma\n");
    /* arbitrarily configure all channels for 15 cycle sample time */
    //rSMPR1 = 0b00 000 000 000 000 010 000 000 000 000 000; //  10--18  Channel 15
    //rSMPR2 = 0b00 000 000 000 000 010 000 000 000 000 000; //  0--9  Channel 5
    rSMPR1 = 0b00000000000000000100000000000000; //  set sample time of adc_ch14 to 010 (28T)
    rSMPR2 = 0b00000000000000000100000000000000; // set sample time of adc_ch4 to 010 (28T)

    rCR1 = ADC_CR1_RES_12BIT; //Resolution
    rCR2 = 0;

    rSQR1 = 0;
    rSQR2 = 0;
    //rSQR3 = 15;  /* will be updated with the channel each tick */
    rSQR3 = _ultrasonic_config[0].adc_ch;
    //PX4_INFO();
    if(rSR & ADC_SR_EOC) {
       rSR &= ~ADC_SR_EOC;
    }

    rCR2 |= ADC_CR2_DDS;
    rCR2 |= ADC_CR2_DMA;
    rCR2 |= ADC_CR2_CONT;

    /* power-cycle the ADC and turn it on */
    rCR2 &= ~ADC_CR2_ADON;
    usleep(10);
    rCR2 |= ADC_CR2_ADON;
    usleep(10);
    rCR2 |= ADC_CR2_ADON;
    usleep(10);

    if(rSR & ADC_SR_EOC) {
       rSR &= ~ADC_SR_EOC;
    }
    rCR2 |= ADC_CR2_DDS;
    rCR2 |= ADC_CR2_DMA;
    // printf("ADC setting\n");
    // return ret;
    // rCR2 |= ADC_CR2_SWSTART;
    // printf("adc init success\n");
   	PX4_INFO("ma40h1s start!");
    ret = OK;

    return ret;
}

int MA40H1S::probe()
{
    return OK;
}

void MA40H1S::set_minimum_distance(float min)
{
    _min_distance = min;
}

void MA40H1S::set_maximum_distance(float max)
{
    _max_distance = max;
}

float MA40H1S::get_minimum_distance()
{
    return _min_distance;
}

float MA40H1S::get_maximum_distance()
{
    return _max_distance;
}

int MA40H1S::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    switch (cmd) {

    case SENSORIOCSPOLLRATE: {
            switch (arg) {

            /* switching to manual polling */
            case SENSOR_POLLRATE_MANUAL:
                stop();
                _measure_ticks = 0;
                return OK;

            /* external signalling (DRDY) not supported */
            case SENSOR_POLLRATE_EXTERNAL:

            /* zero would be bad */
            case 0:
                return -EINVAL;

            /* set default/max polling rate */
            case SENSOR_POLLRATE_MAX:
            case SENSOR_POLLRATE_DEFAULT: {
                    /* do we need to start internal polling? */
                    bool want_start = (_measure_ticks == 0);

                    /* set interval for next measurement to minimum legal value */
                    _measure_ticks = USEC2TICK(_cycling_rate);

                    /* if we need to start the poll state machine, do it */
                    if (want_start) {
                        start();

                    }

                    return OK;
                }

            /* adjust to a legal polling interval in Hz */
            default: {
                    /* do we need to start internal polling? */
                    bool want_start = (_measure_ticks == 0);

                    /* convert hz to tick interval via microseconds */
                    int ticks = USEC2TICK(1000000 / arg);

                    /* check against maximum rate */
                    if (ticks < USEC2TICK(_cycling_rate)) {
                        return -EINVAL;
                    }

                    /* update interval for next measurement */
                    _measure_ticks = ticks;

                    /* if we need to start the poll state machine, do it */
                    if (want_start) {
                        start();
                    }

                    return OK;
                }
            }
        }

    case SENSORIOCGPOLLRATE:
        if (_measure_ticks == 0) {
            return SENSOR_POLLRATE_MANUAL;
        }

        return (1000 / _measure_ticks);

    case SENSORIOCSQUEUEDEPTH: {
            /* lower bound is mandatory, upper bound is a sanity check */
            if ((arg < 1) || (arg > 100)) {
                return -EINVAL;
            }

            irqstate_t flags = px4_enter_critical_section();

            if (!_reports->resize(arg)) {
                px4_leave_critical_section(flags);
                return -ENOMEM;
            }

            px4_leave_critical_section(flags);

            return OK;
        }

    case SENSORIOCGQUEUEDEPTH:
        return _reports->size();

    case SENSORIOCRESET:
        /* XXX implement this */
        return -EINVAL;

    case RANGEFINDERIOCSETMINIUMDISTANCE: {
            set_minimum_distance(*(float *)arg);
            return 0;
        }
        break;

    case RANGEFINDERIOCSETMAXIUMDISTANCE: {
            set_maximum_distance(*(float *)arg);
            return 0;
        }
        break;
    case RANGEFINDERSINGLEMEASURE: {
            single_test_mode = true;
            measure();
            return 0;
        }
        break;
    case RANGEFINDERSINGLECOLLECT: {
            collect();
            return (int)(distance_orginal*1000.0f);
        }
    case RANGEFINDERGETAVR:
        return average_sample;
    case RANGEFINDERGETTHR:
        return adc_buffer[_end_index];
    default:
        /* give it to the superclass */
        return CDev::ioctl(filp, cmd, arg);
    }
}

ssize_t MA40H1S::read(struct file *filp, char *buffer, size_t buflen)
{
    // const size_t maxsize = sizeof(uint16_t) * ADC_BUFFER_SIZE;
    ssize_t ret = 0;
    if(buflen > ADC_BUFFER_SIZE-128){
        // buflen = ADC_BUFFER_SIZE-64;
        ret = 1;
    }

    memcpy(buffer, adc_buffer + buflen, 128);
    // for (int i = 0; i < 64; ++i)
    // {
    //     /* code */
    //     buffer[i] = adc_buffer[buflen+i];
    // }

    return ret;

    // unsigned count = buflen / sizeof(struct distance_sensor_s);
    // struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
    // int ret = 0;

    // /* buffer must be large enough */
    // if (count < 1) {
    //     return -ENOSPC;
    // }

    // /* automatic measurement*/
    // /* if automatic measurement is enabled */
    // if (_measure_ticks > 0) {
    //     /*
    //      * While there is space in the caller's buffer, and reports, copy them.
    //      * Note that we may be pre-empted by the workq thread while we are doing this;
    //      * we are careful to avoid racing with them.
    //      */
    //     while (count--) {
    //         if (_reports->get(rbuf)) {
    //             ret += sizeof(*rbuf);
    //             rbuf++;
    //         }
    //     }

    //     /* if there was no data, warn the caller */
    //     return ret ? ret : -EAGAIN;
    // }

    // /* manual measurement */
    // _reports->flush();

    // if (OK != measure()) {
    //     ret = -EIO;
    //     return ret;
    // }

    // /* wait for it to complete */
    // usleep(_cycling_rate * 2);

    // /* run the collection phase */
    // if (OK != collect()) {
    //     ret = -EIO;
    //     return ret;
    // }

    // /* state machine will have generated a report, copy it out */
    // if (_reports->get(rbuf)) {
    //     ret = sizeof(*rbuf);
    // }

    // return ret;
}

int
MA40H1S::measure()
{
    int ret;
    /**
    * Send pulse sequence to begin a measurement
    */
    // trig();
    // stm32_gpiowrite(_gpio_tab.dr_a_port,true);
    // usleep(10);
    // stm32_gpiowrite(_gpio_tab.dr_a_port,false);
    // stm32_configgpio(_gpio_tab.adc_port);
    // stm32_configgpio(_gpio_tab.dr_a_port);
    // stm32_configgpio(_gpio_tab.dr_b_port);
    //stm32_configgpio(_gpio_tab.sw_a_port);  
    // stm32_configgpio(_gpio_tab.sw_b_port);
    // stm32_dmastart(_tx1_dma, nullptr, this, false);
    // printf("L%u\n", getreg32(0x40026000));
    // printf("H%u\n", getreg32(0x40026004));
   // printf("aa%u\n", getreg32(0x40026428));
	//printf("lt%u\n", getreg32(0x40026400));
	//printf("fifo%u\n", getreg32(0x40026442));
   // printf("cnt%u\n", getreg32(0x4002642c));
    //printf("a%u\n",  getreg32(0x40026410));
    // printf("pa%u\n", getreg32(0x40026018));
    // printf("ma%u\n", getreg32(0x4002601c));
    // printf("ff%u\n", getreg32(0x40026024));
    // stm32_configgpio(_gpio_tab.adc_port);
    trig_state = 0;
    ret = OK;
    return ret;
}

int MA40H1S::collect()
{
    int ret = -EIO;
    //static float storedRngMeas[MID_LENGTH];
    //static uint32_t storedRngMeasTime_ms[MID_LENGTH];
    //static uint8_t rngMeasIndex = 0;
	static uint8_t err_count = 0;
    /* calculate the actual distance */
    float distance_m  = 0.283f;
    static float pre_distance_m = 0.283f;
	static uint64_t _landed_time = 0;
    static uint64_t _data_invalid = 0;
	static float sonar_sample[10] = {0.283f,0.283f,0.283f,0.283f,0.283f,0.283f,0.283f,0.283f,0.283f,0.283f};
	//static float sonar_deta[20] = {1.2f,1.2f,1.2f,1.0f,1.0f,1.0f,0.8f,0.8f,0.8f,0.6f,0.6f,0.6f,0.4f,0.4f,0.4f,0.3f,0.3f,0.3f,0.2f,0.2f};
	float average = 0.0f;
	float sum = 0.0f;
	float sum_2 = 0.0f;
	float variance = 0.0f;
    // for (int i = 0; i < ADC_BUFFER_SIZE; ++i)
    // {
    //     printf("%d:%d\n",i,adc_buffer[i]);
    // }
    distance_orginal = 0.0f;
	
	int  _vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	orb_unsubscribe(_vehicle_land_detected_sub);
	
	int  _armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	orb_unsubscribe(_armed_sub);
	
	if(_vehicle_land_detected.landed){
		err_count = 0;
	}
	
    struct vehicle_local_position_setpoint_s lpsp = {};
    int vehicle_local_position_setpoint_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
    orb_copy(ORB_ID(vehicle_local_position_setpoint), vehicle_local_position_setpoint_sub, &lpsp);
    orb_unsubscribe(vehicle_local_position_setpoint_sub);
	
	if(_armed.armed && _vehicle_land_detected.landed && lpsp.throw_state>50){
		distance_m=0.283f;
		pre_distance_m=0.283f;
		_landed_time = hrt_absolute_time();
		goto out;
	}

	if(!_vehicle_land_detected.landed && (hrt_absolute_time() - _landed_time <1e6)){
		distance_m=0.283f;
		pre_distance_m=0.283f;
		goto out;
	}

    // if(lpsp.throw_state==2 || lpsp.throw_state==1){
    //     distance_m=1.2f;
    //     goto out;
    // }

    if(new_value == true){
        distance_m = ((float)_end_index*0.00229f)*0.17f*1.1f;   // 0.00042823
		 //printf("%d:%d\n",_end_index,adc_buffer[_end_index]);
         //printf("dis :%.2f\n", (double)distance_m);
        new_value = false;
		//err_count = 0;
		_data_invalid = 0;
    } else {
        _echo_valid = false;
        _echo_count = 0; 
		_data_invalid ++;
		//err_count ++;
		if(!_armed.armed && _data_invalid > 4){
			distance_m=0.283f;
			pre_distance_m=0.283f;
			goto out;
		}
        return ret;
    }

out:
    distance_orginal = distance_m;
    
    if(!_armed.armed){
        pre_distance_m = distance_m;
        //PX4_WARN("ARM222222222");
    }

    if(err_count > 1 && !_vehicle_land_detected.landed){
        pre_distance_m = distance_m;
    }
    //PX4_WARN("err %.2f",(double)err_count);
    //PX4_WARN("dist %.2f",(double)distance_m);
    //PX4_WARN("land %.2f",(double)_vehicle_land_detected.landed);
    if(fabsf(distance_m-pre_distance_m)>0.25f){
        distance_m = pre_distance_m;
        err_count ++;
    }else{
        err_count = 0;
        pre_distance_m = distance_m;    
    }

	if(distance_m < 0.28f){
	   distance_m = 0.28f;
	}
	if(pre_distance_m < 0.28f){
		pre_distance_m = 0.28f;
	}

	for(int i=8;i>=0;i--){
	   sonar_sample[i+1] = sonar_sample[i];
	}
	sonar_sample[0] = distance_m;
	for(int i=0;i<10;i++){
	   sum += sonar_sample[i];
	}
	average = sum/10;
	//get variance   sonar_deta[i]*
	for(int i=0;i<10;i++){
	   sum_2 += (sonar_sample[i] - average)*(sonar_sample[i] - average);
	}
	if((!_vehicle_land_detected.landed && (hrt_absolute_time() - _landed_time <1200000))){
	   variance = 0.0f;
	}else{
	   variance = sum_2/10;
	}
    struct distance_sensor_s report;
    report.timestamp = hrt_absolute_time();
    report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
    report.orientation = err_count;
    report.current_distance = distance_m;//0.5; //distance_mid;
    report.min_distance = get_minimum_distance();
    report.max_distance = get_maximum_distance();
	report.sonar_test[0] = distance_orginal;
    report.covariance = variance;
    /* TODO: set proper ID */
    report.id = (uint8_t)_ultrasonic_id;//uint32_t(distance_mid*100.0f);

    /* publish it, if we are the primary */
    if (_distance_sensor_topic != nullptr) {
        orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
    }

    _reports->force(&report);

    static uint8_t k = 0;
    if (trig_state == 5) {
        if ((++k) >= NUM_OF_ULTRASONIC_DEV) k = 0;
        _ultrasonic_id = _ultrasonic_config[k].id;
        io_timer_channel_init(_ultrasonic_config[k].pwm2_ch, IOTimerChanMode_PWMOut, NULL, NULL); // init PWM CH7/CH5
        io_timer_channel_init(_ultrasonic_config[k].pwm1_ch, IOTimerChanMode_PWMOut, NULL, NULL); // init PWM CH8/CH6
        io_timer_set_rate(_ultrasonic_config[k].timer_index, 40000); //timer_index 1: TIM4   timer_index 2: TIM12
        io_timer_set_ccr(_ultrasonic_config[k].pwm2_ch, 12);
        io_timer_set_ccr(_ultrasonic_config[k].pwm1_ch, 12);
        rSQR3 = _ultrasonic_config[k].adc_ch;
    }

    /* notify anyone waiting for data */
    poll_notify(POLLIN);

    ret = OK;

    return ret;
}

void MA40H1S::start()
{

    /* reset the report ring and state machine */
    //_collect_phase = false;
    _reports->flush();

    measure();  /* begin measure */

    /* schedule a cycle to start things */
    work_queue(HPWORK,
           &_work,
           (worker_t)&MA40H1S::cycle_trampoline,
           this,
           USEC2TICK(_cycling_rate));
}

void MA40H1S::stop()
{
    work_cancel(HPWORK, &_work);
}

void MA40H1S::cycle_trampoline(void *arg)
{

    MA40H1S *dev = (MA40H1S *)arg;

    dev->cycle();

}

void MA40H1S::cycle()
{
    /* perform collection */
    if(!single_test_mode){
        if (OK != collect()) {
            DEVICE_DEBUG("collect error");
        }   // kecf

        /* next sonar */
        if (OK != measure()) {
            DEVICE_DEBUG("measure error");
        }
    }

    work_queue(HPWORK,
           &_work,
           (worker_t)&MA40H1S::cycle_trampoline,
           this,
           USEC2TICK(_cycling_rate));
}

void MA40H1S::print_info()
{
    PX4_WARN("absolute time:%lld",_end_time - _start_time);
    // PX4_WARN("absolute time:%lld", _end_time);

    PX4_WARN("poll interval:  %u ticks\n", _measure_ticks);
    _reports->print_info("report queue");
}

void MA40H1S::trig()
{
    trig_state = 0;
}

void MA40H1S::interrupt(hrt_abstime time)
{
	if(_echo_valid){
		// printf("v\n");
		_echo_count++;
		if(_echo_count >= 2){
			// we have waited for too long, so clear the state
			if(hrt_absolute_time() - _start_time > 30000){
				_echo_count = 0;
				_echo_valid = false;
				return;
			}
			_echo_valid = false;
			_echo_count = 0;
			new_value = true;
		}
	}
}

void MA40H1S::test_high()
{
    static uint16_t tick=0;
    printf("%d\n",adc_buffer[tick]);
    tick++;
    if (tick==ADC_BUFFER_SIZE)
    {
        /* code */
        tick = 0;
    }
    // hrt_abstime now = hrt_absolute_time();
    // while (!(rSR & ADC_SR_EOC)) {
    //     // don't wait for more than 50us, since that means something broke - should reset here if we see this 
    //     if ((hrt_absolute_time() - now) > 50) {
    //         printf("sample timeout\n");
    //         return;
    //     }
    // }
    printf("%d\n",rDR);
}

void MA40H1S::test_low()
{
    rCR2 |= ADC_CR2_SWSTART;
    printf("tr:%d\n", trig_state);
}

void MA40H1S::tick_trampoline(void *arg)
{
}

int MA40H1S::timer5_interrupt(int irq, void *context, void *arg)
{
    enum MA40H1S_ID *pdev_id = (enum MA40H1S_ID *)arg;
    static uint16_t ticks = 0;
    putreg16(0xFFFE,STM32_TIM5_SR); // putreg16(0xFFFE,0x40000c10); //STM32_TIM5_BASE:0x40000c00    STM32_GTIM_SR_OFFSET:0x0010
    
	switch(trig_state){
		case 0:
			// stm32_gpiosetevent(_gpio_tab.adc_port, true, false, false, nullptr);
			//stm32_gpiowrite(_gpio_tab.sw_a_port,false);
			//stm32_gpiowrite(_gpio_tab.sw_b_port,false);	
			trig_state = 1;
			ticks = 0;
			break;
		case 1:
			ticks++;
			if(ticks >= 20){
				trig_state = 2;
				ticks = 0;
                // putreg16(0x0145,STM32_TIM8_DIER);// putreg16(0x0145,0x4001040c);//TIM8 STM32_TIM8_BASE:0x40010400 STM32_GTIM_DIER_OFFSET:0x000c  1 0100 0101
               if ((*pdev_id) == MA40H1S_ID_PRIMARY) {
                    io_timer_set_enable(true, IOTimerChanMode_PWMOut, 0b00110000); // PWM-ch5 PWM-ch6
               }
               else if ((*pdev_id) == MA40H1S_ID_EXPANSION) {
                    io_timer_set_enable(true, IOTimerChanMode_PWMOut, 0b11000000); // PWM-ch7 PWM-ch8
               }                  
			}
			break;
		case 2:
			ticks++;
			if(ticks >= 40){
				// putreg16(0x0000,0x4001040c);
                // stm32_gpiowrite(_gpio_tab.sw_b_port,true);
               // stm32_gpiowrite(_gpio_tab.sw_b_port,true);
               // stm32_gpiowrite(_gpio_tab.sw_a_port,true);
                // stm32_gpiowrite(_dr_a_port,false);
                // stm32_gpiowrite(_dr_b_port,false);
                if ((*pdev_id) == MA40H1S_ID_PRIMARY) {
                    io_timer_set_enable(false, IOTimerChanMode_PWMOut, 0b00110000);
                }
                else if ((*pdev_id) == MA40H1S_ID_EXPANSION) {
                    io_timer_set_enable(false, IOTimerChanMode_PWMOut, 0b11000000);
                } 
				trig_state = 3;
				ticks = 0;
			}
			break;
		case 3:
			ticks++;
			if(ticks>=40){
				ticks=0;
				_echo_valid = true;
				_echo_count = 0;
				// stm32_gpiosetevent(_gpio_tab.adc_port, true, false, false, sonar_isr);
				trig_state = 4;
                // printf("st:%llu\n", hrt_absolute_time());                
			}
			break;
        case 4:
            // ticks++;
            // if(ticks>=10){
                // ticks = 0;
                //rSR &= ~ADC_SR_EOC;
                //rCR2 |= ADC_CR2_SWSTART;
                // printf("t\n");
            // }
				// // start adc
                rSR &= ~ADC_SR_OVR;
                rSR &= ~ADC_SR_EOC;
                rCR2 |= ADC_CR2_CONT;
                rCR2 |= ADC_CR2_ADON;
                rCR2 |= ADC_CR2_SWSTART;
                _start_time = hrt_absolute_time();
				//PX4_INFO("timer5_interrupt running");
                trig_state = 5;

                break;
		default:
		    if(rSR & ADC_SR_EOC) {
		       uint32_t adc_value;
		       adc_value = rDR;
		       PX4_INFO("adc_value = %d\n", adc_value);
		       rSR &= ~ADC_SR_EOC;
		    }
			//PX4_INFO("timer5_interrupt running");
			break;
	}

	return OK;
}

void
MA40H1S::_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
    if (arg != nullptr) {
        MA40H1S *ps = reinterpret_cast<MA40H1S *>(arg);
        ps->_do_adc_dma_callback(status);
       	printf("et:%llu\n",hrt_absolute_time());
    }
}

void MA40H1S::_do_adc_dma_callback(unsigned status)
{
    trig_state = 5;
    _end_time = hrt_absolute_time();

	rCR2 &= ~ADC_CR2_ADON;
	//printf("_ultrasonic_id = %d\n", _ultrasonic_id);
	printf("_ultrasonic_id\n");
    stm32_dmastart(_adc_dma, _dma_callback, this, false);
    _end_index = sonar_decoder_c((int16_t *)adc_buffer,(uint16_t)ADC_BUFFER_SIZE);
    if(_end_index != 1){
        new_value = true;
    } else {
        new_value = false;
    }
}


/**
 * Local functions in support of the shell command.
 */
namespace ma40h1s
{
#ifdef ERROR
    #undef ERROR
#endif
const int ERROR = -1;

MA40H1S *g_dev;

void start();
void stop();
void test();
void reset();
void trig();
void info();
void test_high();
void test_low();

/**
* Start the driver
*/
void start()
{
    int fd;

    if(g_dev != nullptr){
        errx(1,"already started");
    }

    /* creat the driver */
    g_dev = new MA40H1S();

    if(g_dev == nullptr){
        goto fail;
    }

    if(OK != g_dev->init()){
        goto fail;
    }

    fd = open(MA40H1S_DEVICE_PATH,O_RDONLY);

    if(fd < 0){
        goto fail;
    }

    if(ioctl(fd,SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0){
   	// if(ioctl(fd,RANGEFINDERSINGLEMEASURE, SENSOR_POLLRATE_DEFAULT) < 0){    	
        goto fail;
    }

    exit(0);

fail:

    if(g_dev != nullptr){
        delete g_dev;
        g_dev = nullptr;
    }

    errx(1,"driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
    if (g_dev != nullptr) {
        delete g_dev;
        g_dev = nullptr;

    } else {
        errx(1, "driver not running");
    }

    exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void test()
{
    struct distance_sensor_s report;
    ssize_t sz;
    int ret;

    int fd = open(MA40H1S_DEVICE_PATH,O_RDONLY);

    if(fd < 0 ){
        err(1,"open failed");
    }

    sz = read(fd, &report, sizeof(report));

    if(sz != sizeof(report)){
        err(1,"immediate read failed");
    }

    warnx("single read");
    warnx("time:        %llu",report.timestamp);

    /* start the sensor polling at 2Hz */
    if(OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)){
        errx(1, "failed to set 2Hz poll rate");
    }

    for (unsigned i = 0; i < 5; i++)
    {
        struct pollfd fds;

        fds.fd = fd;
        fds.events = POLLIN;
        ret = poll(&fds,1,2000);

        if(ret != 1){
            errx(1,"timed out waiting for sensor data");
        }

        /* now go get it */
        sz = read(fd, & report, sizeof(report));

        if(sz != sizeof(report)){
            err(1,"periodic read failed");
        }

        warnx("periodic read %u", i);
        warnx("time:        %llu", report.timestamp);
    }

    /* reset the sensor polling to default rate */
    if(OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)){
        errx(1,"failed to set default poll rate");
    }

    errx(0,"PASS");
}

/**
 * Reset the driver.
 */
 void reset()
 {
    int fd = open(MA40H1S_DEVICE_PATH,O_RDONLY);

    if(fd < 0 ){
        err(1,"failed ");
    }

    if(ioctl(fd, SENSORIOCRESET, 0) < 0){
        err(1,"driver reset failed");
    }

    if(ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0){
        err(1, "driver poll restart failed");
    }

    exit(0);
 }

 void trig()
 {
    if(g_dev == nullptr){
        errx(1,"driver not running");
    }

    g_dev->trig();
    exit(0);
 }

 void test_high()
 {
    if(g_dev == nullptr){
        errx(1,"driver not running");
    }

    g_dev->test_high();
    exit(0);    
 }

 void test_low()
 {
    if(g_dev == nullptr){
        errx(1,"driver not running");
    }

    g_dev->test_low();
    exit(0);    
 }


/**
 * Print a little info about the driver.
 */
 void info()
 {
    if(g_dev == nullptr){
        errx(1,"driver not running");
    }

    printf("stage @ %p\n",g_dev);
    g_dev->print_info();

    exit(0);
 }

}/* namespace */

// Handle the echo GPIO interrupt
// static int sonar_isr(int irq, void *context)
// {
//     // static int i = 0;
//     hrt_fabsftime time = hrt_fabsfolute_time();
//     // if(i++ > 80){
//     //  i = 0;
//     //  printf("i%llu\n", time);
//     // }
//     // printf("o\n");
//     if(ma40h1s::g_dev != nullptr){
//         ma40h1s::g_dev->interrupt(time);
//     }

//     return OK;
// }


int ma40h1s_main(int argc, char *argv[])
{
    /*
     * Start/load the driver.
     */
    if (!strcmp(argv[1], "start")) {
        ma40h1s::start();
    }

    /*
     * Stop the driver
     */
    if (!strcmp(argv[1], "stop")) {
        ma40h1s::stop();
    }

    /*
     * Test the driver/device.
     */
    if (!strcmp(argv[1], "test")) {
        ma40h1s::test();
    }

    /*
     * Reset the driver.
     */
    if (!strcmp(argv[1], "reset")) {
        ma40h1s::reset();
    }

    // send a trig signal
    if (!strcmp(argv[1], "trig")) {
        ma40h1s::trig();
    }

    if (!strcmp(argv[1], "high")) {
        ma40h1s::test_high();
    }

    if (!strcmp(argv[1], "low")) {
        ma40h1s::test_low();
    }

    /*
     * Print driver information.
     */
    if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
        ma40h1s::info();
    }

    errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");  
}
