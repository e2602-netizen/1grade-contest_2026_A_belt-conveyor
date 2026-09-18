#include "Config.hpp"
#include "Motor.hpp"
#include "PWM.hpp"
#include "main.h"
#include "tim.h"
#include "fdcan.h"
#include "CAN.hpp"
#include "Encoder.hpp"
#include "Interrupts.hpp"
#include "Timer.hpp"
#include <cstdint>
#include <cmath>

//0x001
volatile uint8_t g_air_cmd        = 1; // data[0]: エアシリ(0:開く, 1:閉じる)
volatile uint8_t g_conv_large_cmd = 1; // data[1]: ベルコン大 (0:pos1, 1:pos1, 2:pos2)
volatile uint8_t g_conv_small_cmd = 1; // data[2]: ベルコン小(0:pos0, 1:pos1, 2:pos2)

//0x002
uint8_t g_air_status        = 1; // data[0]: エアシリ(0:開完了, 1:閉完了)
uint8_t g_conv_large_status = 1; // data[1]: ベルコン大 (0:pos1, 1:pos1, 2:pos2)
uint8_t g_conv_small_status = 1; // data[2]: ベルコン小(0:pos0, 1:pos1, 2:pos2)


volatile bool g_conv_large_updated = false;
volatile bool g_conv_small_updated = false;


// 0x002関数
void send_status_to_base()
{
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 0x002;               
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_3;   
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    uint8_t TxData[3];
    TxData[0] = g_air_status;        
    TxData[1] = g_conv_large_status; 
    TxData[2] = g_conv_small_status; 

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}


//0x001関数
void USER_HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan2) {
        uint32_t id = RxMessage.RxHeader.Identifier;

        if (id == 0x001) {
            g_air_cmd        = RxMessage.RxData[0]; 
            g_conv_large_cmd = RxMessage.RxData[1]; 
            g_conv_small_cmd = RxMessage.RxData[2]; 

            g_conv_large_updated = true;
            g_conv_small_updated = true;
        }
    }
}


//ベルコン
struct Conveyor {
    Motor* motor;
    Encoder_SINGLE_Interrupt* encoder;

    bool is_running = false;

    double belt_pu_ri_ = 100.0;
    double gia_pu_ri_ = 10.0;
    double encoder_resolution = 512.0;

    double roll_positions[3] = {0.3, 0.3, 0.4}; 

    
    float kp = 0.005f;
    float ki = 0.002f;
    float kd = 0.000f;

    float base_max_pwm = 0.1f; 
    float high_max_pwm = 0.4f; 

    int32_t target_pulse = 0;
    float current_max_pwm = 0.5f;

    float integral = 0.0f;
    float e_prev = 0.0f;

    void setup(double belt, double gia, double r0, double r1, double r2, 
               float p, float i, float d, 
               float base_pwm = 0.5f, float high_pwm = 0.85f) 
    {
        belt_pu_ri_ = belt;
        gia_pu_ri_ = gia;
        roll_positions[0] = r0;
        roll_positions[1] = r1;
        roll_positions[2] = r2;
        
        kp = p; ki = i; kd = d; 
        base_max_pwm = base_pwm;
        high_max_pwm = high_pwm;
    }

    void update(uint8_t pos_cmd, bool is_updated) {
        encoder->update();

        if (is_updated) {
            if (pos_cmd <= 2) {
                double belt_one_roll = belt_pu_ri_ / gia_pu_ri_;
                double roll_amount = roll_positions[pos_cmd];
                int32_t pulse_count = static_cast<int32_t>(belt_one_roll * roll_amount * encoder_resolution);

                if (pos_cmd == 0) {
                    target_pulse = -pulse_count;     // 0:pos0(逆転）
                    current_max_pwm = base_max_pwm;
                } else if (pos_cmd == 1) {
                    target_pulse = pulse_count;      // 1:pos1(正転)
                    current_max_pwm = base_max_pwm;
                } else if (pos_cmd == 2) {
                    target_pulse = pulse_count;      // 2:pos2(正転)
                    current_max_pwm = high_max_pwm;
                }

                is_running = true;
                integral = 0.0f;
                e_prev = 0.0f;
            }
        }

        if (is_running) {
            int32_t current_pulse = encoder->get_pulse();
            float e = static_cast<float>(target_pulse - current_pulse);

            integral += e * cycle::dt;
            if (integral > 1000.0f)  integral = 1000.0f;
            if (integral < -1000.0f) integral = -1000.0f;

            float derivative = (e - e_prev) / cycle::dt;
            e_prev = e;

            float pwm = (kp * e) + (ki * integral) + (kd * derivative);

            if (pwm > current_max_pwm)  pwm = current_max_pwm;
            if (pwm < -current_max_pwm) pwm = -current_max_pwm;

            if (std::abs(e) < 5.0f) {
                stop();
            } else {
                double resolution = static_cast<double>(motor->get_resolution());
                motor->set_value(static_cast<int32_t>(resolution * pwm));
            }
        } else {
            motor->set_value(0);
        }
    }

    void stop() {
        motor->set_value(0);
        is_running = false;
        integral = 0.0f;
        e_prev = 0.0f;
    }
};

//エアシリ
struct AirCylinder {
    PwmOut* pwm;

    void set_state(uint8_t cmd) {
        if (cmd == 0) {
            pwm->set_duty_cycle(5.0 / 26.0); 
        } else {
            pwm->set_duty_cycle(0.0);        
        }
    }
};


int cpp_main()
{
    main_timer::activate();
    cycle::dt = 0.05f;

    // ベルコン大
    Motor motor_large(&htim1, TIM_CHANNEL_1, &htim1, TIM_CHANNEL_2, POSITIVE, POSITIVE); 
    Encoder_SINGLE_Interrupt encoder_large(GPIOA, GPIO_PIN_1, GPIOA, GPIO_PIN_2, 512, POSITIVE);

    // ベルコン小right
    Motor motor_small1(&htim1, TIM_CHANNEL_3, &htim1, TIM_CHANNEL_4, POSITIVE, POSITIVE); 
    Encoder_SINGLE_Interrupt encoder_small1(GPIOA, GPIO_PIN_3, GPIOA, GPIO_PIN_4, 512, POSITIVE);

    // ベルコン小left
    Motor motor_small2(&htim1, TIM_CHANNEL_5, &htim1, TIM_CHANNEL_6, POSITIVE, POSITIVE); 
    Encoder_SINGLE_Interrupt encoder_small2(GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1, 512, POSITIVE);

    // ベルコン大: (ベルト, ギア, pos0回転, pos1回転, pos2回転, Kp, Ki, Kd, 通常PWM, 高速PWM)
    Conveyor conv_large = {&motor_large, &encoder_large};
    conv_large.setup(136.0, 30.0, 0.3, 0.3, 0.4,  0.005f, 0.002f, 0.000f, 0.1f, 0.4f);

    // ベルコン小right: (ベルト, ギア, pos0回転, pos1回転, pos2回転, Kp, Ki, Kd, 通常PWM, 高速PWM)
    Conveyor conv_small1 = {&motor_small1, &encoder_small1};
    conv_small1.setup(102.0, 30.0, 0.3, 0.3, 0.4,  0.005f, 0.002f, 0.000f, 0.1f, 0.4f);

    //// ベルコン小left(ベルト, ギア, pos0回転, pos1回転, pos2回転, Kp, Ki, Kd, 通常PWM, 高速PWM)
    Conveyor conv_small2 = {&motor_small2, &encoder_small2};
    conv_small2.setup(102.0, 30.0, 0.3, 0.3, 0.4,  0.005f, 0.002f, 0.000f, 0.1f, 0.4f);

    
    PwmOut pwm_air1(&htim1, TIM_CHANNEL_1, POSITIVE);
    pwm_air1.activate();
    AirCylinder air1 = {&pwm_air1};

    PwmOut pwm_air2(&htim1, TIM_CHANNEL_2, POSITIVE);
    pwm_air2.activate();
    AirCylinder air2 = {&pwm_air2};


    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    CAN_ns::set_all_pass_filter(&hfdcan2);

    Timer timer;
    timer.reset();
    timer.start();
    double start_time = 0.0;

    uint8_t prev_air_cmd = 0xFF;
    bool conv_large_waiting = false;
    bool conv_small_waiting = false;

    
    while (true)
    {
      //エアシリ
        if (g_air_cmd != prev_air_cmd) {
            prev_air_cmd = g_air_cmd;

            air1.set_state(g_air_cmd);
            air2.set_state(g_air_cmd);

            g_air_status = g_air_cmd;
            send_status_to_base();
        }

        //ベルコン大
        if (g_conv_large_updated) {
            conv_large.update(g_conv_large_cmd, true);
            g_conv_large_updated = false;
            conv_large_waiting = true;
        } else {
            conv_large.update(g_conv_large_cmd, false);
        }

        if (conv_large_waiting && !conv_large.is_running) {
            conv_large_waiting = false;
            g_conv_large_status = g_conv_large_cmd;
            send_status_to_base();
        }

        //ベルコン小 
        if (g_conv_small_updated) {
            conv_small1.update(g_conv_small_cmd, true);
            conv_small2.update(g_conv_small_cmd, true);
            g_conv_small_updated = false;
            conv_small_waiting = true;
        } else {
            conv_small1.update(g_conv_small_cmd, false);
            conv_small2.update(g_conv_small_cmd, false);
        }

        if (conv_small_waiting && !conv_small1.is_running && !conv_small2.is_running) {
            conv_small_waiting = false;
            g_conv_small_status = g_conv_small_cmd;
            send_status_to_base();
        }

        
        while (timer.read() - start_time < cycle::dt) {};
        start_time = timer.read();
    }

    return 0;
}