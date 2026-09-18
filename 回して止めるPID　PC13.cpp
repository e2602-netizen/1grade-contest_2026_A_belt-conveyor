#include "Config.hpp"
#include "Timer.hpp"
#include "main.h"
#include <cstdio>
#include <cmath>
#include "Motor.hpp"
#include "Encoder.hpp"
#include "Interrupts.hpp"  
#include "tim.h"

constexpr double M_PI_VAL = 3.14159265358979323846;


constexpr float Kp = 0.003f;
constexpr float Ki = 0.001f;
constexpr float Kd = 0.0001f;

constexpr double ROLL_AMOUNT = 0.1;
constexpr double BELT_ONE_ROLL = 100.0 / 10.0;
constexpr double ENC_RES = 512.0;
constexpr int32_t TARGET_PULSE = static_cast<int32_t>(BELT_ONE_ROLL * ROLL_AMOUNT * ENC_RES);

int cpp_main()
{
    main_timer::activate();
    cycle::dt = 0.01f; 

    Motor motor(&htim1, TIM_CHANNEL_1, &htim1, TIM_CHANNEL_2, POSITIVE, POSITIVE);
    Encoder_SINGLE_Interrupt encoder(GPIOB, GPIO_PIN_1, GPIOB, GPIO_PIN_2, 512, POSITIVE);

    float resolution = static_cast<float>(motor.get_resolution());

    float pwm = 0.0f;
    constexpr float target_rpm = 50.0f;
    
    float integral = 0.0f;
    float e_prev = 0.0f;
    float rpm_filtered = 0.0f;

    bool is_running = false;
    bool last_button_pressed = false;

    Timer timer;
    float elapsed_time = 0.0f;
    timer.reset();
    timer.start();

    motor.set_value(0);

    while (true)
    {
        bool button_pressed = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET);

        if (button_pressed && !last_button_pressed && !is_running)
        {
            is_running = true;
            encoder.reset();
            pwm = 0.0f;
            integral = 0.0f;
            e_prev = 0.0f;
            rpm_filtered = 0.0f;
        }

        if (is_running)
        {
            encoder.update();
            int32_t current_pulse = std::abs(encoder.get_pulse());

            if (current_pulse < TARGET_PULSE)
            {
                float raw_rpm = encoder.get_omega() * 60.0f / 2.0f / static_cast<float>(M_PI_VAL);
                
                
                rpm_filtered = rpm_filtered * 0.7f + raw_rpm * 0.3f;
                
                float e = target_rpm - rpm_filtered;
                
                
                integral += e * cycle::dt;
                if (integral > 0.3f) integral = 0.3f;
                if (integral < 0.0f) integral = 0.0f;

            
                float derivative = (e - e_prev) / cycle::dt;

                
                pwm = Kp * e + Ki * integral + Kd * derivative;

                if (pwm > 0.4f) pwm = 0.4f;
                if (pwm < 0.0f) pwm = 0.0f;

                motor.set_value(static_cast<int32_t>(resolution * pwm));

                e_prev = e;
            }
            else
            {
                motor.set_value(0);
                pwm = 0.0f;
                integral = 0.0f;
                e_prev = 0.0f;
                is_running = false;
            }
        }
        else
        {
            motor.set_value(0);
        }

        last_button_pressed = button_pressed;

        printf("Pulse: %ld / %ld | RPM: %.1f | PWM: %.2f\n", 
               static_cast<long>(std::abs(encoder.get_pulse())), 
               static_cast<long>(TARGET_PULSE), 
               rpm_filtered,
               pwm);

        while (timer.read() - elapsed_time < cycle::dt) {};
        elapsed_time = timer.read();
    }

    return 0;
}