//
// train-barrier-firmware
// ESP32 firmware for train barrier servo motor and light controller
// Copyright (C) 2022 David Norris & Alexander Norris
//

#include <chrono>
#include <cmath>
#include <cstdint>

#include <esp++/errors.hpp>
#include <esp++/gpio.hpp>
#include <esp++/main.hpp>
#include <freertos++/time.hpp>

#include <driver/mcpwm.h>

using namespace std::chrono_literals;

static constexpr int            MOTOR_GPIO = 27;
static constexpr int            SWITCH_GPIO = 25;
static constexpr int            YELLOW_GPIO = 32;
static constexpr int            RED_GPIO = 33;
static constexpr double         ANGLE_VERTICAL = 0.0;
static constexpr double         ANGLE_HORIZONTAL = 90.0;
static constexpr auto           BLINK_INTERVAL = 750ms;
static constexpr std::uint32_t  PWM_FREQUENCY = 50;

static constexpr auto           ON_DELAY = 250ms;
static constexpr auto           YELLOW_DURATION = 250ms;
static constexpr auto           GATE_DELAY = 500ms;
static constexpr auto           TRAVEL_TIME = 5s;

static constexpr double         VERTICAL_PERIOD = 2050.0;
static constexpr double         HORIZONTAL_PERIOD = 1050.0;

static constexpr double ANGLE_TO_PERIOD(double angle)
{
    double f = (angle - ANGLE_HORIZONTAL) / (ANGLE_VERTICAL - ANGLE_HORIZONTAL);
    return HORIZONTAL_PERIOD + f * (VERTICAL_PERIOD - HORIZONTAL_PERIOD);
}

mcpwm_config_t pwm_config =
{
    .frequency      = PWM_FREQUENCY,
    .cmpr_a         = 0,
    .cmpr_b         = 0,
    .duty_mode      = MCPWM_DUTY_MODE_0,
    .counter_mode   = MCPWM_UP_COUNTER,
};

ESPPP_MAIN()
{
    typedef std::chrono::duration<double> duration;

    double current_angle = ANGLE_VERTICAL;
    auto last_loop_time = std::chrono::steady_clock::now();
    auto last_off_time = last_loop_time;

    esp::gpio::set(YELLOW_GPIO, false);
    esp::gpio::set(RED_GPIO, false);
    esp::gpio::configure({ YELLOW_GPIO, RED_GPIO }, GPIO_MODE_OUTPUT);
    esp::gpio::configure(SWITCH_GPIO, GPIO_MODE_INPUT, true);

    esp::check(
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_GPIO),
        "failed to initialize motor gpio");

    esp::check(
        mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config),
        "failed to initialize motor control");

    while (true)
    {
        auto now = std::chrono::steady_clock::now();
        auto dt = now - last_loop_time;
        bool switch_on = !esp::gpio::get(SWITCH_GPIO);
        double target_angle = current_angle;

        if (switch_on)
        {
            auto elapsed = now - last_off_time;

            if (elapsed < ON_DELAY)
            {
                esp::gpio::set(YELLOW_GPIO, false);
                esp::gpio::set(RED_GPIO, false);
                target_angle = ANGLE_VERTICAL;
            }

            else if (elapsed < ON_DELAY + YELLOW_DURATION)
            {
                esp::gpio::set(YELLOW_GPIO, true);
                esp::gpio::set(RED_GPIO, false);
                target_angle = ANGLE_VERTICAL;
            }

            else if (elapsed < ON_DELAY + YELLOW_DURATION + GATE_DELAY)
            {
                esp::gpio::set(YELLOW_GPIO, false);
                esp::gpio::set(RED_GPIO, true);
                target_angle = ANGLE_VERTICAL;
            }

            else if (elapsed < ON_DELAY + YELLOW_DURATION + GATE_DELAY + TRAVEL_TIME)
            {
                esp::gpio::set(YELLOW_GPIO, false);
                esp::gpio::set(RED_GPIO, true);

                double f =
                    std::chrono::duration_cast<duration>(elapsed - ON_DELAY - YELLOW_DURATION - GATE_DELAY) /
                    std::chrono::duration_cast<duration>(TRAVEL_TIME);

                target_angle = ANGLE_VERTICAL + f * (ANGLE_HORIZONTAL - ANGLE_VERTICAL);
            }

            else
            {
                esp::gpio::set(YELLOW_GPIO, false);
                esp::gpio::set(RED_GPIO, true);
                target_angle = ANGLE_HORIZONTAL;
            }

        }

        else
        {
            last_off_time = now;

            target_angle = ANGLE_VERTICAL;

            esp::gpio::set(YELLOW_GPIO, false);
            esp::gpio::set(RED_GPIO, current_angle != target_angle);
        }

        if (current_angle < target_angle)
        {
            current_angle += std::chrono::duration_cast<duration>(dt) *
                std::abs(ANGLE_HORIZONTAL - ANGLE_VERTICAL)
                / std::chrono::duration_cast<duration>(TRAVEL_TIME);

            if (current_angle > target_angle)
                current_angle = target_angle;
        }

        else if (current_angle > target_angle)
        {
            current_angle -= std::chrono::duration_cast<duration>(dt) *
                std::abs(ANGLE_HORIZONTAL - ANGLE_VERTICAL)
                / std::chrono::duration_cast<duration>(TRAVEL_TIME);

            if (current_angle < target_angle)
                current_angle = target_angle;
        }

        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
            ANGLE_TO_PERIOD(current_angle));

        last_loop_time = now;

        freertos::sleep(10ms);
    }
}
