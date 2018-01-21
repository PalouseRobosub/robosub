#include <ros.h>
#include <robosub/thruster.h>

/*
 * Defines the control pin for the transistor controlling LED ground. Logic
 * high to this pin will turn the LED strips on.
 */
#define LED_PIN A0

/*
 * Forward declaration of the thruster callback function.
 */
void thruster_callback(const robosub::thruster& msg);

/*
 * Construct a node handle for communicating with the ROS framework to tie in
 * thruster speed to LED intensity.
 */
ros::NodeHandle n;

/*
 * The subscriber to thruster messages.
 */
ros::Subscriber<robosub::thruster> sub("/thrusters", &thruster_callback);

/*
 * Define the minimum percentage on. This means the LEDs will never fall below
 * min% of their intensity.
 */
static constexpr int min = 10;

/*
 * Calculates the current duty cycle associated with the minimum on-percentage.
 */
float current_duty = static_cast<float>(min) / 100.0;

/*
 * Stores the current LED percentage. Initialized to min.
 */
int current_percentage = min;

/**
 * Thruster message callback.
 *
 * @note This function updates the LED intensity based upon thruster power.
 *
 * @param msg The thruster message.
 *
 * @return None.
 */
void thruster_callback(const robosub::thruster& msg)
{
    float total = 0;
    for (unsigned int i = 0; i < msg.data_length; ++i)
    {
        total += msg.data[i];
    }

    /*
     * Calculate the current percentage. Because thrusters never go to 100%
     * due to software and hardware limits, weight the thruster power higher
     * than 100%. 50% power will now correlate with LEDs at 100% intensity. As
     * such, ensure that if the software sends invalid values that the
     * percentage is capped at 100.
     */
    current_percentage = total / msg.data_length * 180.0 + min;
    if (current_percentage > 100)
    {
        current_percentage = 100;
    }

    current_duty = static_cast<float>(current_percentage) / 100.0;
    if (current_duty > 1)
    {
        current_duty = 1.0;
    }
}

/**
 * PWMs a pin at the specified frequency for the specified duty cycle.
 *
 * @note This function performs a single cycle of the PWM and must be called
 *       repeatedly to get a dimming effect. The arduino analogWrite() function
 *       will not work because the specified LED_PIN is not compliant with the
 *       output comparator pins.
 *
 * @param pin The pin to PWM.
 * @param duty The duty cycle to PWM. Must be within [0, 1].
 * @param frequency The frequency (in Hz) to run the PWM at.
 *
 * @return None.
 */
void pwm(int pin, float duty, const int frequency = 100)
{
    const float max_time_micros = 1.0 / frequency * 1000000;
    if (duty > 1.0 || duty < 0)
    {
        n.logwarn("Attempted to PWM invalid value.");
        return;
    }

    /*
     * Calculate the on and off delays in microseconds.
     */
    const int on_micros = duty * max_time_micros;
    const int off_micros = max_time_micros - on_micros;

    /*
     * Ensure that the delay is positive before calling delayMicroseconds().
     */
    digitalWrite(pin, HIGH);
    if (on_micros > 0)
    {
        delayMicroseconds(on_micros);
    }

    digitalWrite(pin, LOW);
    if (off_micros > 0)
    {
        delayMicroseconds(off_micros);
    }
}

/**
 * Arduino-compliant setup function.
 *
 * @note Sets up the PWM pin and the ROS node, then waits for a ROS connection.
 *
 * @return None.
 */
void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    n.initNode();
    n.subscribe(sub);

    for (int i = 0; i < 100; ++i)
    {
        /*
         * The initial turn on intensity follows a cosine mapping to be
         * sinusoidal in LED intensity buildup.
         */
        const float duty = (cos(3.14159 * static_cast<float>(i)/100.0) * -1.0 +
                1.0) / 2.0;
        pwm(LED_PIN, duty);
    }

    /*
     * Leave the LEDs on at full intensity for 500ms to show power-on.
     */
    digitalWrite(LED_PIN, HIGH);
    delay(500);

    /*
     * Follow a linear decrease in intensity until the current percentage of
     * LED intensity is reached.
     */
    for (int i = 100; i > current_percentage; --i)
    {
        const float duty = static_cast<float>(i) / 100.0;
        pwm(LED_PIN, duty);
    }

    /*
     * Wait until the ROS node is connected to continue.
     */
    while (n.connected() == false)
    {
        pwm(LED_PIN, current_duty);
        n.spinOnce();
    }
    n.loginfo("Node initialized.");
}

/**
 * Arduino-compliant loop function.
 *
 * @note This function is called continuously.
 *
 * @return None.
 */
void loop()
{
    /*
     * Handle ROS callbacks and update the LED intensity.
     */
    n.spinOnce();
    pwm(LED_PIN, current_duty);
}
