#include <ros.h>
#include <std_msgs/Float32.h>

/*
 * Defines the control pin for the transistor controlling LED ground. Logic
 * high to this pin will turn the LED strips on.
 */
#define LED_PIN A0

/*
 * Forward declaration of the thruster callback function.
 */
void led_callback(const std_msgs::Float32& msg);

/*
 * Construct a node handle for communicating with the ROS framework to tie in
 * thruster speed to LED intensity.
 */
ros::NodeHandle n;

/*
 * The subscriber to thruster messages.
 */
ros::Subscriber<std_msgs::Float32> sub("/led", &led_callback);

/*
 * Define the minimum PWM duty cycle. Due to the physics of the LED circuit,
 * duty cycles below this value will result in the LEDs not being illuminated.
 */
static constexpr float min_duty_cycle = 0.1;

/*
 * Global variable corresponding to the current duty cycle of the LEDs.
 */
float current_duty = min_duty_cycle;

/**
 * LED message callback.
 *
 * @note This function updates the current_duty variable to set the LED duty
 *       cycle.
 *
 * @param msg The LED PWM message.
 *
 * @return None.
 */
void led_callback(const std_msgs::Float32& msg)
{
    float percentage = msg.data;

    /*
     * Bounds checking (plus logic short-circuit for 0% duty cycle)
     */
    if (percentage <= 0.0)
    {
        current_duty = 0.0;
        return;
    }
    else if (percentage > 1.0)
    {
        percentage = 1.0;
    }

    /*
     * Calculate the pwm duty cycle.
     */

    current_duty = (percentage * (1.0 - min_duty_cycle)) + min_duty_cycle;
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
     * Follow a linear decrease in intensity until the minimal LED intensity is
     * reached.
     */
    for (int i = 100; i > min_duty_cycle*100; --i)
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
