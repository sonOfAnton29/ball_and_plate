#include<Arduino.h>
class PID {
private:
    float Kp, Ki, Kd;  // PID gains
    float prevError, integral;  // Error tracking
    float minOutput, maxOutput; // Output limits
    float derivativeFilterFactor; // Factor for the derivative filter
      
    float prevMeasuredValue;  // Previous measured value
    float prevDerivative;  // Previous derivative value
    //std::chrono::steady_clock::time_point lastTime;  // Time tracking for dt
    float dt;
    float lastTime;

public:
    PID(float Kp, float Ki, float Kd, float minOut, float maxOut,float dt, float derivativeFilterFactor = 0.5)
        : Kp(Kp), Ki(Ki), Kd(Kd), minOutput(minOut), maxOutput(maxOut), derivativeFilterFactor(derivativeFilterFactor),dt(dt),
        prevError(0), integral(0), prevDerivative(0) {
        //lastTime = std::chrono::steady_clock::now();  // Initialize lastTime
    }

    float compute(float setpoint, float measuredValue,bool direction) {
        unsigned long now  = millis();
        float offset = 45.0f;
        dt = 0.05; // time step
        lastTime = now;

        // if (dt <= 0.0) dt = 1e-6;  // Prevent division by zero
        float error ;
        // Compute error terms
        if( direction == true)
             error = setpoint - measuredValue;
        else
            error = -setpoint + measuredValue;
        if (Ki != 0 )
        {integral += error * dt;}


        // Apply derivative filter
  // float derivative = (measuredValue - prevMeasuredValue/ dt;
        float derivative = (error - prevError) / dt;
        derivative = derivativeFilterFactor * derivative + (1 - derivativeFilterFactor) * prevDerivative;

        // Final PID output with filtered derivative term
        float output = offset + Kp *( error)+ (Ki * integral) + (Kd * derivative);

        // Apply output limits again after calculations
        if (output > maxOutput) output = maxOutput;
        if (output < minOutput) output = minOutput;

        // Save current error and derivative for next cycle
        prevError = error;
        prevDerivative = derivative;
        prevMeasuredValue = measuredValue;
        return output;
    }

    // Set new PID parameters
    void setTunings(float newKp, float newKi, float newKd) {
        Kp = newKp;
        Ki = newKi;
        Kd = newKd;
    }

    // Set the derivative filter factor (0 to 1)
    void setDerivativeFilterFactor(float factor) {
        derivativeFilterFactor = factor;
    }
    void reset()
  {
    prevError = 0; integral = 0; prevDerivative=0;
       //   lastTime = std::chrono::steady_clock::now(); 
  }
};