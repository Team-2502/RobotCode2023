package com.team2502.robot2023;

import edu.wpi.first.wpilibj.Timer;

/** class for creating trapezoidal curves, used to create a linear transition between two numbers 
 *
 * imported from 2022 */
public class Trapezoidal {
    /** maximum acceleration in units per second per second */
    double maxAccel;
    /** maximum deceleration in units per second per second */
    double maxRevAccel;
    /** target value */
    double target;
    /** current speed */
    double speed;
    /** timer for calculating deltas*/
    double lastTime;

    /**
     * trapezoidal initializer
     * @param maxAccel maximum acceleration of trapezoidal in units per second squared
     * @param maxRevAccel maximum deceleration of trapezoidal in units per second squared
     * @param target goal velocity in units per second
     * @param speed current velocity in units per second
     */
    public Trapezoidal (double maxAccel, double maxRevAccel, double target, double speed) {
        this.maxAccel = maxAccel;
        this.maxRevAccel = maxRevAccel;
        this.target = target;
        this.speed = speed;
        this.lastTime = Timer.getFPGATimestamp();
    }

    /**
     * trapezoidal initializer
     * @param maxAccel maximum acceleration and deceleration of trapezoidal in units per second squared
     * @param target goal velocity in units per second
     * @param speed current velocity in units per second
     */
    public Trapezoidal ( double maxAccel, double target, double speed) {
        this.maxAccel = maxAccel;
        this.maxRevAccel = maxAccel;
        this.target = target;
        this.speed = speed;
        this.lastTime = Timer.getFPGATimestamp();
    }

    /**
     * trapezoidal initializer
     * @param maxAccel maximum acceleration and deceleration of trapezoidal in units per second squared
     */
    public Trapezoidal ( double maxAccel) {
        this.maxAccel = maxAccel;
        this.maxRevAccel = maxAccel;
        this.target = 0;
        this.speed = 0;
        this.lastTime = Timer.getFPGATimestamp();
    }

    public void setTarget(double newTarget) {
        this.target = newTarget;
    }

    /**
     * Reset speed and target
     * */
    public void reset() {
        this.speed = 0;
        this.target = 0;
    }

    /**
    * Calculate next iteration of trapezoidal
    * @return adjusted velocity
     */
    public double calculate() {
        double thisTime = Timer.getFPGATimestamp();
        double deltaTime = thisTime - lastTime;
        this.lastTime = thisTime;
        if (speed < target) { // accelerate if speed less than target
            speed += deltaTime*maxAccel; // add maximum
            speed = Math.min(speed,target); // constrain
        } else if (speed > target) { // decelerate if speed more than target
            speed -= deltaTime*maxRevAccel; // subtract maximum
            speed = Math.max(speed,target); // constrain
        }
        return speed;
    }

    /**
    * Calculate next iteration of trapezoidal
    * @param target setpoint
    * @return adjusted velocity
     */
    public double calculate(double target) {
        setTarget(target);
        return calculate();
    }
}
