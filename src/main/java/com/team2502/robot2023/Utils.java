package com.team2502.robot2023;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Utils {
    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to include placing
     * in appropriate scope for CTRE onboard control.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle       Target Angle
     * @return Closest angle within scope
     */
    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    /**
     * Adds a deadzone to a given value
     *
     * @param val the value (controller input)
     * @return if the val is too low returns 0, or the given value
     */
    public static double deadzone(double val) {
        if (val >= 0.075) { return val; }
        else if (val <= -0.075) { return val; }
        else { return 0; }
    }

    /** class for creating trapezoidal curves, used to create a linear transition between two numbers */
    public static class Trapezoidal {
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
            //SmartDashboard.putNumber("trapezoidal deltat", deltaTime);
            //SmartDashboard.putNumber("trapezoidal delta", maxAccel*deltaTime);
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
}

