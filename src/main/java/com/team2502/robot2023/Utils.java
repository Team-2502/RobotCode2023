package com.team2502.robot2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.Comparator;

/** class for any math beyond a line or two */
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

    public static Translation3d nearest(Translation3d[][] transforms, Translation3d input) {
        Translation3d nearest = input;
        double nearestDistance = Double.MAX_VALUE;

        for (Translation3d[] transformRow : transforms) {
            for (Translation3d transform : transformRow) {
                double distance = transform.getDistance(input);
                if (distance < nearestDistance) {
                    nearestDistance = distance;
                    nearest = transform;
                }
            }
        }
        return nearest;
    }

   /* 2022 import
    * Get distance to target from elevation using trigonometry
    * @param camHeight distance from ground to camera aperture
    * @param camElevation angle of camera
    * @param basketHeight distance from ground to basket vision targets
    * @param targetElevation angle of target relative to camera
    * @return distance to target
    */
    public static double findDist(double camHeight, double camElevation, double basketHeight, double targetElevation) {
	    return (
			    (basketHeight - camHeight)
			    /
			    Math.tan(Math.toRadians(targetElevation+ camElevation))
		   );
    }

    /**
     * map x from one deadzone to another
     * @param minInput start of input deadzone
     * @param minOutput start of mapped deadzone
     * @param x value to map
     * @return transformed x
    */
    public static double deadzone(double minInput, double minOutput, double x) {
        boolean sign = x > 0.0;
        x = sign ? x : -x;
        double inputRange = 1.0 - minInput;
        double outpuRange = 1.0 - minOutput;

        // below input deadzone return 0
        if (x < minInput) {
            return 0;
        }

        // this is some kind of affine transformation
        double inputAdjusted = (x-minInput) * inputRange;

        return inputAdjusted / outpuRange + minOutput;
    }
}
