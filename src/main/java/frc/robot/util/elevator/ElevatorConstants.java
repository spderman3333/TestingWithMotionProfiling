package frc.robot.util.elevator;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import frc.robot.util.GearRatio;

/**
 * A class used to store the most frequent variables/constants used in an elevator subsystem.
 *
 */
public class ElevatorConstants {
    // Note that all of these classes have immutable fields/are immutable (including GearRatio),
    //  and are protected in case a subclass wishes to change them.

    // Standard Constants
    /**
     * The maximum height the elevator can achieve.
     */
    protected Distance maxElevatorHeight;

    /**
     * The radius of the drum/pully that drives the elevator.
     */
    protected Distance pulleyRadius;

    /**
     * The gear ratio of the gear that drives the pully to the gear driven by the motor.
     */
    protected GearRatio motorToElevatorGearing;

    /**
     * The weight of the elevator's carriage, can be measured from the CAD or empirically.
     */
    protected Mass carriageWeight;

    // Constants for Motion Profiling
    /**
     * The maximum allowed angular velocity of the motor.
     */
    protected AngularVelocity maxElevatorVelocityMP;

    /**
     * The maximum allowed angular acceleration of the motor.
     */
    protected AngularAcceleration maxElevatorAccelerationMP;

    /**
     *
     * @param maxElevatorHeight The maximum height the elevator can achieve.
     * @param pulleyRadius The radius of the drum/pully that drives the elevator.
     * @param motorToElevatorGearing The gear ratio of the gear that drives the pully to the gear driven by the motor.
     * @param carriageWeight The weight of the elevator's carriage, can be measured from the CAD or empirically.
     * @param maxElevatorVelocityMP The maximum allowed angular velocity of the motor.
     * @param maxElevatorAccelerationMP The maximum allowed angular acceleration of the motor.
     */
    public ElevatorConstants(Distance maxElevatorHeight,
                             Distance pulleyRadius,
                             GearRatio motorToElevatorGearing,
                             Mass carriageWeight,
                             AngularVelocity maxElevatorVelocityMP,
                             AngularAcceleration maxElevatorAccelerationMP) {
        this.maxElevatorHeight = maxElevatorHeight;
        this.pulleyRadius = pulleyRadius;
        this.motorToElevatorGearing = motorToElevatorGearing;
        this.carriageWeight = carriageWeight;
        this.maxElevatorVelocityMP = maxElevatorVelocityMP;
        this.maxElevatorAccelerationMP = maxElevatorAccelerationMP;
    }

    // Getters
    /**
     * The maximum height the elevator can achieve.
     */
    public Distance getMaxElevatorHeight() {
        return maxElevatorHeight;
    }

    /**
     * The radius of the drum/pully that drives the elevator.
     */
    public Distance getPulleyRadius() {
        return pulleyRadius;
    }

    /**
     * The gear ratio of the gear that drives the pully to the gear driven by the motor.
     */
    public GearRatio getMotorToElevatorGearing() {
        return motorToElevatorGearing;
    }

    /**
     * The weight of the elevator's carriage, can be measured from the CAD or empirically.
     */
    public Mass getCarriageWeight() {
        return carriageWeight;
    }

    /**
     * The maximum allowed angular velocity of the motor.
     */
    public AngularVelocity getMaxElevatorVelocityMP() {
        return maxElevatorVelocityMP;
    }

    /**
     * The maximum allowed angular acceleration of the motor.
     */
    public AngularAcceleration getMaxElevatorAccelerationMP() {
        return maxElevatorAccelerationMP;
    }
}
