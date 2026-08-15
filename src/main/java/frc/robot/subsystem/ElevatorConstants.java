package frc.robot.subsystem;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.GearRatio;

import static edu.wpi.first.units.Units.Meters;

public class ElevatorConstants {

    public static final int TOP_MOTOR_CAN_ID = 16;
    public static final int BOTTOM_MOTOR_CAN_ID = 17;


    public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        .withSlot0(new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0.13978)
            .withKV(0)
            .withKA(0)
            .withKG(0.20163));

    public static ElevatorSim constructElevatorSim() {
        return new ElevatorSim(
            DCMotor.getKrakenX60(2),
            MOTOR_TO_ELEVATOR_GEARING.getGearRatio(),
            CARRIAGE_WEIGHT.in(Units.Kilograms),
            PULLY_RADIUS.in(Meters),
            0,
            MAXIMUM_ELEVATOR_HEIGHT.in(Meters),
            true,
            0);
    }

    // Calculated the Pitch diameter of WCP-0560 divide by 2 for radius
    public static final Distance PULLY_RADIUS = Units.Inches.of(1.75667).div(2);

    // Obtained from CAD
    public static final Mass CARRIAGE_WEIGHT = Units.Pounds.of(13.92);

    // Motors are attached to a 12t gear which turns a 70t gear (which is attached to a 22t gear), simplified to 35/6
    // Note: 35/6 = 0, because of integer division truncation, use doubles as below.
    public static final GearRatio MOTOR_TO_ELEVATOR_GEARING = new GearRatio(12, 70);

    public static final Distance MAXIMUM_ELEVATOR_HEIGHT = Units.Inches.of(15.75);

    // Constants for the Configuration of the Trapezoidal Profile.
    public static final AngularVelocity MAX_ELEVATOR_VELOCITY = Units.RotationsPerSecond.of(5);
    public static final AngularAcceleration MAX_ELEVATOR_ACCELERATION = Units.RotationsPerSecondPerSecond.of(2);

    // Same time period as the RoboRIO cycles
    public static final double MOTION_PROFILING_DELTA_TIME = 0.02;

}
