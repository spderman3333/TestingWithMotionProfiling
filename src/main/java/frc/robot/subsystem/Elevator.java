package frc.robot.subsystem;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {

    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));

    private final TalonFX topMotor = new TalonFX(ElevatorConstants.TOP_MOTOR_CAN_ID);
    private final TalonFX bottomMotor = new TalonFX(ElevatorConstants.BOTTOM_MOTOR_CAN_ID);

    // Cache the control request so we dont have to keep making a new object everytime.
    private final PositionVoltage motorControlScheme;

    private ElevatorMotorPosition currentPositionSetpoint;

    private TrapezoidProfile motionProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            ElevatorConstants.MAX_ELEVATOR_VELOCITY.in(Units.RotationsPerSecond),
            ElevatorConstants.MAX_ELEVATOR_ACCELERATION.in(Units.RotationsPerSecondPerSecond)
        ));

    // Used for motion profiling.
    private Timer motionProfilingTimer = new Timer();

    // Keep track of the network table where data from the elevator will be logged.
    private NetworkTable elevatorLoggingNT;
    // NT publishers for logging
    private DoublePublisher topMotorRotationPublisher; // In Rotations
    private DoublePublisher bottomMotorRotationPublisher; // In Rotations


    public Elevator(NetworkTableInstance ntInstance) {
        elevatorLoggingNT = ntInstance.getTable("Subsystems/Elevator");
        topMotorRotationPublisher = elevatorLoggingNT.getDoubleTopic("Top Motor Rotations (rots)").publish();
        bottomMotorRotationPublisher = elevatorLoggingNT.getDoubleTopic("Bottom Motor Rotations (rots)").publish();

        topMotor.getConfigurator().apply(motorConfig);
        bottomMotor.getConfigurator().apply(motorConfig);

        // No need to specifically instruct the bottom motor.
        bottomMotor.setControl(new Follower(ElevatorConstants.TOP_MOTOR_CAN_ID, MotorAlignmentValue.Aligned));

        currentPositionSetpoint=ElevatorMotorPosition.BASE;

        motorControlScheme = new PositionVoltage(currentPositionSetpoint.getAngleOfMotor());
    }


    @Override
    public void periodic() {
        topMotorRotationPublisher.accept(topMotor.getPosition().getValue().in(Units.Rotations));
        bottomMotorRotationPublisher.accept(bottomMotor.getPosition().getValue().in(Units.Rotations));



    }

    /**
     * Sets the setpoint that the motor will move toward.
     * @param elevatorMotorPosition Position for the motor to move the carriage to.
     */
    public Command setMotorPosition(ElevatorMotorPosition elevatorMotorPosition) {
        currentPositionSetpoint=elevatorMotorPosition;

        // The TrapezoidProfile.calculate is a little missleading with its "current" parameter, as it should be called "start" as it is the state of the system at the beginning (and set only once).
        TrapezoidProfile.State startingState = new TrapezoidProfile.State(topMotor.getPosition().getValue().in(Units.Rotations), topMotor.getVelocity().getValue().in(Units.RotationsPerSecond));
        TrapezoidProfile.State endingState = new TrapezoidProfile.State(elevatorMotorPosition.getAngleOfMotor().in(Units.Rotations), 0);

        return startRun(
            ()->{
            motionProfilingTimer.restart();
            },
            ()->{
                double timeSinceStartOfControl = motionProfilingTimer.get();
                // Units of calculatedPosRots will be in rotations.
                TrapezoidProfile.State calculatedPosRots = motionProfile.calculate(
                    timeSinceStartOfControl,
                    startingState,
                    endingState);

                // Default unit for .withPosition() and .withVelocity() is rotations.
                topMotor.setControl(motorControlScheme.withPosition(calculatedPosRots.position).withVelocity(calculatedPosRots.velocity));
            }
        );
    }

    /**
     * Nested class to allow for quick configuration of the constants in the Elevator code.
     */
    public static class ElevatorConstants {
        public static final int TOP_MOTOR_CAN_ID = 16;
        public static final int BOTTOM_MOTOR_CAN_ID = 17;

        // Constants for the Configuration of the Trapezoidal Profile.
        public static final AngularVelocity MAX_ELEVATOR_VELOCITY = Units.RotationsPerSecond.of(5);
        public static final AngularAcceleration MAX_ELEVATOR_ACCELERATION = Units.RotationsPerSecondPerSecond.of(2);

        // Same time period as the RoboRIO cycles
        public static final double MOTION_PROFILING_DELTA_TIME = 0.02;
    }

    /**
     * Positions for the motor to attempt to reach.
     */
    public enum ElevatorMotorPosition {
        BASE(Units.Rotations.of(0)),
        TOP(Units.Rotations.of(16));

        // Angle of the RAW motor, no gear ratios taken into account.
        private final Angle angleOfMotor;

        ElevatorMotorPosition(Angle position) {
            angleOfMotor = position;
        }

        public Angle getAngleOfMotor() {
            return angleOfMotor;
        }
    }
}
