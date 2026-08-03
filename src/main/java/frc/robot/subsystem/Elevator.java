package frc.robot.subsystem;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {

    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));

    private final TalonFX topMotor = new TalonFX(ElevatorConstants.TOP_MOTOR_CAN_ID);
    private final TalonFX bottomMotor = new TalonFX(ElevatorConstants.BOTTOM_MOTOR_CAN_ID);


    private TrapezoidProfile motionProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            ElevatorConstants.MAX_ELEVATOR_VELOCITY.in(Units.RotationsPerSecond),
            ElevatorConstants.MAX_ELEVATOR_ACCELERATION.in(Units.RotationsPerSecondPerSecond)
        ));

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
    }


    @Override
    public void periodic() {
        topMotorRotationPublisher.accept(topMotor.getPosition().getValue().in(Units.Rotations));
        bottomMotorRotationPublisher.accept(bottomMotor.getPosition().getValue().in(Units.Rotations));



    }

    // TODO: Make this command run the motion profiling.
    //  See https://docs.wpilib.org/en/stable/docs/software/commandbased/profile-subsystems-commands.html#motion-profiling-in-command-based
    private void updateMotionProfilingState() {
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

    }

    /**
     * Positions for the motor to attempt to reach.
     */
    public static enum ElevatorMotorPositions {
        BASE(Units.Rotations.of(0)),
        TOP(Units.Rotations.of(16));
        // TODO: Update TOP with the rotations of the encoder/motor when the elevator is up.

        // Angle of the RAW motor, no gear ratios taken into account.
        private final Angle angleOfMotor;

        ElevatorMotorPositions(Angle position) {
            angleOfMotor = position;
        }

        public Angle getAngleOfMotor() {
            return angleOfMotor;
        }
    }
}
