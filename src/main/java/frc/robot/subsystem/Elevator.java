package frc.robot.subsystem;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystem.ElevatorConstants.*;

/**
 * Class for testing motion profiling on Maelstrom.
 */
public class Elevator extends SubsystemBase implements AutoCloseable {
    // TODO: Add simulation here.

    private final TalonFX topMotor = new TalonFX(ElevatorConstants.TOP_MOTOR_CAN_ID);
    private final TalonFX bottomMotor = new TalonFX(ElevatorConstants.BOTTOM_MOTOR_CAN_ID);

    private final TalonFXSimState topMotorSimState = topMotor.getSimState();

    // Cache the control request so we don't have to keep making a new object everytime.
    private final PositionVoltage motorControlScheme;

    private ElevatorMotorPosition currentPositionSetpoint;

    private final TrapezoidProfile motionProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            ElevatorConstants.MAX_ELEVATOR_VELOCITY.in(Units.RotationsPerSecond),
            ElevatorConstants.MAX_ELEVATOR_ACCELERATION.in(Units.RotationsPerSecondPerSecond)
        ));

    // Used for motion profiling.
    private final Timer motionProfilingTimer = new Timer();

    // NT publishers for logging
    private final DoublePublisher topMotorRotationPublisher; // In Rotations
    private final DoublePublisher bottomMotorRotationPublisher; // In Rotations
    private final DoublePublisher topMotorVelocityPublisher; // In RotationsPerSecond
    private final DoublePublisher bottomMotorVelocityPublisher; // In RotationsPerSecond

    private final DoublePublisher topMotorVoltagePublisher;
    private final DoublePublisher bottomMotorVoltagePublisher;

    private final DoublePublisher motionProfilingRotationSetpointPublisher; // In Rotations
    private final DoublePublisher motionProfilingVelocitySetpointPublisher; // In RotationsPerSecond

    private final StringPublisher sysIDStatePublisher;

    private Angle motionProfilingRotationSetpoint;
    private AngularVelocity motionProfilingVelocitySetpoint;

    private ElevatorSim elevatorSim;

    private Mechanism2d elevatorMechanism;
    private MechanismRoot2d elevatorMechanismRoot;
    private MechanismLigament2d elevatorMechanismLigament;


    public Elevator(NetworkTableInstance ntInstance) {
        // Keep track of the network table where data from the elevator will be logged.
        NetworkTable elevatorLoggingNT = ntInstance.getTable("Subsystems/Elevator");
        // Motor Rotations
        topMotorRotationPublisher = elevatorLoggingNT.getDoubleTopic("Top Motor Rotations (rots)").publish();
        bottomMotorRotationPublisher = elevatorLoggingNT.getDoubleTopic("Bottom Motor Rotations (rots)").publish();
        // Motor Velocity
        topMotorVelocityPublisher = elevatorLoggingNT.getDoubleTopic("Top Motor Velocity (rots per sec)").publish();
        bottomMotorVelocityPublisher = elevatorLoggingNT.getDoubleTopic("Bottom Motor Velocity (rots per sec)").publish();
        // Motor Voltage
        topMotorVoltagePublisher = elevatorLoggingNT.getDoubleTopic("Top Motor Voltage (volts)").publish();
        bottomMotorVoltagePublisher = elevatorLoggingNT.getDoubleTopic("Bottom Motor Voltage (volts)").publish();


        // Setpoints from the motor profiling so we can log them.
        motionProfilingRotationSetpoint = Rotation.of(0);
        motionProfilingVelocitySetpoint = RotationsPerSecond.of(0);

        motionProfilingRotationSetpointPublisher = elevatorLoggingNT.getDoubleTopic("Motion Profiling Rotations (rots)").publish();
        motionProfilingVelocitySetpointPublisher = elevatorLoggingNT.getDoubleTopic("Motion Profiling Velocity (rots per sec)").publish();

        sysIDStatePublisher = elevatorLoggingNT.getStringTopic("SYS ID State").publish();

        topMotor.getConfigurator().apply(ElevatorConstants.MOTOR_CONFIG);
        bottomMotor.getConfigurator().apply(ElevatorConstants.MOTOR_CONFIG);

        topMotorSimState.Orientation = ChassisReference.Clockwise_Positive;

        // No need to specifically instruct the bottom motor.
        bottomMotor.setControl(new Follower(ElevatorConstants.TOP_MOTOR_CAN_ID, MotorAlignmentValue.Aligned));

        currentPositionSetpoint=ElevatorMotorPosition.BASE;

        motorControlScheme = new PositionVoltage(currentPositionSetpoint.getAngleOfMotor());

        if (Robot.isSimulation()) {
            elevatorSim = constructElevatorSim();

            var subsystemNetworkTableName = getSubsystemNameFromNetworkTable(elevatorLoggingNT);

            // Todo: add a method to resize elevatorMechanism and change its background color.
            // Note: Width and height are in meters.
            elevatorMechanism = new Mechanism2d(MAXIMUM_ELEVATOR_HEIGHT.in(Meters)*1.5, MAXIMUM_ELEVATOR_HEIGHT.in(Meters)*1.75, new Color8Bit(Color.kBlue));
            elevatorMechanismRoot = elevatorMechanism.getRoot(subsystemNetworkTableName + " Root", MAXIMUM_ELEVATOR_HEIGHT.in(Meters)*0.75,MAXIMUM_ELEVATOR_HEIGHT.in(Meters)*0.5);
            elevatorMechanismLigament = elevatorMechanismRoot.append(new MechanismLigament2d(subsystemNetworkTableName + " Ligament", 0, 90, 5, new Color8Bit(Color.kOrange)));
            // Todo: do the rest of the set up here.

            // Display the elevator mechanism to SmartDashboard
            SmartDashboard.putData(subsystemNetworkTableName + " Sim", elevatorMechanism);
        }
    }

    @Override
    public void simulationPeriodic() {

        topMotorSimState.setSupplyVoltage(12); // This might be unneeded.
        double leaderMotorVoltage = topMotor.getMotorVoltage().getValueAsDouble();
        elevatorSim.setInputVoltage(leaderMotorVoltage);

        elevatorSim.update(0.02);
        topMotorSimState.setRawRotorPosition(convertElevatorLinearToMotorAngular(elevatorSim.getPositionMeters()));
        topMotorSimState.setRotorVelocity(convertElevatorLinearToMotorAngular(elevatorSim.getVelocityMetersPerSecond()));

        elevatorMechanismLigament.setLength(elevatorSim.getPositionMeters());
    }

    /**
     * Converts linear measures from the elevator (i.e. position, velocity) to motor angular values
     * @param elevatorLinearValueMeters Value of the elevators positon (meters)/velocity (meters/sec)
     * @return The value converted for the motor in the gearbox.
     */
    private double convertElevatorLinearToMotorAngular(double elevatorLinearValueMeters) {
        return (elevatorLinearValueMeters/(2*Math.PI*ElevatorConstants.PULLY_RADIUS.in(Meters)))*ElevatorConstants.MOTOR_TO_ELEVATOR_GEARING.getGearRatio();
    }

    /**
     *  Parses the name of the subsystem from a given NetworkTable instance.
     * @param subsystemNetworkTable A reference to the subsystem NetworkTable.
     * @return The right most "folder" in the path, in this case, it will be the name of the subsystem.
     */
    private static String getSubsystemNameFromNetworkTable(NetworkTable subsystemNetworkTable) {
        /*
         The NetworkTable path has the "folders" separated by forward slashes, so we can split it into an array.
         Then we can grab the last entry in the array.
         */
        String[] elevatorNetworkTablePath = subsystemNetworkTable.getPath().split("/");
        return elevatorNetworkTablePath[elevatorNetworkTablePath.length-1];
        // Todo: add unit tests for this method.
    }

    @Override
    public void periodic() {
        // Motor Rotation Logging Update
        topMotorRotationPublisher.accept(topMotor.getPosition().getValue().in(Units.Rotations));
        bottomMotorRotationPublisher.accept(bottomMotor.getPosition().getValue().in(Units.Rotations));
        // Motor Velocity Logging Update
        topMotorVelocityPublisher.accept(topMotor.getVelocity().getValue().in(RotationsPerSecond));
        bottomMotorVelocityPublisher.accept(bottomMotor.getVelocity().getValue().in(RotationsPerSecond));
        // Motor Voltage Logging Update
        topMotorVoltagePublisher.accept(topMotor.getMotorVoltage().getValue().in(Volt));
        bottomMotorVoltagePublisher.accept(bottomMotor.getMotorVoltage().getValue().in(Volt));

        motionProfilingRotationSetpointPublisher.accept(motionProfilingRotationSetpoint.in(Rotations));
        motionProfilingVelocitySetpointPublisher.accept(motionProfilingVelocitySetpoint.in(RotationsPerSecond));
    }

    /**
     * Sets the setpoint that the motor will move toward.
     * @param elevatorMotorPosition Position for the motor to move the carriage to.
     */
    public Command setMotorPosition(ElevatorMotorPosition elevatorMotorPosition) {
        currentPositionSetpoint=elevatorMotorPosition;

        // The TrapezoidProfile.calculate is a little misleading with its "current" parameter, as it should be called "start" as it is the state of the system at the beginning (and set only once).
        TrapezoidProfile.State startingState = new TrapezoidProfile.State();
        TrapezoidProfile.State endingState = new TrapezoidProfile.State(elevatorMotorPosition.getAngleOfMotor().in(Units.Rotations), 0);

        // TODO: the "current" param should actually be updated with each loop
        //  We also dont need the "motionProfilingTimer", rather in ".calculate()" it should be 0.02 sec (as per cycle).
        return startRun(
            () -> motionProfilingTimer.restart(),
            () -> {
                startingState.position = topMotor.getPosition().getValue().in(Units.Rotations);
                startingState.velocity = topMotor.getVelocity().getValue().in(Units.RotationsPerSecond);

                // Units of calculatedPosRots will be in rotations.
                TrapezoidProfile.State calculatedPosRots = motionProfile.calculate(
                    MOTION_PROFILING_DELTA_TIME,
                    startingState,
                    endingState);

                motionProfilingRotationSetpoint=Rotations.of(calculatedPosRots.position);
                motionProfilingVelocitySetpoint=RotationsPerSecond.of(calculatedPosRots.velocity);

                // Default unit for .withPosition() and .withVelocity() is rotations.
                topMotor.setControl(motorControlScheme.withPosition(calculatedPosRots.position).withVelocity(calculatedPosRots.velocity));
            }
        ).until(() -> motionProfile.isFinished(motionProfilingTimer.get()));
    }

    /**
     * Controls the subsystem's motors directly with voltage for system identification.
     * @param voltage Voltage to apply to both motors.
     */
    private void controlWithVoltage(Voltage voltage) {
        topMotor.setVoltage(voltage.in(Volts));
    }

    /**
     * Stops both motors and makes them apply a brake.
     */
    public void stop() {
        topMotor.disable();
    }

    @Override
    public void close() {
        topMotor.close();
        bottomMotor.close();

        elevatorMechanismLigament.close();
        elevatorMechanismRoot.close();
        elevatorMechanism.close();
    }

    /**
     * Runs a sysid routine to get the kS and kG constant.
     * @return
     */
    public Command getSysIdRoutine() {
        SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Seconds).of(0.5),
                Volts.of(3),
                Seconds.of(3),
                (state) -> sysIDStatePublisher.accept(state.toString())
            ),
            new SysIdRoutine.Mechanism(
                this::controlWithVoltage,
                null,
                this)
        );

        return new SequentialCommandGroup(
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
                .until(() -> topMotor.getPosition().getValue().in(Units.Rotations) >= 15),
            new WaitCommand(3),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
                .until(() -> topMotor.getPosition().getValue().in(Units.Rotations) <= 0.75),
            new WaitCommand(3),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
                .until(() -> topMotor.getPosition().getValue().in(Units.Rotations) >= 15),
            new WaitCommand(3),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
                .until(() -> topMotor.getPosition().getValue().in(Units.Rotations) <= 0.75)
        );
    }

    /**
     * Positions for the motor to attempt to reach.
     */
    public enum ElevatorMotorPosition {
        BASE(Units.Rotations.of(0)),
        MIDDLE(Units.Rotations.of(8)),
        TEST(Units.Rotations.of(12)),
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
