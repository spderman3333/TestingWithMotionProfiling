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
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;


import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystem.ElevatorConstants.*;

/**
 * Class for testing motion profiling on Maelstrom.
 */
public class Elevator extends SubsystemBase {
    // TODO: Add simulation here.

    private final TalonFX topMotor = new TalonFX(ElevatorConstants.TOP_MOTOR_CAN_ID);
    private final TalonFX bottomMotor = new TalonFX(ElevatorConstants.BOTTOM_MOTOR_CAN_ID);

    private final TalonFXSimState topMotorSimState = topMotor.getSimState();

    // Cache the control request so we don't have to keep making a new object everytime.
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

    private ElevatorSim elevatorSim;

    private Mechanism2d elevatorMechanism;
    private MechanismRoot2d elevatorMechanismRoot;
    private MechanismLigament2d elevatorMechanismLigament;


    public Elevator(NetworkTableInstance ntInstance) {
        elevatorLoggingNT = ntInstance.getTable("Subsystems/Elevator");
        topMotorRotationPublisher = elevatorLoggingNT.getDoubleTopic("Top Motor Rotations (rots)").publish();
        bottomMotorRotationPublisher = elevatorLoggingNT.getDoubleTopic("Bottom Motor Rotations (rots)").publish();

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
        topMotorRotationPublisher.accept(topMotor.getPosition().getValue().in(Units.Rotations));
        bottomMotorRotationPublisher.accept(bottomMotor.getPosition().getValue().in(Units.Rotations));

    }

    /**
     * Sets the setpoint that the motor will move toward.
     * @param elevatorMotorPosition Position for the motor to move the carriage to.
     */
    public Command setMotorPosition(ElevatorMotorPosition elevatorMotorPosition) {
        currentPositionSetpoint=elevatorMotorPosition;

        // The TrapezoidProfile.calculate is a little misleading with its "current" parameter, as it should be called "start" as it is the state of the system at the beginning (and set only once).
        TrapezoidProfile.State startingState = new TrapezoidProfile.State(topMotor.getPosition().getValue().in(Units.Rotations), topMotor.getVelocity().getValue().in(Units.RotationsPerSecond));
        TrapezoidProfile.State endingState = new TrapezoidProfile.State(elevatorMotorPosition.getAngleOfMotor().in(Units.Rotations), 0);

        // TODO: the "current" param should actually be updated with each loop
        //  We also dont need the "motionProfilingTimer", rather in ".calculate()" it should be 0.02 sec (as per cycle).
        return startRun(
            () -> motionProfilingTimer.restart(),
            () -> {
                double timeSinceStartOfControl = motionProfilingTimer.get();
                // Units of calculatedPosRots will be in rotations.
                TrapezoidProfile.State calculatedPosRots = motionProfile.calculate(
                    timeSinceStartOfControl,
                    startingState,
                    endingState);

                // Default unit for .withPosition() and .withVelocity() is rotations.
                topMotor.setControl(motorControlScheme.withPosition(calculatedPosRots.position).withVelocity(calculatedPosRots.velocity));
            }
        ).until(() -> motionProfile.isFinished(motionProfilingTimer.get()));
    }

    public Command setMotorPositionWithoutMP(ElevatorMotorPosition elevatorMotorPosition) {
        System.out.println("Set motor Position without MP:" + elevatorMotorPosition.getAngleOfMotor());
        return runOnce(() -> topMotor.setControl(motorControlScheme.withPosition(elevatorMotorPosition.getAngleOfMotor())));
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
