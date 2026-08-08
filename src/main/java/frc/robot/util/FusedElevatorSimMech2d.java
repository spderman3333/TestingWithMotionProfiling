package frc.robot.util;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

/**
 * A helper class for {@link edu.wpi.first.wpilibj.simulation.ElevatorSim} that also automatically generates a {@link edu.wpi.first.wpilibj.smartdashboard.Mechanism2d} for the {@code ElevatorSim}.
 */
public class FusedElevatorSimMech2d {

    private final ElevatorSim elevatorSim;
    private final NetworkTable elevatorNetworkTable;
    private final String subsystemNetworkTableName;

    private Mechanism2d elevatorMechanism;
    private MechanismRoot2d elevatorMechanismRoot;
    private MechanismLigament2d elevatorMechanismLigament;

    // If we have multiple motors in the gearbox, then lets cache them.
    // The ordering of both will be based on the order of the motors passed into the constructor.
    private TalonFX leaderMotor;
    private TalonFXSimState leaderMotorSimState;

    private final double motorToSystemGearing;
    private final Distance drumRadius;

    private DoublePublisher elevatorHeight;

    /**
     * @param elevatorSim A reference to {@link edu.wpi.first.wpilibj.simulation.ElevatorSim} that represents the real elevator.
     * @param elevatorNetworkTable A reference to the network table used in the real subsystem's logging.
     * @param motorToSystemGearing The gear ratio from the motor to the drum that drives the elevator
     * @param drumRadius The radius of the drum that drives the elevator.
     * @param maxElevatorHeight The maximum height the elevator can reach (does not support elevators that start at a height higher than 0)
     * @param leaderMotorInGearBox An array of the same motors passed into {@code elevatorSim}'s {@code gearbox} param should be passed here.
     */
    public FusedElevatorSimMech2d(ElevatorSim elevatorSim, NetworkTable elevatorNetworkTable, double motorToSystemGearing, Distance drumRadius, Distance maxElevatorHeight, TalonFX leaderMotorInGearBox) {
        this.elevatorSim = elevatorSim;

        this.elevatorNetworkTable = elevatorNetworkTable.getSubTable("Sim");
        elevatorHeight = elevatorNetworkTable.getDoubleTopic("Elevator Height").publish();


        subsystemNetworkTableName = getSubsystemNameFromNetworkTable(elevatorNetworkTable);

        // Cache the motors in the ElevatorSim gearbox
        leaderMotor = leaderMotorInGearBox;
        leaderMotorSimState = leaderMotorInGearBox.getSimState();
        leaderMotorSimState.Orientation = ChassisReference.Clockwise_Positive;
        leaderMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        this.motorToSystemGearing = motorToSystemGearing;
        this.drumRadius = drumRadius;

        // Todo: add a method to resize elevatorMechanism and change its background color.
        // Note: Width and height are in meters.
        elevatorMechanism = new Mechanism2d(maxElevatorHeight.in(Meters)*1.5, maxElevatorHeight.in(Meters)*1.75, new Color8Bit(Color.kBlue));
        elevatorMechanismRoot = elevatorMechanism.getRoot(subsystemNetworkTableName + " Root", maxElevatorHeight.in(Meters)*0.75,maxElevatorHeight.in(Meters)*0.5);
        elevatorMechanismLigament = elevatorMechanismRoot.append(new MechanismLigament2d(subsystemNetworkTableName + " Ligament", 0, 90, 5, new Color8Bit(Color.kOrange)));
        // Todo: do the rest of the set up here.

        // Display the elevator mechanism to SmartDashboard
        SmartDashboard.putData(subsystemNetworkTableName + " Sim", elevatorMechanism);
    }

    public void runPeriodic() {
        leaderMotorSimState.setSupplyVoltage(12); // This might be unneeded.
        double leaderMotorVoltage = leaderMotor.getMotorVoltage().getValueAsDouble();
        elevatorSim.setInputVoltage(leaderMotorVoltage);

        elevatorSim.update(0.02);
        leaderMotorSimState.setRawRotorPosition(convertElevatorLinearToMotorAngular(elevatorSim.getPositionMeters()));
        leaderMotorSimState.setRotorVelocity(convertElevatorLinearToMotorAngular(elevatorSim.getVelocityMetersPerSecond()));

        elevatorHeight.accept(elevatorSim.getPositionMeters());
        elevatorMechanismLigament.setLength(elevatorSim.getPositionMeters());

    }

    public Command test(Angle e) {
        return runOnce(() -> leaderMotor.setControl(new PositionVoltage(e)));
    }

    /**
     * Converts linear measures from the elevator (i.e. position, velocity) to motor angular values
     * @param elevatorLinearValueMeters Value of the elevators positon (meters)/velocity (meters/sec)
     * @return The value converted for the motor in the gearbox.
     */
    private double convertElevatorLinearToMotorAngular(double elevatorLinearValueMeters) {
        return (elevatorLinearValueMeters/(2*Math.PI*drumRadius.in(Meters)))*motorToSystemGearing;
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

}