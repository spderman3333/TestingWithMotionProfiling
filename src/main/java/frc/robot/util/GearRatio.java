package frc.robot.util;

/**
 * An immutable wrapper class around a gear ratio, in order to ensure gear ratios are created correctly, and ready for simulation.
 */
public class GearRatio {

    private final int numberOfDrivingTeeth;
    private final int numberOfDrivenTeeth;
    private final double gearRatio;

    private final GearRatioType gearRatioType;

    /**
     * Constructs a gear ratio from the given input.
     *
     * @param numberOfDrivingTeeth Number of teeth on the driving (input) gear.
     * @param numberOfDrivenTeeth Number of teeth on the driven (output) gear.
     */
    public GearRatio(int numberOfDrivingTeeth, int numberOfDrivenTeeth) {
        this.numberOfDrivingTeeth = numberOfDrivingTeeth;
        this.numberOfDrivenTeeth = numberOfDrivenTeeth;

        gearRatio = (double) numberOfDrivenTeeth / numberOfDrivingTeeth;

        if (gearRatio > 1.0) {
            gearRatioType = GearRatioType.UNDERDRIVE;
        } else if (gearRatio == 1.0) {
            gearRatioType = GearRatioType.DIRECT_DRIVE;
        } else if (gearRatio < 1.0) {
            gearRatioType = GearRatioType.OVERDRIVE;
        } else {
            throw new RuntimeException("\"Math ain't mathing,\" gearRatio: \"" + gearRatio + "\" is invalid, and some how not a number!");
        }
    }

    /**
     * The gear ratio from the ensuing two gears, in the ratio of "driven:driving."
     * This is also known as the "reduction" between the two gears.
     * This field will work for WPILib's <a href=https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/LinearSystemSim.html>LinearSystemSim</a> classes.
     */
    public double getGearRatio() {
        return gearRatio;
    }

    /**
     * Number of teeth on the driven (output) gear.
     */
    public int getNumberOfDrivenTeeth() {
        return numberOfDrivenTeeth;
    }

    /**
     * Number of teeth on the driving (input) gear.
     */
    public int getNumberOfDrivingTeeth() {
        return numberOfDrivingTeeth;
    }

    /**
     * The type of gear ratio this object is.
     * @see GearRatioType
     */
    public GearRatioType getGearRatioType() {
        return gearRatioType;
    }

    /**
     * Represents the different types of gear ratios (reduction/underdrive, direct drive, and overdrive).
     */
    public enum GearRatioType {
        /**
         * Gear reduction, input smaller than the output, so the ratio is &gt; 1:1 (e.g. 2:1, 3:1)
         */
        UNDERDRIVE,
        /**
         * The ratio is 1:1, so there is no difference in torque or speed, except the driven gear spins opposite the driving gear.
         */
        DIRECT_DRIVE,
        /**
         * Input is larger than the output, so the ratio is &lt; 1:1 (e.g 0.8:1, 0.75:1)
         */
        OVERDRIVE
    }
}
