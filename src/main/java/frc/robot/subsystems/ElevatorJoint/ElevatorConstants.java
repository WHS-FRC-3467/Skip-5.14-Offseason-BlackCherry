package frc.robot.subsystems.ElevatorJoint;

import frc.robot.Constants;

public class ElevatorConstants {
    public static double Tolerance = 0.5;
    public static final double homingCurrent = 0.5;

    public static final Gains gains =
        switch (Constants.currentMode) {
            case SIM -> new Gains(0, 0, 0, 0, 0, 0, 0); // Need to establish
            case REAL -> new Gains(2, 0, 0, 0, 0, 0, 0.1);
            default -> new Gains(2, 0, 0, 0, 0, 0, 0.1);
        };

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
}
