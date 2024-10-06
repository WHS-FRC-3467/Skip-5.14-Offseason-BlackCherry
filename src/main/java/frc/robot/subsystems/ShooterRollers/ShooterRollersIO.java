package frc.robot.subsystems.ShooterRollers;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterRollersIO {
    // MJW: IO Layering 10/4/2024
    @AutoLog
    class ShooterRollersIOInputs {
        public double topPositionRads = 0.0;
        public double topVelocityRpm = 0.0;
        public double topAppliedVolts = 0.0;
        public double topSupplyCurrentAmps = 0.0;
        public double topTorqueCurrentAmps = 0.0;

        public double bottomPositionRads = 0.0;
        public double bottomVelocityRpm = 0.0;
        public double bottomAppliedVolts = 0.0;
        public double bottomSupplyCurrentAmps = 0.0;
        public double bottomTorqueCurrentAmps = 0.0;
    }

    // Update Inputs
    default void updateInputs(ShooterRollersIOInputs inputs) {}

    /* Stop ShooterRollers */
    default void stop() {}

    /* runm ShooterRollers at veloicity in rpm */
    default void runVelocity(double Rpm, double Feedforward) {}

    default void runVolts(double topVolts, double bottomVolts) {}
    // MJW: IO Layering 10/4/2024
}
