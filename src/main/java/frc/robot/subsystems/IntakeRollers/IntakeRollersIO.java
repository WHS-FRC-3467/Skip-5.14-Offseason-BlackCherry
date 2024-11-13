package frc.robot.subsystems.IntakeRollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
    // MJW: IO Layering 11/12/2024
    @AutoLog
    class IntakeRollersIOInputs {
        // USED CURRENTLY
        double supplyCurrent = 0.0;
        double motorVoltage = 0.0;
    }
    // Update Inputs
    default void updateInputs(IntakeRollersIOInputs inputs) {}

    // Turn motors to Nuetral Mode
    default void stop() {}

    // Run Duty Cycle
    default void runDutyCycle(double output) {}
}
