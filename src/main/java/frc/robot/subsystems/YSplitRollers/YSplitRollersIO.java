package frc.robot.subsystems.YSplitRollers;

import org.littletonrobotics.junction.AutoLog;

public interface YSplitRollersIO {
    // MJW: IO Layering 11/12/2024
    @AutoLog
    class YSplitRollersIOInputs {
        // USED CURRENTLY
        double supplyCurrent = 0.0;
        double motorVoltage = 0.0;

        double motor2supplyCurrent = 0.0;
        double motor2Voltage = 0.0;
    }
    // Update Inputs
    default void updateInputs(YSplitRollersIOInputs inputs) {}

    // Turn motors to Nuetral Mode
    default void stop() {}

    // Run Duty Cycle
    default void runDutyCycle(int motor, double output) {}
}
