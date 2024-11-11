package frc.robot.subsystems.ClimberJoint;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public interface ClimberJointIO {
    // MJW: IO Layering 11/11/2024
    @AutoLog
    class ClimberJointIOInputs {

        // USED CURRENTLY
        double supplyCurrent = 0.0;
        double position = 0.0;
        double motorVelocity = 0.0;
    }

    // Update Inputs
    default void updateInputs(ClimberJointIOInputs inputs) {}

    // Turn motors to Nuetral Mode
    default void stop() {}

    // Run Duty Cycle
    default void runDutyCycle(double output) {}

    // Move to setpoint
    default void setControl(MotionMagicVoltage goal) {}

    // Set Setpoint
    default void setPosition(double goal) {}
}
