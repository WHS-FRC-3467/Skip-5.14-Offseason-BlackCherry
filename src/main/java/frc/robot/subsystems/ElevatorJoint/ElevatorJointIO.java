package frc.robot.subsystems.ElevatorJoint;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

public interface ElevatorJointIO {
    //MJW: IO Layering 11/11/2024
    @AutoLog
    class ElevatorJointIOInputs {
        // USED CURRENTLY
        double supplyCurrent = 0.0;
        double position = 0.0;
        double motorVelocity = 0.0;
    }

    // Update Inputs
    default void updateInputs(ElevatorJointIOInputs inputs) {}

    // Stop Elevator and set to Neutral Mode
    default void stop() {}

    // Run Duty Cycle
    default void runDutyCycle(double output) {}

    // Set MotionMagic Control
    default void setControl(MotionMagicVoltage goal) {}

    // Set Setpoint
    default void setPosition(double goal) {}
}
