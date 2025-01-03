// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.ShooterRollersConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ShooterRollers extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State { // RPS
        OFF(() -> 0.0),
        SUBWOOFER(() -> 35.0),
        SPEAKER(() -> 45.0),
        FEED(() -> 22.0),
        REVERSE(() -> -20.0),
        TUNING(() -> RobotState.getInstance().getShooterTuningSpeed().get());

        private final DoubleSupplier velocitySupplier;

        private double getStateOutput() {
            return velocitySupplier.getAsDouble();
        }
    }

    @Getter
    @Setter
    private State state = State.OFF;

    // Initialize motor controllers
    TalonFX m_motor = new TalonFX(ShooterRollersConstants.ID_LEADER);
    TalonFX m_follower = new TalonFX(ShooterRollersConstants.ID_FOLLOWER); 
    
    private final VelocityVoltage m_velocity = new VelocityVoltage(0).withSlot(1);
    private final NeutralOut m_neutral = new NeutralOut();

    private double goalSpeed;

    /** Creates a new Flywheel. */
    public ShooterRollers() {
        m_motor.getConfigurator().apply(ShooterRollersConstants.motorConfig());
        m_follower.getConfigurator().apply(ShooterRollersConstants.motorConfig());
        m_follower.setControl(new Follower(m_motor.getDeviceID(), true));
    }

    @Override
    public void periodic() {
      
        if (state == State.OFF) {
            m_motor.setControl(m_neutral);
        } else {
            goalSpeed = MathUtil.clamp(state.getStateOutput(), ShooterRollersConstants.lowerLimit, ShooterRollersConstants.upperLimit); //TODO:Remove 
            m_motor.setControl(m_velocity.withVelocity(goalSpeed).withSlot(1)); // create a velocity closed-loop request, voltage output, slot 1 configs
        }

        displayInfo(true);
    }

    public boolean atGoal() {
        return Math.abs(state.getStateOutput() - m_motor.getVelocity().getValueAsDouble()) < ShooterRollersConstants.tolerance;
    }

    public Command setStateCommand(State state) {
        return startEnd(() -> this.state = state, () -> this.state = State.OFF);
    }

    public void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Setpoint ", state.getStateOutput());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output ", m_motor.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw", m_motor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putBoolean(this.getClass().getSimpleName() + " atGoal", atGoal());
        }
    }
}
