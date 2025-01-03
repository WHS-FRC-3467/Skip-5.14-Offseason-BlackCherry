// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterJointConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ShooterJoint extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State {
        STOW(() -> 35.0),
        SUBWOOFER(() -> 42.0),
        CLIMBCLEARANCE(() -> 40.0),
        DYNAMIC(() -> RobotState.getInstance().getShotAngle()), //Dynamic aiming, data fed by RobotState for lookup tables
        TUNING(() -> RobotState.getInstance().getShooterTuningAngle().get()); //Dashboard tunable number

        private final DoubleSupplier outputSupplier;

        private double getStateOutput() {
            return Units.degreesToRotations(outputSupplier.getAsDouble());
        }
    }

    @Getter
    @Setter
    private State state = State.STOW;

    private Debouncer m_debounce = new Debouncer(.1);

    TalonFX m_motor = new TalonFX(ShooterJointConstants.ID_MOTOR);
    CANcoder m_encoder = new CANcoder(ShooterJointConstants.ID_ENCODER);

    private final static MotionMagicVoltage m_magic = new MotionMagicVoltage(0);
    private final static PositionVoltage m_position = new PositionVoltage(0);
    // private final NeutralOut m_neutral = new NeutralOut();

    public ShooterJoint() {
        m_encoder.getConfigurator().apply(ShooterJointConstants.encoderConfig());
        m_motor.getConfigurator().apply(ShooterJointConstants.motorConfig());
    }

    @Override
    public void periodic() {
        if (state == State.DYNAMIC) {
            m_motor.setControl(m_position.withPosition(state.getStateOutput()).withSlot(0));
            //m_motor.setControl(m_magic.withPosition(state.getStateOutput()).withSlot(1));
        } else {
            m_motor.setControl(m_magic.withPosition(state.getStateOutput()).withSlot(1));
        }

        displayInfo(true);
    }

    public boolean atGoal() {
        return m_debounce.calculate(MathUtil.isNear(state.getStateOutput(), m_motor.getPosition().getValueAsDouble(), ShooterJointConstants.tolerance));
    }

    public Command setStateCommand(State state) {
        return startEnd(() -> this.state = state, () -> this.state = State.STOW);
    }

    private void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Setpoint ", state.getStateOutput());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Setpoint (deg) ", Units.rotationsToDegrees(state.getStateOutput()));
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output ", m_motor.getPosition().getValueAsDouble());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output deg ", Units.rotationsToDegrees(m_motor.getPosition().getValueAsDouble()));
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw", m_motor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putBoolean(this.getClass().getSimpleName() + " atGoal", atGoal());
        }

    }
}
