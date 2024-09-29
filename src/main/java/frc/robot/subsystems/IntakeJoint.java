// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExampleComplexSubsystemConstants;
import frc.robot.Constants.IntakeJointConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class IntakeJoint extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State {
    STOW(90.0),
    INTAKE(0.0);

    private final double output;

    private double getStateOutput() {
      return Units.degreesToRadians(output);
    }
  }

  @Getter
  @Setter
  private State state = State.STOW;

  TalonFX m_motor = new TalonFX(IntakeJointConstants.ID_Motor);
  private final PositionVoltage m_position = new PositionVoltage(state.getStateOutput());
  //private final MotionMagicVoltage m_magic = new MotionMagicVoltage(state.getStateOutput());
  private final NeutralOut m_neutral = new NeutralOut();

  private double goalAngle;


  /** Creates a new ComplexSubsystem. */
  public IntakeJoint() {
    m_motor.getConfigurator().apply(IntakeJointConstants.motorConfig());
  }

  @Override
  public void periodic() {
    goalAngle = MathUtil.clamp(state.getStateOutput(), IntakeJointConstants.lowerLimit, IntakeJointConstants.upperLimit);

    if (state == State.STOW && atGoal()) {
      m_motor.setControl(m_neutral);
    } else {
      m_motor.setControl(m_position.withPosition(goalAngle).withSlot(0));
    }

    displayInfo(true);
  }

  public boolean atGoal() {
    return Math.abs(state.getStateOutput() - m_motor.getPosition().getValueAsDouble()) < ExampleComplexSubsystemConstants.tolerance;
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.STOW);
  }

  private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " Setpoint ", state.getStateOutput());
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output ", m_motor.getPosition().getValueAsDouble());
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw", m_motor.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putBoolean(this.getClass().getSimpleName() + " atGoal", atGoal());
    }

  }
}
