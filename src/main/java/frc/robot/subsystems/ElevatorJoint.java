// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorJointConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ElevatorJoint extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State {
    STOW(0.0),
    HOMING(0.0),
    SCORE(90.0);

    private final double output;

    private double getStateOutput() {
      return Units.degreesToRadians(output);
    }
  }

  @Getter
  @Setter
  private State state = State.STOW;

  TalonFX m_motor = new TalonFX(ElevatorJointConstants.ID_LEADER);
  TalonFX m_follower = new TalonFX(ElevatorJointConstants.ID_LEADER);

  private final MotionMagicVoltage m_magic = new MotionMagicVoltage(state.getStateOutput());
  private final DutyCycleOut m_duty = new DutyCycleOut(0.0);
  private final NeutralOut m_neutral = new NeutralOut();

  private double goalAngle;


  /** Creates a new ComplexSubsystem. */
  public ElevatorJoint() {
    m_motor.getConfigurator().apply(ElevatorJointConstants.motorConfig());
    m_follower.getConfigurator().apply(ElevatorJointConstants.motorConfig());
    m_follower.setControl(new Follower(ElevatorJointConstants.ID_LEADER, false));
  }

  @Override
  public void periodic() {
    goalAngle = MathUtil.clamp(state.getStateOutput(), ElevatorJointConstants.lowerLimit, ElevatorJointConstants.upperLimit);

    if (state == State.STOW && atGoal()) {
    
      m_motor.setControl(m_neutral);

    } else if (state == State.HOMING) {

      m_motor.setControl(m_duty.withOutput(-0.2));

      if (m_motor.getSupplyCurrent().getValueAsDouble() > 10.0) {
        m_motor.setPosition(0.0);
        this.state = State.STOW;
      }

    } else {
      m_motor.setControl(m_magic.withPosition(goalAngle).withSlot(1));
    }

    displayInfo(true);
  }

  public boolean atGoal() {
    return Math.abs(state.getStateOutput() - m_motor.getPosition().getValueAsDouble()) < ElevatorJointConstants.tolerance;
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
