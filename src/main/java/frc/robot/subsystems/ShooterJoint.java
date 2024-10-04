// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.Util.TunableNumber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterPivotConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ShooterJoint extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State {
    STOW(() -> 20.0),
    SUBWOOFER(() -> 44.0),
    CLIMBCLEARANCE(() -> 40.0),
    DYNAMIC(() -> RobotState.getInstance().getShotAngle()),
    TUNING(() -> RobotState.getInstance().getShooterTuningAngle().get());

    private final DoubleSupplier outputSupplier;

    private double getStateOutput() {
      return Units.degreesToRotations(outputSupplier.getAsDouble());
    }
  }

  @Getter
  @Setter
  private State state = State.STOW;

  private double goalAngle;

  private Debouncer m_debounce = new Debouncer(.1);


  TalonFX m_motor = new TalonFX(ShooterPivotConstants.ID_MOTOR);
  private final static MotionMagicVoltage m_magic = new MotionMagicVoltage(0);
  private final static PositionVoltage m_position = new PositionVoltage(0);
  private final NeutralOut m_neutral = new NeutralOut();

  /** Creates a new ComplexSubsystem. */
  public ShooterJoint() {
    m_motor.getConfigurator().apply(ShooterPivotConstants.motorConfig());
  }

  @Override
  public void periodic() {
    goalAngle = MathUtil.clamp(state.getStateOutput(), ShooterPivotConstants.lowerLimit, ShooterPivotConstants.upperLimit);

    if (state == State.DYNAMIC) {
      m_motor.setControl(m_position.withPosition(goalAngle).withSlot(0));
    } else {
      m_motor.setControl(m_magic.withPosition(goalAngle).withSlot(1));
    }

    displayInfo(true);
  }

  public boolean atGoal() {
    return m_debounce.calculate(Math.abs(state.getStateOutput() - m_motor.getPosition().getValueAsDouble()) < ShooterPivotConstants.tolerance);
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.STOW);
  }

  private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " Setpoint ", state.getStateOutput());
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output ", m_motor.getPosition().getValueAsDouble());
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output deg ", Units.radiansToDegrees(m_motor.getPosition().getValueAsDouble()));
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw", m_motor.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putBoolean(this.getClass().getSimpleName() + " atGoal", atGoal());
    }

  }
}
