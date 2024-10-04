package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import frc.robot.Constants.ClimberConstants;

public class ClimberJoint extends SubsystemBase{
    @RequiredArgsConstructor
    @Getter
  public enum State {
    
    //Following are rough esitamtes
    CLIMB(76.0),
    DOWN(0.0);

    private final double output;


  }

  @Getter
  @Setter
  private State state = State.DOWN;

  TalonFX m_motor = new TalonFX(ClimberConstants.ID_LEADER);
  TalonFX m_follower = new TalonFX(ClimberConstants.ID_FOLLOWER);
  private final MotionMagicVoltage m_magic = new MotionMagicVoltage(state.getOutput());
  private final NeutralOut m_neutral = new NeutralOut();

  private double goalAngle;


  /** Creates a new ComplexSubsystem. */
  public ClimberJoint() {
    m_motor.getConfigurator().apply(ClimberConstants.motorConfig());
    m_follower.getConfigurator().apply(ClimberConstants.motorConfig());
    m_follower.setControl(new Follower(ClimberConstants.ID_LEADER, false));
    m_motor.setPosition(0.0);
  }

  @Override
  public void periodic() {
    goalAngle = MathUtil.clamp(state.getOutput(), ClimberConstants.lowerLimit, ClimberConstants.upperLimit);

    if (state == State.DOWN && atGoal()) {
      m_motor.setControl(m_neutral);
    } else {
      m_motor.setControl(m_magic.withPosition(goalAngle).withSlot(1));
    }

    displayInfo(true);
  }

  public boolean atGoal() {
    return Math.abs(state.getOutput() - m_motor.getPosition().getValueAsDouble()) < ClimberConstants.tolerance;
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.DOWN);
  }

  private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " Setpoint ", state.getOutput());
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output ", m_motor.getPosition().getValueAsDouble());
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw", m_motor.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putBoolean(this.getClass().getSimpleName() + " atGoal", atGoal());
    }

  }
}