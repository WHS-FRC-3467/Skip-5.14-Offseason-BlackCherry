package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import frc.robot.Constants.ClimberJointConstants;

public class ClimberJoint extends SubsystemBase {
    @RequiredArgsConstructor
    @Getter
    public enum State {

        STOW(0.0),
        HOMING(0.0),
        CLIMB(76.0);

        private final double output;
    }

    @Getter
    @Setter
    private State state = State.STOW;

    TalonFX m_motor = new TalonFX(ClimberJointConstants.ID_LEADER);
    TalonFX m_follower = new TalonFX(ClimberJointConstants.ID_FOLLOWER);

    private final MotionMagicVoltage m_magic = new MotionMagicVoltage(state.getOutput());
    private final DutyCycleOut m_duty = new DutyCycleOut(0.0);
    private final NeutralOut m_neutral = new NeutralOut();

    private boolean hasHomed = false;

    public ClimberJoint() {
        m_motor.getConfigurator().apply(ClimberJointConstants.motorConfig());
        m_follower.getConfigurator().apply(ClimberJointConstants.motorConfig());
        m_follower.setControl(new Follower(ClimberJointConstants.ID_LEADER, false));
        m_motor.setPosition(0.0);
    }

    @Override
    public void periodic() {


        if (state == State.STOW && atGoal()) {
            m_motor.setControl(m_neutral);

        } else if (state == State.HOMING) {
            m_motor.setControl(m_duty.withOutput(-0.05));

            if (m_motor.getSupplyCurrent().getValueAsDouble() > 0.5) {
                m_motor.setPosition(0.0);
                System.out.println("HOMED Climber");
                this.state = State.STOW;

            }

        } else {
            m_motor.setControl(m_magic.withPosition(state.getOutput()).withSlot(1));
        }

        displayInfo(true);
    }

    public boolean atGoal() {
        return MathUtil.isNear(state.getOutput(), m_motor.getPosition().getValueAsDouble(),
                ClimberJointConstants.tolerance);
    }

    public Command setStateCommand(State state) {
        return startEnd(() -> this.state = state, () -> this.state = State.STOW);
    }

    private void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Setpoint ", state.getOutput());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output ", m_motor.getPosition().getValueAsDouble());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw", m_motor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putBoolean(this.getClass().getSimpleName() + " atGoal", atGoal());
            SmartDashboard.putBoolean(this.getClass().getSimpleName() + " has homed", hasHomed);
        }

    }
}