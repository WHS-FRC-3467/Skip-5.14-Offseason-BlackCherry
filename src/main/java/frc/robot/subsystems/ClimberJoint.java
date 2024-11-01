package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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

        STOW(0.0), //Lowered down onto the drivebase
        HOMING(0.0), //Homing state
        PREP(50.0), //Partially raised, low enough to pass under the chain
        CLIMB(68.0); //Fully up above robot, ready to climb

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

    public boolean hasHomed = false;
    private Debouncer homingDebouncer = new Debouncer(0.5,DebounceType.kRising); 

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

        } else 
        if (state == State.HOMING) {
            m_motor.setControl(m_duty.withOutput(-0.5));

            //TODO: Test velocity based homing
            if (m_motor.getSupplyCurrent().getValueAsDouble() > 2) {
            //if (homingDebouncer.calculate(m_motor.getVelocity().getValueAsDouble() < 0.1)) {
                m_motor.setPosition(0.0);
                System.out.println("HOMED Climber");
                this.hasHomed = true;
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