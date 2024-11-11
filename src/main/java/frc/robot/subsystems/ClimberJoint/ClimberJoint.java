package frc.robot.subsystems.ClimberJoint;

import com.ctre.phoenix6.hardware.TalonFX;

import org.littletonrobotics.junction.Logger;

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
//import frc.robot.Constants.ClimberJointConstants;
import frc.robot.subsystems.ClimberJoint.ClimberJointConstants;
import frc.robot.subsystems.ClimberJoint.ClimberJointIO;
import frc.robot.subsystems.ClimberJoint.ClimberJointIOInputsAutoLogged;

public class ClimberJoint extends SubsystemBase {
    @RequiredArgsConstructor
    @Getter
    public enum State {

        STOW(0.0),
        HOMING(0.0),
        PREP(50.0),
        CLIMB(68.0);

        private final double output;
    }

    @Getter
    @Setter
    public State state = State.STOW;

    //TalonFX m_motor = new TalonFX(ClimberJointConstants.ID_LEADER);
    //TalonFX m_follower = new TalonFX(ClimberJointConstants.ID_FOLLOWER);

    private final MotionMagicVoltage m_magic = new MotionMagicVoltage(state.getOutput());
    //private final DutyCycleOut m_duty = new DutyCycleOut(0.0);
    //private final NeutralOut m_neutral = new NeutralOut();

    //AdvantageKit addition MJW 11/11/2024
    private final ClimberJointIO io;
    private final ClimberJointIOInputsAutoLogged inputs = new ClimberJointIOInputsAutoLogged();

    public boolean hasHomed = false;

    public ClimberJoint(ClimberJointIO io) {
        this.io = io;
        // m_motor.getConfigurator().apply(ClimberJointConstants.motorConfig());
        // m_follower.getConfigurator().apply(ClimberJointConstants.motorConfig());
        // m_follower.setControl(new Follower(ClimberJointConstants.ID_LEADER, false));
        // m_motor.setPosition(0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ClimberJoint", inputs);
        if (state == State.STOW && atGoal()) {
            io.stop();
            //m_motor.setControl(m_neutral);

        } else 
        if (state == State.HOMING) {
            //m_motor.setControl(m_duty.withOutput(-0.1));
            io.runDutyCycle(-0.1);

            if (inputs.supplyCurrent > 2) {
                //m_motor.setPosition(0.0);
                io.setPosition(0.0);
                System.out.println("HOMED Climber");
                this.hasHomed = true;
                this.state = State.STOW;

            }

        } else {
            //m_motor.setControl(m_magic.withPosition(state.getOutput()).withSlot(1));
            io.setControl(m_magic.withPosition(state.getOutput()).withSlot(1));
        }

        displayInfo(true);
    }

    public boolean atGoal() {
        return MathUtil.isNear(state.getOutput(), inputs.position,
                ClimberJointConstants.Tolerance);
    }

 

    public Command setStateCommand(State state) {
        return startEnd(() -> this.state = state, () -> this.state = State.STOW);
    }

    private void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Setpoint ", state.getOutput());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output ", inputs.position);
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw", inputs.supplyCurrent);
            SmartDashboard.putBoolean(this.getClass().getSimpleName() + " atGoal", atGoal());
            SmartDashboard.putBoolean(this.getClass().getSimpleName() + " has homed", hasHomed);
        }

    }
}