// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorJointConstants;
import frc.robot.Constants.IntakeJointConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class IntakeJoint extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State {
        STOW(0.0),
        HOMING(0.0),
        INTAKE(-0.27);

        private final double output;
    }

    @Getter
    @Setter
    private State state = State.STOW;
    private final SendableChooser<State> stateChooser = new SendableChooser<>();
    

    TalonFX m_motor = new TalonFX(IntakeJointConstants.ID_Motor);
    private final MotionMagicVoltage m_position = new MotionMagicVoltage(state.getOutput());
    private final DutyCycleOut m_duty = new DutyCycleOut(0.0);
    private final NeutralOut m_neutral = new NeutralOut();

    private boolean hasHomed = false;

    /** Creates a new ComplexSubsystem. */
    public IntakeJoint() {
        m_motor.getConfigurator().apply(IntakeJointConstants.motorConfig());
        m_motor.setPosition(0.0);
        for (State states : State.values()) {
                stateChooser.addOption(states.toString(), states);  
        }
        stateChooser.setDefaultOption(state.toString(), state);
        SmartDashboard.putData("IntakeJoint State Chooser", stateChooser);
        SmartDashboard.putData("IntakeJoint Override Command",Commands.runOnce(() -> setState(stateChooser.getSelected()), this));

    }

    @Override
    public void periodic() {
        if (state == State.STOW && atGoal()) {
            m_motor.setControl(m_neutral);
        } else if (state == State.HOMING) {
            m_motor.setControl(m_duty.withOutput(0.05));

            if (m_motor.getSupplyCurrent().getValueAsDouble() > IntakeJointConstants.homingCurrent) {
                m_motor.setPosition(0.0);
                System.out.println("HOMED Elevator");
                this.hasHomed = true;
                this.state = State.STOW;
            }

        } else {
            m_motor.setControl(m_position.withPosition(state.getOutput()).withSlot(1));
        }

        displayInfo(true);
    }

    public boolean atGoal() {
        return MathUtil.isNear(state.getOutput(), m_motor.getPosition().getValueAsDouble(),
                IntakeJointConstants.tolerance);
    }

    public Command setStateCommand(State state) {
        return startEnd(() -> this.state = state, () -> this.state = State.STOW);
    }

    private void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Setpoint ", state.getOutput());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output ", Units.rotationsToDegrees(m_motor.getPosition().getValueAsDouble()));
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw", m_motor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putBoolean(this.getClass().getSimpleName() + " atGoal", atGoal());
            SmartDashboard.putBoolean(this.getClass().getSimpleName() + " has homed", hasHomed);
        }

    }
}
