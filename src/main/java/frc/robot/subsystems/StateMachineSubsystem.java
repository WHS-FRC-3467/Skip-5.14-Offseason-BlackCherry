// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRollersConstants;
import lombok.RequiredArgsConstructor;

/** Add your docs here. */
public class StateMachineSubsystem extends SubsystemBase {
    
    private final NeutralOut m_neutral = new NeutralOut();
    private final DutyCycleOut m_duty = new DutyCycleOut(0);

    public enum RequestType {
        NONE,
        NEUTRAL,
        DUTY,
        MMPOSITION,
        MMVELOCITY;
    }

    
    @RequiredArgsConstructor
    public enum State {
        OFF(RequestType.NONE,0.0),
        ON(RequestType.DUTY,1.0);

        private final RequestType requestType;
        private final double setpoint;
    }

    private final State defaulState = State.values()[0];
    private State currentState = defaulState;
    private TalonFX m_motor;

    public StateMachineSubsystem(int motorID) {
        this.m_motor = new TalonFX(motorID);
        

    }

    @Override
    public void periodic() {
        switch (currentState.requestType) {
            case NONE:
                break;
            case NEUTRAL:
                m_motor.setControl(m_neutral);
                break;
            case DUTY:
                m_motor.setControl(m_duty.withOutput(currentState.setpoint));
            default: 
                break;

        }

    }

    public Command setStateCommand(State state) {
        return startEnd(() -> this.currentState = state, () -> this.currentState = defaulState);
    }
}
