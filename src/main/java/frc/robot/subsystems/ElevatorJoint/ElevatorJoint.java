// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ElevatorJoint;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorJointConstants;
import frc.robot.subsystems.ClimberJoint.ClimberJointIOInputsAutoLogged;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ElevatorJoint extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State {
        STOW(0.0),
        HOMING(0.0),
        SCORE(30.0),
        TRAP(30.0);

        private final double output;
    }

    @Getter
    @Setter
    private State state = State.STOW;

    //TalonFX m_motor = new TalonFX(ElevatorJointConstants.ID_LEADER);
    //TalonFX m_follower = new TalonFX(ElevatorJointConstants.ID_LEADER);

    private final MotionMagicVoltage m_magic = new MotionMagicVoltage(state.getOutput());
    //private final DutyCycleOut m_duty = new DutyCycleOut(0.0);
    //private final NeutralOut m_neutral = new NeutralOut();

    //AdvantageKit addition MJW 11/11/2024
    private final ElevatorJointIO io;
    private final ElevatorJointIOInputsAutoLogged inputs = new ElevatorJointIOInputsAutoLogged();

    public boolean hasHomed = false;

    /** Creates a new ComplexSubsystem. */
    public ElevatorJoint(ElevatorJointIO io) {
        this.io = io;
        //m_motor.getConfigurator().apply(ElevatorJointConstants.motorConfig());
        //m_follower.getConfigurator().apply(ElevatorJointConstants.motorConfig());
        //m_follower.setControl(new Follower(ElevatorJointConstants.ID_LEADER, false));
        //m_motor.setPosition(0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ElevatorJoint", inputs);
        if (state == State.STOW && atGoal()) {
            //m_motor.setControl(m_neutral);
            io.stop();

        } else if (state == State.HOMING) {
            //m_motor.setControl(m_duty.withOutput(-0.05));
            io.runDutyCycle(-0.05);

            if (inputs.supplyCurrent > ElevatorConstants.homingCurrent) {
                //m_motor.setPosition(0.0);
                io.setPosition(0.0);
                System.out.println("HOMED Elevator");
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
                ElevatorConstants.Tolerance);
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
