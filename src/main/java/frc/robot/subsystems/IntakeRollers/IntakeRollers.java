// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeRollers;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRollersConstants;
//import frc.robot.subsystems.ElevatorRollers.ElevatorRollersIO;
//import frc.robot.subsystems.ElevatorRollers.ElevatorRollersIOInputsAutoLogged;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

/* Local Imports */
import frc.robot.subsystems.IntakeRollers.IntakeRollersIO;
import frc.robot.subsystems.IntakeRollers.IntakeRollersIOInputsAutoLogged;
//import frc.robot.subsystems.IntakeRollers.IntakeRollersConstants;

public class IntakeRollers extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State {
        OFF(0.0),
        INTAKE(1.0),
        EJECT(-0.3);

        private final double output;
    }

    @Getter
    @Setter
    private State state = State.OFF;

    private boolean debug = true;

    //TalonFX m_motor = new TalonFX(IntakeRollersConstants.ID_Motor);
    //private final DutyCycleOut m_percent = new DutyCycleOut(0);
    //private final NeutralOut m_neutral = new NeutralOut();

    //AdvantageKit addition MJW 11/12/2024
    private final IntakeRollersIO io;
    private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

    /** Creates a new SimpleSubsystem. */
    public IntakeRollers(IntakeRollersIO io) {
        this.io = io;
        //m_motor.getConfigurator().apply(IntakeRollersConstants.motorConfig());

    }

    @Override
    public void periodic() {

        if (state == State.OFF) {
            //m_motor.setControl(m_neutral);
            io.stop();
        } else {
            //m_motor.setControl(m_percent.withOutput(state.getOutput()));
            io.runDutyCycle(state.getOutput());
        }

        displayInfo(debug);
    }

    public Command setStateCommand(State state) {
        return startEnd(() -> this.state = state, () -> this.state = State.OFF);
    }

    //@AutoLogOutput(key = "IntakeRollers/Info")
    private void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Setpoint ", state.getOutput());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output ", inputs.motorVoltage);
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw", inputs.supplyCurrent);
        }

    }
}