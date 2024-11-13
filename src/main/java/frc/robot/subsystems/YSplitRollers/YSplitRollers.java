// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.YSplitRollers;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeRollers.IntakeRollersIO;
import frc.robot.subsystems.IntakeRollers.IntakeRollersIOInputsAutoLogged;
import frc.robot.subsystems.YSplitRollers.YSplitRollersIO.YSplitRollersIOInputs;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

// TODO: Check rotation direction
// TODO: add shuffle behavior

public class YSplitRollers extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State {
        OFF(0.0, 0.0), // No movement
        INTAKE(0.65, 0.0), // Roller 1 only
        SLOWINTAKE(0.4, 0.0),
        SHUFFLE(-0.15, 0.3),
        SHOOTER(1.0, -1.0), // Roller 1 intakes, Roller 2 sends to shooter
        REVSHOOTER(-1.0, 1.0), // Reverse out of shooter
        AMP(0.65, 0.65), // Roller 1 intake, Roller 2 sends to elevator //Was .5 before 10/25
        REVAMP(-0.5, -0.5); // Reverse out of elevator

        private final double roller1; // From the intake
        private final double roller2; // Decides to shooter or to elevator
    }

    @Getter
    @Setter
    private State state = State.OFF;

    private boolean debug = true;

    //TalonFX m_roller1 = new TalonFX(Constants.YSplitRollersConstants.ID_YSPLIT_ROLLER1);
    //TalonFX m_roller2 = new TalonFX(Constants.YSplitRollersConstants.ID_YSPLIT_ROLLER2);
    //private final DutyCycleOut m_percent = new DutyCycleOut(0);
    //private final NeutralOut m_neutral = new NeutralOut();

    //AdvantageKit addition MJW 11/12/2024
    private final YSplitRollersIO io;
    private final YSplitRollersIOInputsAutoLogged inputs = new YSplitRollersIOInputsAutoLogged();

    /** Creates a new YSplitRollers. */
    public YSplitRollers(YSplitRollersIO io) {
        this.io = io;
        //m_roller1.getConfigurator().apply(Constants.YSplitRollersConstants.motorConfig());
        //m_roller2.getConfigurator().apply(Constants.YSplitRollersConstants.motorConfig());
    }

    @Override
    public void periodic() {

        if (state == State.OFF) {
            //m_roller1.setControl(m_neutral);
            io.stop(); // Maybe add seperation so that one will stop and not the other
            //m_roller2.setControl(m_neutral);
        } else {
            //m_roller1.setControl(m_percent.withOutput(state.getRoller1()));
            io.runDutyCycle(1, state.getRoller1());
            //m_roller2.setControl(m_percent.withOutput(state.getRoller2()));
            io.runDutyCycle(2, state.getRoller2());
        }

        displayInfo(debug);
    }

    public Command setStateCommand(State state) {
        return startEnd(() -> this.state = state, () -> this.state = State.OFF);
    }

    //@AutoLogOutput(key = "YSplitRollers/Info")
    private void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putString(this.getClass().getSimpleName() + "  State ", state.toString());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Roller1 Setpoint ", state.getRoller1());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Roller2 Setpoint ", state.getRoller2());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Roller1 Output ", inputs.motorVoltage);
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Roller2 Output ", inputs.motor2Voltage);
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Roller1 Current Draw", inputs.supplyCurrent);
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Roller2 Current Draw", inputs.motor2supplyCurrent);
        }

    }
}