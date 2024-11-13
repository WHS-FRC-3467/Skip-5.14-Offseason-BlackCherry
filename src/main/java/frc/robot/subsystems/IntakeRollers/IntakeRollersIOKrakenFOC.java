package frc.robot.subsystems.IntakeRollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IntakeRollersConstants;

public class IntakeRollersIOKrakenFOC implements IntakeRollersIO{
    // Hardware
    private final TalonFX m_motor;

    // Status Signals
    
    private final StatusSignal<Double> m_motorAppliedVolts;
    private final StatusSignal<Double> m_motorSupplyCurrent;
    private final StatusSignal<Double> m_motorTorqueCurrent;

    // Control
    private final NeutralOut m_neutral = new NeutralOut();
    private final DutyCycleOut m_duty = new DutyCycleOut(0.0);

    public IntakeRollersIOKrakenFOC() {
        m_motor = new TalonFX(IntakeRollersConstants.ID_Motor);

        TalonFXConfiguration m_configuration = new TalonFXConfiguration();

        m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_configuration.Voltage.PeakForwardVoltage = 12.0;
        m_configuration.Voltage.PeakReverseVoltage = -12.0;

        m_configuration.CurrentLimits.SupplyCurrentLimit = 40;
        m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
        m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
        m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_configuration.CurrentLimits.StatorCurrentLimit = 80;
        m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply Configs
        m_motor.getConfigurator().apply(m_configuration);

        // Set Signals
        m_motorAppliedVolts = m_motor.getMotorVoltage();
        m_motorSupplyCurrent = m_motor.getSupplyCurrent();
        m_motorTorqueCurrent = m_motor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            m_motorAppliedVolts,
            m_motorSupplyCurrent,
            m_motorTorqueCurrent
            );
    }
    // Update Inputs
    public void updateInputs(IntakeRollersIOInputs inputs) {
        inputs.motorVoltage = m_motorAppliedVolts.getValueAsDouble();
        inputs.supplyCurrent = m_motorSupplyCurrent.getValueAsDouble();
    }

    // Turn motors to Nuetral Mode
    public void stop() {
        m_motor.setControl(m_neutral);
    }

    // Run Duty Cycle
    public void runDutyCycle(double output) {
        m_motor.setControl(m_duty.withOutput(output));
    }
}
