package frc.robot.subsystems.YSplitRollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.YSplitRollersConstants;

public class YSplitRollersIOKrakenFOC implements YSplitRollersIO{
    // Hardware
    private final TalonFX m_roller1;
    private final TalonFX m_roller2;

    // Status Signals
    
    private final StatusSignal<Double> m_roller1AppliedVolts;
    private final StatusSignal<Double> m_roller1SupplyCurrent;
    private final StatusSignal<Double> m_roller1TorqueCurrent;

    private final StatusSignal<Double> m_roller2AppliedVolts;
    private final StatusSignal<Double> m_roller2SupplyCurrent;
    private final StatusSignal<Double> m_roller2TorqueCurrent;

    // Control
    private final NeutralOut m_neutral = new NeutralOut();
    private final DutyCycleOut m_duty = new DutyCycleOut(0.0);

    public YSplitRollersIOKrakenFOC() {
        m_roller1 = new TalonFX(YSplitRollersConstants.ID_YSPLIT_ROLLER1);
        m_roller2 = new TalonFX(YSplitRollersConstants.ID_YSPLIT_ROLLER2);

        TalonFXConfiguration m_configuration = new TalonFXConfiguration();

        m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_configuration.Voltage.PeakForwardVoltage = 12.0;
        m_configuration.Voltage.PeakReverseVoltage = -12.0;

        m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
        m_configuration.CurrentLimits.SupplyCurrentThreshold = 60;
        m_configuration.CurrentLimits.SupplyTimeThreshold = 0.25;
        m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_configuration.CurrentLimits.StatorCurrentLimit = 140;
        m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply Configs
        m_roller1.getConfigurator().apply(m_configuration);
        m_roller2.getConfigurator().apply(m_configuration);

        // Set Signals
        m_roller1AppliedVolts = m_roller1.getMotorVoltage();
        m_roller1SupplyCurrent = m_roller1.getSupplyCurrent();
        m_roller1TorqueCurrent = m_roller1.getTorqueCurrent();

        m_roller2AppliedVolts = m_roller2.getMotorVoltage();
        m_roller2SupplyCurrent = m_roller2.getSupplyCurrent();
        m_roller2TorqueCurrent = m_roller2.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            m_roller1AppliedVolts,
            m_roller1SupplyCurrent,
            m_roller1TorqueCurrent,
            m_roller2AppliedVolts,
            m_roller2SupplyCurrent,
            m_roller2TorqueCurrent
            );
    }
    // Update Inputs
    public void updateInputs(YSplitRollersIOInputs inputs) {
        inputs.motorVoltage = m_roller1AppliedVolts.getValueAsDouble();
        inputs.supplyCurrent = m_roller1SupplyCurrent.getValueAsDouble();
        inputs.motor2Voltage = m_roller2AppliedVolts.getValueAsDouble();
        inputs.motor2supplyCurrent = m_roller2SupplyCurrent.getValueAsDouble();
    }

    // Turn motors to Nuetral Mode
    public void stop() {
        m_roller1.setControl(m_neutral);
        m_roller2.setControl(m_neutral);
    }

    // Run Duty Cycle
    public void runDutyCycle(int motor, double output) {
        if (motor == 1) {
            m_roller1.setControl(m_duty.withOutput(output));
        } else {
            m_roller2.setControl(m_duty.withOutput(output));
        }
    }
}
