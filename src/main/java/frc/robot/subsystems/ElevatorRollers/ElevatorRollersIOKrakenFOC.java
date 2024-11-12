package frc.robot.subsystems.ElevatorRollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.util.Units;

import frc.robot.Constants.ElevatorRollersConstants;
import frc.robot.subsystems.ElevatorRollers.ElevatorRollersIO.ElevatorRollersIOInputs;

public class ElevatorRollersIOKrakenFOC implements ElevatorRollersIO{
    // Hardware
    private final TalonFX m_motor;

    // Status Signals
    
    private final StatusSignal<Double> m_motorAppliedVolts;
    private final StatusSignal<Double> m_motorSupplyCurrent;
    private final StatusSignal<Double> m_motorTorqueCurrent;

    // Control
    private final NeutralOut m_neutral = new NeutralOut();
    private final DutyCycleOut m_duty = new DutyCycleOut(0.0);

    public ElevatorRollersIOKrakenFOC() {
        m_motor = new TalonFX(ElevatorRollersConstants.ID_Motor);

        TalonFXConfiguration m_configuration = new TalonFXConfiguration();

        m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_configuration.Voltage.PeakForwardVoltage = 12.0;
        m_configuration.Voltage.PeakReverseVoltage = -12.0;

        m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
        m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
        m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
        m_configuration.CurrentLimits.SupplyCurrentLimitEnable = false;
        m_configuration.CurrentLimits.StatorCurrentLimit = 70;
        m_configuration.CurrentLimits.StatorCurrentLimitEnable = false;

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
    public void updateInputs(ElevatorRollersIOInputs inputs) {
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
