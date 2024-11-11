package frc.robot.subsystems.ClimberJoint;

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

import frc.robot.Constants.ClimberJointConstants;

public class ClimberJointIOKrakenFOC implements ClimberJointIO {
    // Hardware
    private final TalonFX m_motor;
    private final TalonFX m_follower;

    // Status Signals

    private final StatusSignal<Double> m_motorPosition;
    private final StatusSignal<Double> m_motorVelocity;
    private final StatusSignal<Double> m_motorAppliedVolts;
    private final StatusSignal<Double> m_motorSupplyCurrent;
    private final StatusSignal<Double> m_motorTorqueCurrent;

    private final StatusSignal<Double> m_followerPosition;
    private final StatusSignal<Double> m_followerVelocity;
    private final StatusSignal<Double> m_followerAppliedVolts;
    private final StatusSignal<Double> m_followerSupplyCurrent;
    private final StatusSignal<Double> m_followerTorqueCurrent;

    // Control
    private final VelocityVoltage m_velocity = new VelocityVoltage(0).withSlot(1); //Line 50
    private final NeutralOut m_neutral = new NeutralOut().withUpdateFreqHz(0.0); //Line 51
    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
    private final DutyCycleOut m_duty = new DutyCycleOut(0.0);

    public ClimberJointIOKrakenFOC() {
        m_motor = new TalonFX(ClimberJointConstants.ID_LEADER);
        m_follower = new TalonFX(ClimberJointConstants.ID_FOLLOWER);

        TalonFXConfiguration m_configuration = new TalonFXConfiguration();

        m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 77.6;
        m_configuration.Voltage.PeakForwardVoltage = 12.0;
        m_configuration.Voltage.PeakReverseVoltage = -12.0;

        m_configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        m_configuration.Feedback.SensorToMechanismRatio = 1;

        m_configuration.Slot0.kP = 0; // output per unit of error in position (output/rotation)
        m_configuration.Slot0.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
        m_configuration.Slot0.kD = 0; // output per unit of error derivative in position (output/rps)

        m_configuration.Slot1.kG = 0; // output to overcome gravity (output)
        m_configuration.Slot1.kS = 0; // output to overcome static friction (output)
        m_configuration.Slot1.kV = 0; // output per unit of requested velocity (output/rps)
        m_configuration.Slot1.kA = 0; // unused, as there is no target acceleration
        m_configuration.Slot1.kP = 5; // output per unit of error in position (output/rotation)
        m_configuration.Slot1.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
        m_configuration.Slot1.kD = 0; // output per unit of error derivative in position (output/rps)

        m_configuration.MotionMagic.MotionMagicCruiseVelocity = 600;
        m_configuration.MotionMagic.MotionMagicAcceleration = 200;
        m_configuration.MotionMagic.MotionMagicJerk = 0;

        m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
        m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
        m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
        m_configuration.CurrentLimits.SupplyCurrentLimitEnable = false;
        m_configuration.CurrentLimits.StatorCurrentLimit = 70;
        m_configuration.CurrentLimits.StatorCurrentLimitEnable = false;

        // Apply Configs
        m_motor.getConfigurator().apply(m_configuration);
        m_follower.getConfigurator().apply(m_configuration);
        m_follower.setControl(new Follower(m_motor.getDeviceID(), true));

        // Set Signals
        m_motorPosition = m_motor.getPosition();
        m_motorVelocity = m_motor.getVelocity();
        m_motorAppliedVolts = m_motor.getMotorVoltage();
        m_motorSupplyCurrent = m_motor.getSupplyCurrent();
        m_motorTorqueCurrent = m_motor.getTorqueCurrent();

        m_followerPosition = m_follower.getPosition();
        m_followerVelocity = m_follower.getVelocity();
        m_followerAppliedVolts = m_follower.getMotorVoltage();
        m_followerSupplyCurrent = m_follower.getSupplyCurrent();
        m_followerTorqueCurrent = m_follower.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            m_motorPosition,
            m_motorVelocity,
            m_motorAppliedVolts,
            m_motorSupplyCurrent,
            m_motorTorqueCurrent,
            m_followerPosition,
            m_followerVelocity,
            m_followerAppliedVolts,
            m_followerSupplyCurrent,
            m_followerTorqueCurrent
            );
    }

    // Update Inputs
    public void updateInputs(ClimberJointIOInputs inputs) {
        inputs.motorVelocity = m_motorVelocity.getValueAsDouble();
        inputs.position = m_motorPosition.getValueAsDouble();
        inputs.supplyCurrent = m_motorSupplyCurrent.getValueAsDouble();
    }

    // Turn motors to Nuetral Mode
    public void stop() {
        m_motor.setControl(m_neutral);
        m_follower.setControl(m_neutral);
    }

    // Run Duty Cycle
    public void runDutyCycle(double output) {
        m_motor.setControl(m_duty.withOutput(output));
    }

    // Move to setpoint
    public void setControl(MotionMagicVoltage goal) {
        m_motor.setControl(goal.withSlot(1));
    }

    public void setPosition(double goal) {
        m_motor.setPosition(goal);
    }
}
