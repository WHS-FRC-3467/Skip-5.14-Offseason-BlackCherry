// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

/** Add your docs here. */
public class Constants {

    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }


    public static final class ExampleSimpleSubsystemConstants {
        public static final int ID_Motor = 0;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

            return m_configuration;
        }
    }

    public static final class ExampleComplexSubsystemConstants {
        public static final int ID_Motor = 1;
        public static final double upperLimit = Units.degreesToRadians(180);
        public static final double lowerLimit = Units.degreesToRadians(0);
        public static final double tolerance = Units.degreesToRadians(1);

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0;

            m_configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            m_configuration.Feedback.SensorToMechanismRatio = 1;

            m_configuration.Slot0.kP = 1; // output per unit of error in position (output/rotation)
            m_configuration.Slot0.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot0.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.Slot1.kG = 0; // output to overcome gravity (output)
            m_configuration.Slot1.kS = 0; // output to overcome static friction (output)
            m_configuration.Slot1.kV = 0; // output per unit of requested velocity (output/rps)
            m_configuration.Slot1.kA = 0; // unused, as there is no target acceleration
            m_configuration.Slot1.kP = 1; // output per unit of error in position (output/rotation)
            m_configuration.Slot1.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot1.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.MotionMagic.MotionMagicCruiseVelocity = 10;
            m_configuration.MotionMagic.MotionMagicAcceleration = 10;
            m_configuration.MotionMagic.MotionMagicJerk = 10;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

            return m_configuration;
        }
    }

    public static final class DriveConstants {
        public static final double headingAngleTolerance = 2.0;
        public static final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
        public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    }

    public static final class ClimberConstants {
        public static final int ID_LEADER = 25;
        public static final int ID_FOLLOWER = 26;

        public static final double upperLimit = Units.degreesToRadians(120);
        public static final double lowerLimit = Units.degreesToRadians(0);
        public static final double tolerance = Units.degreesToRadians(3);

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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
            m_configuration.Slot1.kP = 0; // output per unit of error in position (output/rotation)
            m_configuration.Slot1.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot1.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.MotionMagic.MotionMagicCruiseVelocity = 10;
            m_configuration.MotionMagic.MotionMagicAcceleration = 10;
            m_configuration.MotionMagic.MotionMagicJerk = 10;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

            return m_configuration;
        }
    }

    public static final class YSplitRollersConstants {
        public static final int ID_YSPLIT_ROLLER1 = 16;
        public static final int ID_YSPLIT_ROLLER2 = 17;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

            return m_configuration;
        }
    }

    public static final class ShooterPivotConstants {
        public static final int ID_MOTOR = 18;
        public static final int ID_ENCODER = 19;

        //RPS
        public static final double upperLimit = Units.degreesToRadians(75);
        public static final double lowerLimit = Units.degreesToRadians(0);
        public static final double tolerance = Units.degreesToRadians(.5);

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0;

            m_configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            m_configuration.Feedback.FeedbackRemoteSensorID = ID_ENCODER;
            m_configuration.Feedback.SensorToMechanismRatio = 5168.0/95.0;

            m_configuration.Slot0.kP = 1; // output per unit of error in position (output/rotation)
            m_configuration.Slot0.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot0.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.Slot1.kG = 0; // output to overcome gravity (output)
            m_configuration.Slot1.kS = 0; // output to overcome static friction (output)
            m_configuration.Slot1.kV = 0; // output per unit of requested velocity (output/rps)
            m_configuration.Slot1.kA = 0; // unused, as there is no target acceleration
            m_configuration.Slot1.kP = 1; // output per unit of error in position (output/rotation)
            m_configuration.Slot1.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot1.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.MotionMagic.MotionMagicCruiseVelocity = 10;
            m_configuration.MotionMagic.MotionMagicAcceleration = 10;
            m_configuration.MotionMagic.MotionMagicJerk = 10;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

            return m_configuration;
        }
    }

    public static final class ShooterRollersConstants {
        public static final int ID_LEADER = 20;
        public static final int ID_FOLLOWER = 21;

        //RPS
        public static final double upperLimit = 200.0;
        public static final double lowerLimit = -upperLimit;
        public static final double tolerance = 10;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0;

            m_configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            m_configuration.Feedback.SensorToMechanismRatio = 24.0/15.0;

            m_configuration.Slot0.kP = 1; // output per unit of error in position (output/rotation)
            m_configuration.Slot0.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot0.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.Slot1.kG = 0; // output to overcome gravity (output)
            m_configuration.Slot1.kS = 0; // output to overcome static friction (output)
            m_configuration.Slot1.kV = 0.13; // output per unit of requested velocity (output/rps)
            m_configuration.Slot1.kA = 0; // unused, as there is no target acceleration
            m_configuration.Slot1.kP = 0.03; // output per unit of error in position (output/rotation)
            m_configuration.Slot1.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot1.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.MotionMagic.MotionMagicCruiseVelocity = 10;
            m_configuration.MotionMagic.MotionMagicAcceleration = 10;
            m_configuration.MotionMagic.MotionMagicJerk = 10;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

            return m_configuration;
        }
    }

    public static final class IntakeRollersConstants {
        public static final int ID_Motor = 15;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

            return m_configuration;
        }
    }

    public static final class IntakeJointConstants {
        public static final int ID_Motor = 14;
    
        public static final double upperLimit = Units.degreesToRotations(0);
        public static final double lowerLimit = Units.degreesToRotations(-100);
        public static final double tolerance = Units.degreesToRotations(5);

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0;

            m_configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            m_configuration.Feedback.SensorToMechanismRatio = 500.0/7.0;

            m_configuration.Slot0.kP = 1; // output per unit of error in position (output/rotation)
            m_configuration.Slot0.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot0.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.Slot1.kG = 0; // output to overcome gravity (output)
            m_configuration.Slot1.kS = 0; // output to overcome static friction (output)
            m_configuration.Slot1.kV = 0; // output per unit of requested velocity (output/rps)
            m_configuration.Slot1.kA = 0; // unused, as there is no target acceleration
            m_configuration.Slot1.kP = 20; // output per unit of error in position (output/rotation)
            m_configuration.Slot1.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot1.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.MotionMagic.MotionMagicCruiseVelocity = 5;
            m_configuration.MotionMagic.MotionMagicAcceleration = 50;
            m_configuration.MotionMagic.MotionMagicJerk = 0;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

            return m_configuration;
        }
    }

    public static final class ElevatorRollersConstants {
        public static final int ID_Motor = 24;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

            return m_configuration;
        }
    }

    public static final class ElevatorJointConstants {
        public static final int ID_LEADER = 22;
        public static final int ID_FOLLOWER = 23;

        public static final double upperLimit = 1000;
        public static final double lowerLimit = 0;
        public static final double tolerance = Units.degreesToRotations(10);
        public static final double homingCurrent = 10.0;

        public static TalonFXConfiguration motorConfig() {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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
            m_configuration.Slot1.kP = 0; // output per unit of error in position (output/rotation)
            m_configuration.Slot1.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot1.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.MotionMagic.MotionMagicCruiseVelocity = 10;
            m_configuration.MotionMagic.MotionMagicAcceleration = 10;
            m_configuration.MotionMagic.MotionMagicJerk = 10;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

            return m_configuration;
        }
    }

    public static final class SensorConstants {

        public static final int ID_LC1 = 30;
        public static final int ID_LC2 = 31;
        public static final int PORT_BB1 = 0; //DIO port for beam break

    }
    public static final class ControllerConstants {

        public static final double leftTriggerMin = 0.3;
        public static final double leftTriggerMid = 0.6;

    }

    public static class FieldConstants {

        public static final Pose2d BLUE_SPEAKER = new Pose2d(Units.inchesToMeters(-1.5 + 12), Units.inchesToMeters(218.42), new Rotation2d(0));
        public static final Pose2d RED_SPEAKER = new Pose2d(Units.inchesToMeters(652.73 - 12), Units.inchesToMeters(218.42), new Rotation2d(Math.PI));
        public static final Pose2d BLUE_FEED = new Pose2d(1.25, 6.62, new Rotation2d(0));
        public static final Pose2d RED_FEED = new Pose2d(15.250, 6.62, new Rotation2d(0));
        public static final Pose2d BLUE_AMP = new Pose2d(Units.inchesToMeters(72.5),Units.inchesToMeters(323.00),new Rotation2d(Math.PI/2));
        public static final Pose2d RED_AMP = new Pose2d(Units.inchesToMeters(578.77),Units.inchesToMeters(323.00),new Rotation2d(-Math.PI/2));
        public static final double BLUE_AUTO_PENALTY_LINE = 9; // X distance from origin to center of the robot almost fully crossing the midline
        public static final double RED_AUTO_PENALTY_LINE = 7.4; // X distance from origin to center of the robot almost fully crossing the midline

        public static final Rotation2d ampAngle = new Rotation2d(Math.PI / 2);
    }

}

