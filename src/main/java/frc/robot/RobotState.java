// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Util.TunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

/** Add your docs here. */
public class RobotState {
    private static RobotState instance;

    @Getter
    @Setter
    private Pose2d robotPose = new Pose2d();

    @Getter
    @Setter
    private ChassisSpeeds robotSpeeds = new ChassisSpeeds();

    @RequiredArgsConstructor
    @Getter
    public enum TARGET { //Targets for aiming the robot, separate blue and red poses to allow for adjustment to field
        NONE(null,null),
        NOTE(null,null), //TODO: Add supplier from LL
        SUBWOOFER(Constants.FieldConstants.BLUE_SPEAKER,Constants.FieldConstants.RED_SPEAKER),
        SPEAKER(Constants.FieldConstants.BLUE_SPEAKER,Constants.FieldConstants.RED_SPEAKER),
        AMP(Constants.FieldConstants.BLUE_AMP,Constants.FieldConstants.RED_AMP),
        FEED(Constants.FieldConstants.BLUE_FEED,Constants.FieldConstants.RED_FEED);

        private final Pose2d blueTargetPose;
        private final Pose2d redTargetPose;

    }

    @Getter
    @Setter
    private TARGET target = TARGET.NONE;

    @Getter
    @Setter
    private OptionalDouble angleToNote = OptionalDouble.empty();

    //The amount of time into the future to predict pose (seconds)
    private double deltaT = .15; 

    @Getter
    TunableNumber shooterTuningAngle = new TunableNumber("Shooter Tuning Angle (deg)",20);

    @Getter
    TunableNumber shooterTuningSpeed = new TunableNumber("Shooter Tuning Speed (rps)",25);

    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }

    //Returns future pose based on current velocity
    private Translation2d getFuturePose() {
        //If magnitude of velocity is low enough, use current pose
        if (Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond) < .25) {
            return robotPose.getTranslation();
        } else {
            //Add translation based on current speed and time in the future deltaT
            return robotPose.getTranslation().plus(new Translation2d(deltaT * robotSpeeds.vxMetersPerSecond, deltaT * robotSpeeds.vyMetersPerSecond));
        }
        
    }

    //Return the angle to allign to target (cardinal)
    public Rotation2d getAngleOfTarget() {
        return (DriverStation.getAlliance().get() == Alliance.Blue) ? target.blueTargetPose.getRotation() : target.redTargetPose.getRotation();
    }

    //Return the angle to point towards the target (heading)
    public Rotation2d getAngleToTarget() {
        return ((DriverStation.getAlliance().get() == Alliance.Blue) ? target.blueTargetPose.getTranslation() : target.redTargetPose.getTranslation())
                .minus(getFuturePose())
                .getAngle();

    }

    //Return distance from target
    public double getDistanceToTarget() {
        if (target != TARGET.NONE && target != TARGET.NOTE) {
            return getFuturePose().getDistance(
                (DriverStation.getAlliance().get() == Alliance.Blue) ? target.blueTargetPose.getTranslation() : target.redTargetPose.getTranslation());
        } else {
            return -1;
        }
    }

    //Lookup table for speaker shots
    private static final InterpolatingDoubleTreeMap speakerAngleMap = new InterpolatingDoubleTreeMap();
    static {
        speakerAngleMap.put(1.01, 43.00);
        speakerAngleMap.put(2.15, 27.00);
        speakerAngleMap.put(2.56, 23.00);
        speakerAngleMap.put(3.0, 21.00);
        speakerAngleMap.put(3.5, 17.00);
        speakerAngleMap.put(4.02, 15.00);
        speakerAngleMap.put(4.6, 11.50);
        speakerAngleMap.put(4.95, 10.00);
        speakerAngleMap.put(5.5,9.00);
        speakerAngleMap.put(6.08,8.00);
    }

    //Lookup table for feeding over stage
    private static final InterpolatingDoubleTreeMap feedOverAngleMap = new InterpolatingDoubleTreeMap();
    static {
        feedOverAngleMap.put(8.00, 30.0);
        feedOverAngleMap.put(8.42, 28.0);
        feedOverAngleMap.put(9.0, 26.0);
        feedOverAngleMap.put(9.82, 25.0);
        feedOverAngleMap.put(11.0, 25.0);
    }

    //Returns angle for the shooter based on target and distance
    public double getShotAngle() {
        switch (target) {
            case SPEAKER:
                return speakerAngleMap.get(getDistanceToTarget());
            case FEED:
                if (getDistanceToTarget() < 8) {
                    return 0.5;
                } else {
                    return feedOverAngleMap.get(getDistanceToTarget()); 
                }
            default:
                return 0.0;
        }
    }

    public Command setTargetCommand(TARGET target) {
        return Commands.startEnd(() -> setTarget(target), () -> setTarget(TARGET.NONE))
                .withName("Set Robot Target: " + target.toString());
    }

}
