// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import static frc.robot.Constants.LimelightConstants.*;

public class Limelight extends SubsystemBase {
    Drivetrain m_drivetrain;
    Alliance alliance;
    private String ll = kCameraName;
    private Boolean hasTarget = false;

    /** Creates a new Limelight. */
    public Limelight(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;

    }

    @Override
    public void periodic() {
        if (!Robot.isSimulation()) {
            LimelightHelpers.Results result = LimelightHelpers.getLatestResults(ll).targetingResults;
            if (result.valid && LimelightHelpers.getTA(kCameraName) > .25) {
                hasTarget = true;
            } else {
                hasTarget = false;
            }
            if (RobotConstants.kIsTuningMode) {
                SmartDashboard.putBoolean("Limelight has note detected", hasTarget);
            }

        }
    }

    public boolean hasTarget() {
        return hasTarget;
    }

}
