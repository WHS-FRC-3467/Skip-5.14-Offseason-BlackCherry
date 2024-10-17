/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVisionConstants;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

//https://github.com/gladiatorsprogramming1591/Crescendo2024/blob/main/src/main/java/frc/robot/subsystems/DriveSubsystem.java#L794

public class PhotonVisionManager extends SubsystemBase {
    Drivetrain drivetrain;
    private PhotonPoseEstimator[] m_photonPoseEstimators;
    private PhotonCamera m_frontLeftCamera;
    private PhotonCamera m_frontRightCamera;
    private boolean hasTarget;
    AprilTagFieldLayout fieldLayout;
    private double field_margin = 0.5;

    public PhotonVisionManager(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        m_frontLeftCamera = new PhotonCamera(PhotonVisionConstants.front_left_cam.kCameraName);
        m_frontRightCamera = new PhotonCamera(PhotonVisionConstants.front_right_cam.kCameraName);

        m_photonPoseEstimators = new PhotonPoseEstimator[] {
                new PhotonPoseEstimator(
                    AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    m_frontLeftCamera,
                    PhotonVisionConstants.front_left_cam.kRobotToCam),
                 new PhotonPoseEstimator(
                     AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                     m_frontRightCamera,
                     PhotonVisionConstants.front_right_cam.kRobotToCam),
            };
        

    }

  
    @Override
    public void periodic() {
        Pose2d currentPose = drivetrain.getState().Pose;
            for (PhotonPoseEstimator poseEstimator : m_photonPoseEstimators) {
                // print out the time for this line to run 
                Optional<EstimatedRobotPose> pose = poseEstimator.update();
                if (pose.isPresent()) {
                    Pose3d pose3d = pose.get().estimatedPose;
                    Pose2d pose2d = pose3d.toPose2d();
                    if (
                        pose3d.getX() >= -field_margin &&
                        pose3d.getX() <= 16.5417 + field_margin &&
                        pose3d.getY() >= -field_margin &&
                        pose3d.getY() <= 16.5417 + field_margin &&
                        pose3d.getZ() >= -.75 &&
                        pose3d.getZ() <= 0.75
                    ) {
                        double sum = 0.0;
                        for (PhotonTrackedTarget target : pose.get().targetsUsed) {
                            Optional<Pose3d> tagPose =
                                fieldLayout.getTagPose(target.getFiducialId());
                            if (tagPose.isEmpty()) continue;
                            sum += currentPose.getTranslation().getDistance(tagPose.get().getTranslation().toTranslation2d());
                        }

                        int tagCount = pose.get().targetsUsed.size();
                        double stdScale = Math.pow(sum / tagCount, 2.0) / tagCount;
                        double xyStd = 0.02 * stdScale;
                        double rotStd = 0.035 * stdScale;
                        //time this as well
                        drivetrain.addVisionMeasurement(pose2d, pose.get().timestampSeconds, VecBuilder.fill(xyStd, xyStd, rotStd));
                        continue;
                    }
                }
            }

    }

    /**
     * Checks if the latest results from photonvision camera has a target
     * 
     * @return whether a target is found
     */
    public boolean hasTarget() {
        return hasTarget;
    }
}