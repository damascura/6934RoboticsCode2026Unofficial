package frc.robot;

import frc.robot.Constants.Vision;
import frc.robot.LimelightHelpers.RawFiducial;
import org.littletonrobotics.junction.Logger;

public final class VisionInfo {
    private static final boolean[] targetValidResults = new boolean[Vision.targetDetectionListSize];

    private VisionInfo() {}

    /**
     * Gets the ID of the current AprilTag target.
     * @return The detected AprilTag ID, or -1 if no target is visible
     */
    public static int getTargetID() {
        if (!hasValidTargets()) {
            return -1;
        }
        return (int) LimelightHelpers.getFiducialID(Vision.limelightName);
    }

    /**
     * Gets the motor output or angle horizontal crosshair-to-target error.
     * @param asOutput If true, return a normalized value in [-1, 1]; if false, return degrees
     * @return Horizontal target error
     */
    public static double getTX(boolean asOutput) {
        if (asOutput) {
            return LimelightHelpers.getTX(Vision.limelightName) / 41.0; // LL 3G horizontal FoV is +/-41 deg
        }
        return LimelightHelpers.getTX(Vision.limelightName);
    }

    /**
     * Gets the motor output or angle vertical crosshair-to-target error.
     * @param asOutput If true, return a normalized value in [-1, 1]; if false, return degrees
     * @return Vertical target error
     */
    public static double getTY(boolean asOutput) {
        if (asOutput) {
            return LimelightHelpers.getTY(Vision.limelightName) / 28.1; // LL 3G vertical FoV is +/-28.1 deg
        }
        return LimelightHelpers.getTY(Vision.limelightName);
    }

    /**
     * Gets robot yaw error in target space from Limelight botpose_targetspace.
     * @return Yaw error in degrees
     */
    public static double getTagYawErrorDegrees() {
        double[] robotPose = LimelightHelpers.getBotPose_TargetSpace(Vision.limelightName);
        if (robotPose.length < 6) {
            return 0;
        }
        return robotPose[5];
    }

    /**
     * Gets nearest detected fiducial distance from robot center.
     * @return Distance in meters, or NaN if no fiducials are detected
     */
    public static double getNearestTagDistanceMeters() {
        RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(Vision.limelightName);
        if (rawFiducials.length == 0) {
            return Double.NaN;
        }

        double nearestDistance = Double.POSITIVE_INFINITY;
        for (RawFiducial fiducial : rawFiducials) {
            if (fiducial.distToRobot < nearestDistance) {
                nearestDistance = fiducial.distToRobot;
            }
        }
        return nearestDistance;
    }

    /**
     * Checks if the camera currently detects a target.
     * @return True if a target is visible, false otherwise
     */
    public static boolean hasValidTargets() {
        return LimelightHelpers.getTV(Vision.limelightName);
    }

    /**
     * Checks whether any of the specified fiducial IDs are currently visible.
     * @param ids Fiducial IDs to check
     * @return True if any listed IDs are detected, false otherwise
     */
    public static boolean hasAnyFiducialIds(int... ids) {
        if (ids == null || ids.length == 0) {
            return false;
        }
        RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(Vision.limelightName);
        if (rawFiducials.length == 0) {
            return false;
        }
        for (RawFiducial fiducial : rawFiducials) {
            for (int id : ids) {
                if (fiducial.id == id) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Checks whether the camera has reliably detected targets over the past few ticks.
     * @return True if recent target detection reliability exceeds threshold
     */
    public static boolean willTarget() {
        BasicOperations.insertBooleanToConfinedList(targetValidResults, hasValidTargets());
        return BasicOperations.getSuccessRate(targetValidResults) >= Vision.averageTVThreshold;
    }

    /**
     * Sets the Limelight fiducial point-of-interest X offset for branch/slot auto-align.
     * @param offsetXMeters Offset in meters
     */
    public static void setFiducialOffsetX(double offsetXMeters) {
        LimelightHelpers.setFiducial3DOffset(
            Vision.limelightName,
            offsetXMeters,
            Vision.autoAlignOffsetY,
            Vision.autoAlignOffsetZ
        );
    }

    /**
     * Resets Limelight fiducial point-of-interest offset to center.
     */
    public static void resetFiducialOffset() {
        setFiducialOffsetX(Vision.autoAlignCenterOffsetX);
    }

    /**
     * Forces the Limelight into the configured AprilTag pipeline.
     */
    public static void setAprilTagPipeline() {
        LimelightHelpers.setPipelineIndex(Vision.limelightName, Vision.aprilTagPipeline);
    }

    /**
     * Logs key vision and pose-estimator values to AdvantageKit.
     * @param estimatedX Estimated robot field X in meters
     * @param estimatedY Estimated robot field Y in meters
     * @param estimatedYaw Estimated robot yaw in degrees
     */
    public static void updateDashboardValues(double estimatedX, double estimatedY, double estimatedYaw) {
        Logger.recordOutput("Vision/ValidAprilTagDetected", hasValidTargets());
        Logger.recordOutput("Vision/HasTarget", willTarget());
        Logger.recordOutput("Vision/EstimatedPose/XMeters", estimatedX);
        Logger.recordOutput("Vision/EstimatedPose/YMeters", estimatedY);
        Logger.recordOutput("Vision/EstimatedPose/YawDegrees", estimatedYaw);

        Logger.recordOutput("Vision/TargetId", getTargetID());
        Logger.recordOutput("Vision/TXDegrees", getTX(false));
        Logger.recordOutput("Vision/TYDegrees", getTY(false));
        Logger.recordOutput("Vision/TagYawErrorDegrees", getTagYawErrorDegrees());

        double distance = getNearestTagDistanceMeters();
        Logger.recordOutput("Vision/TagDistanceMeters", Double.isNaN(distance) ? -1 : distance);
    }
}
