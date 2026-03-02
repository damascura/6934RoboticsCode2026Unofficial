package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class BasicOperations {
    /**
     * Gets the percentage of true values within a boolean array
     * @param attemptsList The boolean array to check
     * @return The percentage (as a decimal) of true values within the inputted boolean array
     */
    public static double getSuccessRate(boolean[] attemptsList) {
        int attempts = 0;
        int successes = 0;
        for (boolean attemptElement : attemptsList) {
            attempts++;
            if (attemptElement) {
                successes++;
            }
        }
        double successRate = (double) successes / attempts;
        return successRate;
    }

    /**
     * Inserts a boolean into a boolean array without expanding the array. The oldest boolean value is removed from the array
     * @param list The boolean array to insert the boolean into
     * @param newValue The boolean that is to be inserted
     */
    public static void insertBooleanToConfinedList(boolean[] list, boolean newValue) {
        for (int i = list.length - 1; i > 0; i--) {
            list[i] = list[i - 1];
        }
        list[0] = newValue;
    }

    /**
     * Inserts a double into a double array without expanding the array. The oldest double value is removed from the array
     * @deprecated Unused
     * @param list The double array to insert the double into
     * @param newValue The double that is to be inserted
     */
    public static void insertDoubleToConfinedList(double[] list, double newValue) {
        for (int i = list.length - 1; i > 0; i--) {
            list[i] = list[i - 1];
        }
        list[0] = newValue;
    }

    /**
     * Finds the average of two double values
     * @deprecated Unused
     * @param numOne A double value
     * @param numTwo Another double value
     * @return The average of the two inputted double values
     */
    public static double findAverage(double numOne, double numTwo) {
        return (numOne + numTwo) / 2;
    }

    /**
     * Finds the average of the double values within a double array
     * @deprecated Unused
     * @param numList A double array
     * @return The average of the double values within the inputted double array
     */
    public static double findAverageArray(double[] numList) {
        double total = 0;
        for (int i = 0; i < numList.length; i++) {
            total += numList[i];
        }
        double average = total / numList.length;
        return average;
    }

    /**
     * Translates the 2D position of a 2D pose value forwards or backwards, then adjusts the 2D rotation (yaw) of the 2D pose 
     * value. This is useful for converting a 2D pose value representing the center of an AprilTag to one representing a goal robot 
     * 2D pose value directly in front of the AprilTag
     * @param targetPose The 2D pose value to be transformed
     * @param robotOffset The distance (meters) to translate the 2D position of the 2D pose value forward by
     * @param angularOffset The angle (degrees) to adjust the 2D rotation of the 2D pose value by (CCW+)
     * @return The transformed Pose2D object
     */
    public static Pose2d findTranslatedPoseCenter(Pose2d targetPose, double robotOffset, double angularOffset) {
        double resultX = targetPose.getX() + (robotOffset * Math.cos(targetPose.getRotation().getRadians())); // Use the original angle to shift the position coordinates
        double resultY = targetPose.getY() + (robotOffset * Math.sin(targetPose.getRotation().getRadians()));
        double resultAngle = targetPose.getRotation().getDegrees() + angularOffset;
        return new Pose2d(resultX, resultY, Rotation2d.fromDegrees(resultAngle));
    }
    
    /**
     * Translates the 2D position of a 2D pose value left and rotates the robot by 180 degrees to account for reef rotation requirements, 
     * if applicable. This is useful for shifting a goal robot 2D pose value directly in front of the AprilTag leftward
     * @param translatedPoseCenter The 2D pose value to be transformed
     * @param leftOffset The distance (meters) to translate the 2D position of the 2D pose value leftward by. Do NOT use a negative value
     * @param accountForReefRotation If the 2D pose needs to be rotated 180 degrees to have a correct orientation
     * @return The transformed Pose2D object
     */
    public static Pose2d findTranslatedPoseLeft(Pose2d translatedPoseCenter, double leftOffset, boolean accountForReefRotation) { // Use findTranslatedPoseCenter to find translatedPoseCenter
        double lateralAngleOffset = Units.degreesToRadians(90);
        if (accountForReefRotation) {
            lateralAngleOffset += Units.degreesToRadians(180);
        }
        double resultX = translatedPoseCenter.getX() + (leftOffset * Math.cos(translatedPoseCenter.getRotation().getRadians() + lateralAngleOffset));
        double resultY = translatedPoseCenter.getY() + (leftOffset * Math.sin(translatedPoseCenter.getRotation().getRadians() + lateralAngleOffset));
        return new Pose2d(resultX, resultY, translatedPoseCenter.getRotation());
    }

    /**
     * Translates the 2D position of a 2D pose value right and rotates the robot by 180 degrees to account for reef rotation requirements, 
     * if applicable. This is useful for shifting a goal robot 2D pose value directly in front of the AprilTag rightward
     * @param translatedPoseCenter The 2D pose value to be transformed
     * @param leftOffset The distance (meters) to translate the 2D position of the 2D pose value rightward by. Do NOT use a negative value
     * @param accountForReefRotation If the 2D pose needs to be rotated 180 degrees to have a correct orientation
     * @return The transformed Pose2D object
     */
    public static Pose2d findTranslatedPoseRight(Pose2d translatedPoseCenter, double rightOffset, boolean accountForReefRotation) { // Use findTranslatedPoseCenter to find translatedPoseCenter
        double lateralAngleOffset = Units.degreesToRadians(-90);
        if (accountForReefRotation) {
            lateralAngleOffset += Units.degreesToRadians(180);
        }
        double resultX = translatedPoseCenter.getX() + (rightOffset * Math.cos(translatedPoseCenter.getRotation().getRadians() + lateralAngleOffset));
        double resultY = translatedPoseCenter.getY() + (rightOffset * Math.sin(translatedPoseCenter.getRotation().getRadians() + lateralAngleOffset));
        return new Pose2d(resultX, resultY, translatedPoseCenter.getRotation());
    }

    /**
     * Flips a 2D pose value for the blue alliance to become a mirror 2D pose value for the red alliance; this applies ONLY FOR REEFSCAPE!!!
     * @param blueAlliancePose The 2D pose value for the blue alliance that is to be flipped
     * @return A Pose2D object representing the new mirror 2D pose value for the red alliance
     */
    public static Pose2d transformBlueToRedAlliancePose(Pose2d blueAlliancePose) { 
        double resultX = Constants.Vision.fieldLayout.getFieldLength() - blueAlliancePose.getX();
        double resultY = Constants.Vision.fieldLayout.getFieldWidth() - blueAlliancePose.getY();
        double resultRotation = blueAlliancePose.getRotation().getDegrees() - 180;
        return new Pose2d(resultX, resultY, Rotation2d.fromDegrees(resultRotation));
    }
}
