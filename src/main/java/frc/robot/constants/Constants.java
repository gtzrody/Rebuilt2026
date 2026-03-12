package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.util.Units;


public class Constants {
    
 public static class SwerveConstants {

        public static final double HUB_ROTATION_PID_KP = 2.0;
        public static final double HUB_ROTATION_PID_KI = 0.0;
        public static final double HUB_ROTATION_PID_KD = 0.0;

         public static final double CLOSE_TRANSLATION_PP_KP = 1.5;
        public static final double CLOSE_TRANSLATION_PP_KI = 0.0;
        public static final double CLOSE_TRANSLATION_PP_KD = 0.0;

        public static final double CLOSE_ROTATION_PP_KP = 2.0;
        public static final double CLOSE_ROTATION_PP_KI = 0.0;
        public static final double CLOSE_ROTATION_PP_KD = 0.0;


 }

  public static class AutoDriveConstants {
    public static final Pose2d[] RED_HUB_POSES = new Pose2d[] {
         new Pose2d(11.9013986, 4.6247558, new Rotation2d(0.7071067811865476)), // ID 2
            new Pose2d(11.2978438, 4.3769534, new Rotation2d(1.0)), // ID 3
            new Pose2d(11.2978438, 4.0213534, new Rotation2d(1.0)), // ID 4
            new Pose2d(11.9013986, 3.417951, new Rotation2d(0.7071067811865476)), // ID 5
            new Pose2d(12.2569986, 3.417951, new Rotation2d(0.7071067811865476)), // ID 8
            new Pose2d(12.5051566, 3.6657534, new Rotation2d(0.0)), // ID 9
            new Pose2d(12.5051566, 4.0213534, new Rotation2d(0.0)), // ID 10
            new Pose2d(12.2569986, 4.6247558, new Rotation2d(1.0471975511965974)) // ID 11
        };


        public static final Pose2d[] BLUE_HUB_POSES = new Pose2d[] {
            new Pose2d(4.6115986, 3.417951, new Rotation2d(0.7071067811865476)), // ID 18
            new Pose2d(5.2151534, 3.6657534, new Rotation2d(0.0)), // ID 19
            new Pose2d(5.2151534, 4.0213534, new Rotation2d(0.0)), // ID 20
            new Pose2d(4.6115986, 4.6247558, new Rotation2d(0.7071067811865476)), // ID 21
            new Pose2d(4.2559986, 4.6247558 , new Rotation2d(0.7071067811865476)), // ID 24
            new Pose2d(4.007866, 4.3769534, new Rotation2d(1.0)), // ID 25
            new Pose2d(4.007866, 4.0213534, new Rotation2d(1.0)), // ID 26
            new Pose2d(4.2559986, 3.417951, new Rotation2d(0.7071067811865476)) // ID 27
        };
         public static final int[] RED_HUB_IDS = {2, 3, 4, 5, 8, 9, 10, 11};

        public static final Translation2d RED_HUB_CENTER = new Translation2d(12, 4.0);
        public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.5, 4.0);

        public static final int[] GOOD_RED_APRIL_TAGS = {
            2, 3, 4, 5, 6, 9, 10, 11
        };

        public static final int[] GOOD_BLUE_APRIL_TAGS = {
            18, 19, 20, 21, 24, 25, 26, 27
        };

        public static final Pose2d[] BLUE_HUMAN_STATION_POSES = {
            new Pose2d(1.300, 1.000, new Rotation2d(55 * Math.PI / 180.0)),
            new Pose2d(1.300, 7.000, new Rotation2d(-55 * Math.PI / 180.0))
        };

        public static final Pose2d[] RED_HUMAN_STATION_POSES = {
            new Pose2d(16.300, 1.000, new Rotation2d(125 * Math.PI / 180.0)),
            new Pose2d(16.300, 7.000, new Rotation2d(-125 * Math.PI / 180.0))
        };

          public static final double[][] ADDITIONS = {
            {0.4, 0.08}, // LEFT ADDITION
            {0.42, -0.3}  // RIGHT ADDITION
        };

        public static final double[] POSE_ADDITION = {
            -0.4, 0.0
        };

    }
}