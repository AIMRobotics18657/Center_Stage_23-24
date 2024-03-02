package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class FinalsAutoConstants {

    public static final double ROBOT_WIDTH = 16.25;
    public static final double ROBOT_LENGTH = 15.0;
    public static final double WIDTH_OFFSET = ROBOT_WIDTH / 2;
    public static final double LENGTH_OFFSET = ROBOT_LENGTH / 2;
    public static final double RELEASE_OFFSET = LENGTH_OFFSET + 4;

    public static final Pose2d START_FAR_RED_POSE = new Pose2d(-36, -72 + LENGTH_OFFSET, Math.toRadians(90));
    public static final Pose2d START_NEAR_RED_POSE = new Pose2d(12, -72 + LENGTH_OFFSET, Math.toRadians(90));
    public static final Pose2d START_FAR_BLUE_POSE = new Pose2d(-36, 72 - LENGTH_OFFSET, Math.toRadians(270));
    public static final Pose2d START_NEAR_BLUE_POSE = new Pose2d(12, 72 - LENGTH_OFFSET, Math.toRadians(270));

    public static final Pose2d P_DROP_RED_1_A = new Pose2d(-48, -42, Math.toRadians(90));
    public static final double P_DROP_RED_1_A_TANGENT = Math.toRadians(90);
    public static final Pose2d P_DROP_RED_2_A = new Pose2d(-36, -39, Math.toRadians(90));
    public static final double P_DROP_RED_2_A_TANGENT = Math.toRadians(90);
    public static final Vector2d P_DROP_RED_3_A = new Vector2d(-40, -40);
    public static final Pose2d P_DROP_RED_3_B = new Pose2d(-36.25, -38, Math.toRadians(0));
    public static final double P_DROP_RED_3_B_TANGENT = Math.toRadians(0);

    public static final Vector2d P_DROP_BLUE_1_A = new Vector2d(-40, 40);
    public static final Pose2d P_DROP_BLUE_1_B = new Pose2d(-35.5, 37, Math.toRadians(0));
    public static final double P_DROP_BLUE_1_B_TANGENT = Math.toRadians(0);
    public static final Pose2d P_DROP_BLUE_2_A = new Pose2d(-36, 41, Math.toRadians(270));
    public static final double P_DROP_BLUE_2_A_TANGENT = Math.toRadians(90);
    public static final Pose2d P_DROP_BLUE_3_A = new Pose2d(-54, 44, Math.toRadians(270));
    public static final double P_DROP_BLUE_3_A_TANGENT = Math.toRadians(90);

    public static final Vector2d TRUSS_PASS_RED_A = new Vector2d(-36, -41);
    public static final Pose2d TRUSS_PASS_RED_B = new Pose2d(-40, -64.25, Math.toRadians(0));

    public static final Vector2d TRUSS_PASS_BLUE_A = new Vector2d(-36, 41);
    public static final Pose2d TRUSS_PASS_BLUE_B = new Pose2d(-40, 65, Math.toRadians(0));

    public static final Vector2d PREP_CLOSE_RED_A = new Vector2d(12, -42);
    public static final Pose2d PREP_CLOSE_RED_B = new Pose2d(16, -65, Math.toRadians(0));
    public static final Vector2d PREP_CLOSE_BLUE_A = new Vector2d(12, 42);
    public static final Pose2d PREP_CLOSE_BLUE_B = new Pose2d(16, 64, Math.toRadians(0));


    public static final Vector2d SPLINE_PRE_RED = new Vector2d(24, -63);
    public static final Vector2d SPLINE_POST_RED = new Vector2d(36, -36);
    public static final double SPLINE_POST_RED_HEADING = Math.toRadians(0);
    public static final double SPLINE_POST_RED_TANGENT = Math.toRadians(90);
    public static final double SPILINE_POST_RED_TANGENT_CLOSE = Math.toRadians(90);

    public static final Vector2d SPLINE_PRE_BLUE = new Vector2d(24, 64);
    public static final Vector2d SPLINE_POST_BLUE = new Vector2d(36, 36);
    public static final double SPLINE_POST_BLUE_HEADING = Math.toRadians(0);
    public static final double SPLINE_POST_BLUE_TANGENT = Math.toRadians(90);
    public static final double SPILINE_POST_BLUE_TANGENT_CLOSE = Math.toRadians(90);


    public static final Vector2d P_DROP_RED_1_A_CLOSE = new Vector2d(16, -40);
    public static final Pose2d P_DROP_RED_1_B_CLOSE = new Pose2d(12.5, -35, Math.toRadians(180));
    public static final double P_DROP_RED_1_B_TANGENT_CLOSE = Math.toRadians(0);
    public static final Pose2d P_DROP_RED_2_A_CLOSE = new Pose2d(12, -40, Math.toRadians(90));
    public static final double P_DROP_RED_2_A_TANGENT_CLOSE = Math.toRadians(90);
    public static final Pose2d P_DROP_RED_3_A_CLOSE = new Pose2d(28.25, -45, Math.toRadians(90));
    public static final double P_DROP_RED_3_A_TANGENT_CLOSE = Math.toRadians(90);

    public static final Pose2d P_DROP_BLUE_1_A_CLOSE = new Pose2d(26, 43, Math.toRadians(270));
    public static final double P_DROP_BLUE_1_A_TANGENT_CLOSE = Math.toRadians(90);
    public static final Pose2d P_DROP_BLUE_2_A_CLOSE = new Pose2d(12, 40, Math.toRadians(270));
    public static final double P_DROP_BLUE_2_A_TANGENT_CLOSE = Math.toRadians(90);
    public static final Vector2d P_DROP_BLUE_3_A_CLOSE = new Vector2d(16, 40);
    public static final Pose2d P_DROP_BLUE_3_B_CLOSE = new Pose2d(12, 37, Math.toRadians(180));
    public static final double P_DROP_BLUE_3_B_TANGENT_CLOSE = Math.toRadians(90);

    public static final double PIXEL_BOARD_X = 60.5;
    public static final double PIXEL_BOARD_HEADING = Math.toRadians(0);
    public static final double PIXEL_BOARD_X_DROP = PIXEL_BOARD_X - RELEASE_OFFSET;
    public static final Vector2d Y_DROP_RED_1 = new Vector2d(PIXEL_BOARD_X_DROP, -21.75);
    public static final Vector2d Y_DROP_RED_2 = new Vector2d(PIXEL_BOARD_X_DROP, -29);
    public static final Vector2d Y_DROP_RED_3 = new Vector2d(PIXEL_BOARD_X_DROP, -42);
    public static final Vector2d Y_DROP_BLUE_1 = new Vector2d(PIXEL_BOARD_X_DROP, 52);
    public static final Vector2d Y_DROP_BLUE_2 = new Vector2d(PIXEL_BOARD_X_DROP, 46);
    public static final Vector2d Y_DROP_BLUE_3 = new Vector2d(PIXEL_BOARD_X_DROP, 31);


    public static final Vector2d Y_DROP_RED_1_CLOSE = new Vector2d(PIXEL_BOARD_X_DROP, -22.25);
    public static final Vector2d Y_DROP_RED_2_CLOSE = new Vector2d(PIXEL_BOARD_X_DROP, -31);
    public static final Vector2d Y_DROP_RED_3_CLOSE = new Vector2d(PIXEL_BOARD_X_DROP, -42);
    public static final Vector2d Y_DROP_BLUE_1_CLOSE = new Vector2d(PIXEL_BOARD_X_DROP, 49.5);
    public static final Vector2d Y_DROP_BLUE_2_CLOSE = new Vector2d(PIXEL_BOARD_X_DROP, 40);
    public static final Vector2d Y_DROP_BLUE_3_CLOSE = new Vector2d(PIXEL_BOARD_X_DROP, 25);
    public static final double Y_DROP_RED_TANGENT = Math.toRadians(90);
    public static final double Y_DROP_BLUE_TANGENT = Math.toRadians(90);

    public static final Vector2d SAFE_PARK_TRANSITION_RED = new Vector2d(40, -23);
    public static final Vector2d SAFE_PARK_TRANSITION_BLUE = new Vector2d(40, 23);
    public static final Pose2d PARK_RED_AWAY = new Pose2d(48, -15, Math.toRadians(0));
    public static final Pose2d PARK_BLUE_AWAY = new Pose2d(48, 15, Math.toRadians(0));
    public static final Pose2d PARK_RED_WALL_SIDE = new Pose2d(50, -65, Math.toRadians(0));
    public static final Pose2d PARK_BLUE_WALL_SIDE = new Pose2d(50, 65, Math.toRadians(0));
}
