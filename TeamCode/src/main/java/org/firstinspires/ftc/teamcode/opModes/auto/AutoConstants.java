package org.firstinspires.ftc.teamcode.opModes.auto;

public class AutoConstants {

    public static final double ROBOT_WIDTH = 16.25;
    public static final double ROBOT_LENGTH = 15.0;
    public static final double WIDTH_OFFSET = ROBOT_WIDTH / 2;
    public static final double LENGTH_OFFSET = ROBOT_LENGTH / 2;
    public static final double RELEASE_OFFSET = LENGTH_OFFSET + 4;

    public static final double START_X = 12;
    public static final double RED_START_Y = -72 + LENGTH_OFFSET;
    public static final double BLUE_START_Y = 72 - LENGTH_OFFSET;
    public static final double RED_START_HEADING = Math.toRadians(90);
    public static final double BLUE_START_HEADING = Math.toRadians(270);

    public static final double LEFT_PURPLE_X_OFFSET = -11;
    public static final double MIDDLE_PURPLE_X = 12;
    public static final double RIGHT_PURPLE_X_OFFSET = 11;
    public static final double RED_LEFT_RIGHT_PURPLE_Y = -30;
    public static final double RED_MIDDLE_PURPLE_Y = -24;
    public static final double BLUE_LEFT_RIGHT_PURPLE_Y = 30;
    public static final double BLUE_MIDDLE_PURPLE_Y = 24;

    public static final double LEFT_RELEASE_PURPLE_X = MIDDLE_PURPLE_X + LEFT_PURPLE_X_OFFSET + RELEASE_OFFSET;
    public static final double RED_LEFT_RELEASE_PURPLE_Y = RED_LEFT_RIGHT_PURPLE_Y;
    public static final double BLUE_LEFT_RELEASE_PURPLE_Y = BLUE_LEFT_RIGHT_PURPLE_Y;

    public static final double MIDDLE_RELEASE_PURPLE_X = MIDDLE_PURPLE_X + RELEASE_OFFSET;
    public static final double RED_MIDDLE_RELEASE_PURPLE_Y = RED_MIDDLE_PURPLE_Y;
    public static final double BLUE_MIDDLE_RELEASE_PURPLE_Y = BLUE_MIDDLE_PURPLE_Y;

    public static final double RIGHT_RELEASE_PURPLE_X = MIDDLE_PURPLE_X + RIGHT_PURPLE_X_OFFSET + RELEASE_OFFSET;
    public static final double RED_RIGHT_RELEASE_PURPLE_Y = RED_LEFT_RIGHT_PURPLE_Y;
    public static final double BLUE_RIGHT_RELEASE_PURPLE_Y = BLUE_LEFT_RIGHT_PURPLE_Y;

    public static final double RELEASE_PURPLE_HEADING = Math.toRadians(180);
    public static final double RELEASE_PURPLE_TANGENT = Math.toRadians(90);


    public static final double PIXEL_BOARD_X = 60;
    public static final double PIXEL_BOARD_PREP_X = PIXEL_BOARD_X - RELEASE_OFFSET - 6;
    public static final double RED_PIXEL_BOARD_PREP_Y = -36;
    public static final double BLUE_PIXEL_BOARD_PREP_Y = 36;
    public static final double PIXEL_BOARD_PREP_HEADING = Math.toRadians(0);

    public static final double PIXEL_BOARD_DROP_YELLOW_X = PIXEL_BOARD_X - RELEASE_OFFSET;
    public static final double RED_PIXEL_BOARD_MIDDLE_DROP_YELLOW_Y = -36;
    public static final double BLUE_PIXEL_BOARD_MIDDLE_DROP_YELLOW_Y = 36;
    public static final double PIXEL_BOARD_LEFT_DROP_YELLOW_Y_OFFSET = 7;
    public static final double PIXEL_BOARD_RIGHT_DROP_YELLOW_Y_OFFSET = -7;
    public static final double RED_PIXEL_BOARD_LEFT_DROP_YELLOW_Y = RED_PIXEL_BOARD_MIDDLE_DROP_YELLOW_Y + PIXEL_BOARD_LEFT_DROP_YELLOW_Y_OFFSET;
    public static final double RED_PIXEL_BOARD_RIGHT_DROP_YELLOW_Y = RED_PIXEL_BOARD_MIDDLE_DROP_YELLOW_Y + PIXEL_BOARD_RIGHT_DROP_YELLOW_Y_OFFSET;
    public static final double BLUE_PIXEL_BOARD_LEFT_DROP_YELLOW_Y = BLUE_PIXEL_BOARD_MIDDLE_DROP_YELLOW_Y + PIXEL_BOARD_LEFT_DROP_YELLOW_Y_OFFSET;
    public static final double BLUE_PIXEL_BOARD_RIGHT_DROP_YELLOW_Y = BLUE_PIXEL_BOARD_MIDDLE_DROP_YELLOW_Y + PIXEL_BOARD_RIGHT_DROP_YELLOW_Y_OFFSET;


    public static final double PARK_X = 48;
    public static final double RED_PARK_Y = -12;
    public static final double BLUE_PARK_Y = 12;
    public static final double PARK_HEADING = Math.toRadians(180);

    public static final double CAMERA_DROP_START_X = -36;
    public static final double CAMERA_DROP_RED_START_Y = -72 + LENGTH_OFFSET;
    public static final double CAMERA_DROP_BLUE_START_Y = 72 - LENGTH_OFFSET;
    public static final double CAMERA_DROP_START_HEADING = Math.toRadians(90);
    public static final double CAMERA_DROP_START_HEADING_BLUE = Math.toRadians(270);

    public static final double CAMERA_DROP_RED_LEFT_RELEASE_PURPLE_X = -39;
    public static final double CAMERA_DROP_RED_MIDDLE_RELEASE_PURPLE_X = -36;
    public static final double CAMERA_DROP_RED_RIGHT_RELEASE_PURPLE_X = -33;
    public static final double CAMERA_DROP_RED_LEFT_RIGHT_RELEASE_PURPLE_Y = -38;
    public static final double CAMERA_DROP_RED_MIDDLE_RELEASE_PURPLE_Y = -32;
    public static final double CAMERA_DROP_RED_LEFT_RELEASE_PURPLE_HEADING = Math.toRadians(135);
    public static final double CAMERA_DROP_RED_MIDDLE_RELEASE_PURPLE_HEADING = Math.toRadians(90);
    public static final double CAMERA_DROP_RED_RIGHT_RELEASE_PURPLE_HEADING = Math.toRadians(45);

    public static final double CAMERA_DROP_BLUE_LEFT_RELEASE_PURPLE_X = 33;
    public static final double CAMERA_DROP_BLUE_MIDDLE_RELEASE_PURPLE_X = 36;
    public static final double CAMERA_DROP_BLUE_RIGHT_RELEASE_PURPLE_X = 39;
    public static final double CAMERA_DROP_BLUE_LEFT_RIGHT_RELEASE_PURPLE_Y = 38;
    public static final double CAMERA_DROP_BLUE_MIDDLE_RELEASE_PURPLE_Y = 32;
    public static final double CAMERA_DROP_BLUE_LEFT_RELEASE_PURPLE_HEADING = Math.toRadians(315);
    public static final double CAMERA_DROP_BLUE_MIDDLE_RELEASE_PURPLE_HEADING = Math.toRadians(270);
    public static final double CAMERA_DROP_BLUE_RIGHT_RELEASE_PURPLE_HEADING = Math.toRadians(225);

}
