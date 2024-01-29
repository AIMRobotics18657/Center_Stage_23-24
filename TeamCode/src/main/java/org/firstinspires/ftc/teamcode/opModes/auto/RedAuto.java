package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "RedAuto", group = "Tests")
public final class RedAuto extends LinearOpMode {
    Robot robot = new Robot(true, AutoConstants.START_X, AutoConstants.RED_START_Y, AutoConstants.RED_START_HEADING);
    int randomization = 1;

    private static final int sleepInSeconds = 1;

    Pose2d lastPurpleDropPose = new Pose2d(0, 0, 0);
    Pose2d lastYellowDropPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

//        Action driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
//                .splineToSplineHeading(randomization == 1 ? new Pose2d(AutoConstants.LEFT_RELEASE_PURPLE_X, AutoConstants.RED_LEFT_RELEASE_PURPLE_Y, AutoConstants.RELEASE_PURPLE_HEADING)
//                        : randomization == 2 ? new Pose2d(AutoConstants.MIDDLE_RELEASE_PURPLE_X, AutoConstants.RED_MIDDLE_RELEASE_PURPLE_Y, AutoConstants.RELEASE_PURPLE_HEADING)
//                        : new Pose2d(AutoConstants.RIGHT_RELEASE_PURPLE_X, AutoConstants.RED_RIGHT_RELEASE_PURPLE_Y, AutoConstants.RELEASE_PURPLE_HEADING), AutoConstants.RELEASE_PURPLE_TANGENT)
//                .build();
//
//        Action driveToPixelBoard = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
//                .strafeToLinearHeading(new Vector2d(AutoConstants.PIXEL_BOARD_PREP_X, AutoConstants.RED_PIXEL_BOARD_PREP_Y), AutoConstants.PIXEL_BOARD_PREP_HEADING)
//                .build();
//
//        Action driveToYellowDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
//                .strafeTo(new Vector2d(AutoConstants.PIXEL_BOARD_DROP_YELLOW_X, randomization == 1 ? AutoConstants.RED_PIXEL_BOARD_LEFT_DROP_YELLOW_Y
//                        : randomization == 2 ? AutoConstants.RED_PIXEL_BOARD_MIDDLE_DROP_YELLOW_Y
//                        : AutoConstants.RED_PIXEL_BOARD_RIGHT_DROP_YELLOW_Y))
//                .build();
//
//        Action driveToPark = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
//                .strafeToLinearHeading(new Vector2d(AutoConstants.PARK_X, AutoConstants.RED_PARK_Y), AutoConstants.PARK_HEADING)
//                .build();

        waitForStart();
        while (opModeIsActive()) {
//            randomization = robot.drivebase.getGameRandomization();
            randomization = 1;
            Action driveToPurpleDrop;
            if (randomization == 1) {
                lastPurpleDropPose = new Pose2d(AutoConstants.LEFT_RELEASE_PURPLE_X, AutoConstants.RED_LEFT_RELEASE_PURPLE_Y, AutoConstants.RELEASE_PURPLE_HEADING);
                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .splineToSplineHeading(new Pose2d(AutoConstants.LEFT_RELEASE_PURPLE_X, AutoConstants.RED_LEFT_RELEASE_PURPLE_Y, AutoConstants.RELEASE_PURPLE_HEADING), AutoConstants.RELEASE_PURPLE_TANGENT)
                        .build();
            } else if (randomization == 2) {
                lastPurpleDropPose = new Pose2d(AutoConstants.MIDDLE_RELEASE_PURPLE_X, AutoConstants.RED_MIDDLE_RELEASE_PURPLE_Y, AutoConstants.RELEASE_PURPLE_HEADING);
                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .splineToSplineHeading(new Pose2d(AutoConstants.MIDDLE_RELEASE_PURPLE_X, AutoConstants.RED_MIDDLE_RELEASE_PURPLE_Y, AutoConstants.RELEASE_PURPLE_HEADING), AutoConstants.RELEASE_PURPLE_TANGENT)
                        .build();
            } else {
                lastPurpleDropPose = new Pose2d(AutoConstants.RIGHT_RELEASE_PURPLE_X, AutoConstants.RED_RIGHT_RELEASE_PURPLE_Y, AutoConstants.RELEASE_PURPLE_HEADING);
                driveToPurpleDrop = robot.drivebase.drive.actionBuilder(robot.drivebase.drive.pose)
                        .splineToSplineHeading(new Pose2d(AutoConstants.RIGHT_RELEASE_PURPLE_X, AutoConstants.RED_RIGHT_RELEASE_PURPLE_Y, AutoConstants.RELEASE_PURPLE_HEADING), AutoConstants.RELEASE_PURPLE_TANGENT)
                        .build();
            }

            Action driveToPixelBoard = robot.drivebase.drive.actionBuilder(lastPurpleDropPose)
                    .strafeToLinearHeading(new Vector2d(AutoConstants.PIXEL_BOARD_PREP_X, AutoConstants.RED_PIXEL_BOARD_PREP_Y), AutoConstants.PIXEL_BOARD_PREP_HEADING)
                    .build();

            Action driveToYellowDrop;
            if (randomization == 1) {
                lastYellowDropPose = new Pose2d(AutoConstants.PIXEL_BOARD_DROP_YELLOW_X, AutoConstants.RED_PIXEL_BOARD_LEFT_DROP_YELLOW_Y, AutoConstants.PIXEL_BOARD_PREP_HEADING);
                driveToYellowDrop = robot.drivebase.drive.actionBuilder(new Pose2d(AutoConstants.PIXEL_BOARD_PREP_X, AutoConstants.RED_PIXEL_BOARD_PREP_Y, AutoConstants.PIXEL_BOARD_PREP_HEADING))
                        .strafeTo(new Vector2d(AutoConstants.PIXEL_BOARD_DROP_YELLOW_X, AutoConstants.RED_PIXEL_BOARD_LEFT_DROP_YELLOW_Y))
                        .build();
            } else if (randomization == 2) {
                lastYellowDropPose = new Pose2d(AutoConstants.PIXEL_BOARD_DROP_YELLOW_X, AutoConstants.RED_PIXEL_BOARD_MIDDLE_DROP_YELLOW_Y, AutoConstants.PIXEL_BOARD_PREP_HEADING);
                driveToYellowDrop = robot.drivebase.drive.actionBuilder(new Pose2d(AutoConstants.PIXEL_BOARD_PREP_X, AutoConstants.RED_PIXEL_BOARD_PREP_Y, AutoConstants.PIXEL_BOARD_PREP_HEADING))
                        .strafeTo(new Vector2d(AutoConstants.PIXEL_BOARD_DROP_YELLOW_X, AutoConstants.RED_PIXEL_BOARD_MIDDLE_DROP_YELLOW_Y))
                        .build();
            } else {
                lastYellowDropPose = new Pose2d(AutoConstants.PIXEL_BOARD_DROP_YELLOW_X, AutoConstants.RED_PIXEL_BOARD_RIGHT_DROP_YELLOW_Y, AutoConstants.PIXEL_BOARD_PREP_HEADING);
                driveToYellowDrop = robot.drivebase.drive.actionBuilder(new Pose2d(AutoConstants.PIXEL_BOARD_PREP_X, AutoConstants.RED_PIXEL_BOARD_PREP_Y, AutoConstants.PIXEL_BOARD_PREP_HEADING))
                        .strafeTo(new Vector2d(AutoConstants.PIXEL_BOARD_DROP_YELLOW_X, AutoConstants.RED_PIXEL_BOARD_RIGHT_DROP_YELLOW_Y))
                        .build();
            }

            Action driveToPark = robot.drivebase.drive.actionBuilder(lastYellowDropPose)
                    .strafeToLinearHeading(new Vector2d(AutoConstants.PARK_X, AutoConstants.RED_PARK_Y), AutoConstants.PARK_HEADING)
                    .build();

            //DRIVE TO DROP PIXEL
            Actions.runBlocking(
                    new SequentialAction(
                            driveToPurpleDrop,
//                            (telemetryPacket) -> { // Drop Purple
//                                robot.pixelManipulator.claw.releaseServo(robot.pixelManipulator.claw.leftClamp);
//                                return false;
//                            },
                            new SleepAction(sleepInSeconds),
                            driveToPixelBoard,
//                            new ParallelAction(
//                                    (telemetryPacket) -> { // Extend Arm
//                                        robot.pixelManipulator.arm.extend();
//                                        return false;
//                                    },
//                                    (telemetryPacket) -> { // Lift Slides
//                                        robot.pixelManipulator.slides.update(PURPLE_DROP_POS);
//                                        return robot.pixelManipulator.slides.isAtTargetPosition();
//                                    }
//                            ),
                            new SleepAction(sleepInSeconds),
                            driveToYellowDrop,
//                            (telemetryPacket) -> { // Drop Yellow
//                                robot.pixelManipulator.claw.releaseServo(robot.pixelManipulator.claw.rightClamp);
//                                return false;
//                            },
//                            new SleepAction(sleepMs),
//                            (telemetryPacket) -> { // Retract Arm
//                                robot.pixelManipulator.arm.retract();
//                                return robot.pixelManipulator.arm.isRetracted;
//                            },
                            new SleepAction(sleepInSeconds),
                            driveToPark
//                            (telemetryPacket) -> { // Retract Slides
//                                robot.pixelManipulator.slides.update(RESET_POS);
//                                return robot.pixelManipulator.slides.isAtTargetPosition();
//                            }
                    )
            );
            break;
        }
    }
}
