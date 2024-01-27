package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigInfo;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class IMU_Controller extends Mechanism {

    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    private IMU imu;

        @Override
        public void init(HardwareMap hwMap) {
            imu = hwMap.get(IMU.class, ConfigInfo.imu.getDeviceName());
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    logoFacingDirection, usbFacingDirection))
            );

        }

        @Override
        public void loop(Gamepad gamepad) {

        }

        public double getHeading() {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
}
