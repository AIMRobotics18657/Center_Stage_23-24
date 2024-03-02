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
    public IMU imu;

    public IMU_Controller(HardwareMap hwMap, boolean resetYAW) {
        imu = hwMap.get(IMU.class, ConfigInfo.imu.getDeviceName());
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection))
        );
        if (resetYAW) {
            imu.resetYaw();
        }
    }

    @Override
    public void init(HardwareMap hwMap) {

    }

    @Override
    public void loop(Gamepad gamepad) {

    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getHeadingError(double targetHeading) {
        return targetHeading - getHeading();
    }
}
