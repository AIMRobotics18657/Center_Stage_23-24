package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigInfo;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class Odometry extends Mechanism {

    public DcMotorEx par0, par1, perp;

    @Override
    public void init(HardwareMap hwMap) {
        par0 = hwMap.get(DcMotorEx.class, ConfigInfo.leftBack.getDeviceName());
        par1 = hwMap.get(DcMotorEx.class, ConfigInfo.rightBack.getDeviceName());
        perp = hwMap.get(DcMotorEx.class, ConfigInfo.leftFly.getDeviceName());

        par1.setDirection(DcMotorEx.Direction.REVERSE);

        par0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        par1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        perp.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getParTicks() {
        return (par0.getCurrentPosition() + par1.getCurrentPosition()) / 2.0;
    }

    public double getPerpTicks() {
        return perp.getCurrentPosition();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("par0 Position:", par0.getCurrentPosition());
        telemetry.addData("par1 Position:", par1.getCurrentPosition());
        telemetry.addData("perp Position:", perp.getCurrentPosition());
    }

    @Override
    public void systemsCheck(Gamepad gamepad, Telemetry telemetry) {
        telemetry.addData("par0 Position:", par0.getCurrentPosition());
        telemetry.addData("par1 Position:", par1.getCurrentPosition());
        telemetry.addData("perp Position:", perp.getCurrentPosition());
    }

    // Add any other necessary methods
}
