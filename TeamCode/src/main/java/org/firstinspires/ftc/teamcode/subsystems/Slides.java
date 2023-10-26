package org.firstinspires.ftc.teamcode.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.LowPassEstimator;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;

import org.firstinspires.ftc.teamcode.util.Mechanism;

import org.firstinspires.ftc.teamcode.util.RunToPositionMotorUtil;

import java.util.function.DoubleSupplier;


public class Slides extends Mechanism{
    RunToPositionMotorUtil motorUtil = new RunToPositionMotorUtil();
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;

    int lastLeftPos;

    int lastRightPos;

    public double releaseSpeedLimit = 0.25;

    public String leftSlideName = "leftSlide";
    public String rightSlideName = "rightSlide";

    public boolean isSpeeding = false;

    public boolean isReset = true;

    @Override
    public void init(HardwareMap hwMap) {
        leftSlide = hwMap.get(DcMotorEx.class, leftSlideName);
        rightSlide = hwMap.get(DcMotorEx.class, rightSlideName);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop(Gamepad gamepad) {
        isSpeeding = getAverageSlidesSpeed() > releaseSpeedLimit;
        isReset = getAverageSlidesPosition() == 0;
    }

    public void setPower(double power) {
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        lastLeftPos = leftSlide.getCurrentPosition();
        lastRightPos = rightSlide.getCurrentPosition();
    }

    public void stop() {
        setPower(0);
    }

    public void setMode(DcMotor.RunMode mode) {
        leftSlide.setMode(mode);
        rightSlide.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftSlide.setZeroPowerBehavior(behavior);
        rightSlide.setZeroPowerBehavior(behavior);
    }

    public void holdPosition() {
        setSlidesPosition(lastLeftPos, lastRightPos, 1);
    }

    public void setSlidesPosition(int position, double speed) {
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorUtil.motorToPosition(leftSlide, 1, position);
        motorUtil.motorToPosition(rightSlide, 1, position);
    }

    public void setSlidesPosition(int leftPosition, int rightPosition, double speed) {
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorUtil.motorToPosition(leftSlide, 1, leftPosition);
        motorUtil.motorToPosition(rightSlide, 1, rightPosition);
    }
    public void resetSlidesPosition(double speed) {
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorUtil.motorToPosition(leftSlide, 1, 0);
        motorUtil.motorToPosition(rightSlide, 1, 0);
    }

    public int getAverageSlidesPosition() {
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition())/2;
    }

    public double getAverageSlidesSpeed() {
        return (leftSlide.getPower() + rightSlide.getPower())/2;
    }
}