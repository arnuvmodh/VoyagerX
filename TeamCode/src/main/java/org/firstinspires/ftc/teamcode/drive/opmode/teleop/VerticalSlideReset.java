package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.tuning.PoseStorage;

@TeleOp
public class VerticalSlideReset extends LinearOpMode {

    private DcMotor leftVertical = null;
    private DcMotor rightVertical = null;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        leftVertical = hardwareMap.get(DcMotor.class, "leftVerticalSlide");
        rightVertical = hardwareMap.get(DcMotor.class, "rightVerticalSlide");
        leftVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVertical.setDirection(DcMotorSimple.Direction.FORWARD);
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            leftVertical.setPower(-1);
            rightVertical.setPower(-1);
        }
    }
}
