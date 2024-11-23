package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled()
public class autoCodeOct26 extends LinearOpMode {
    private Servo servoLeft;
    private Servo servoRight;
    private Servo servoPivot;
    private Servo servoClawLeft;
    private Servo servoClawRight;
    private Servo outtakePivotLeft;
    private Servo outtakePivotRight;
    private Servo outtakeClawLeft;
    private Servo outtakeClawRight;

    // Declare OpMode members for the linear slides
    private DcMotor leftHorizontal;
    private DcMotor rightHorizontal;
    private DcMotor leftVertical;
    private DcMotor rightVertical;
    private SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException{
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoRight.setDirection(Servo.Direction.REVERSE);
        servoLeft.setDirection(Servo.Direction.FORWARD);
        servoLeft.scaleRange(0, 0.65);
        servoRight.scaleRange(0, 0.65);
        servoLeft.setPosition(0);
        servoRight.setPosition(0);

        servoPivot = hardwareMap.get(Servo.class, "servoPivot");
        servoPivot.setDirection(Servo.Direction.FORWARD);
        servoPivot.scaleRange(0, 1);
        servoPivot.setPosition(0.5);

        servoClawLeft = hardwareMap.get(Servo.class, "servoClawLeft");
        servoClawRight = hardwareMap.get(Servo.class, "servoClawRight");
        servoClawLeft.setDirection(Servo.Direction.FORWARD);
        servoClawRight.setDirection(Servo.Direction.REVERSE);
        servoClawLeft.setPosition(0.3);
        servoClawRight.setPosition(0.3);

        outtakePivotLeft = hardwareMap.get(Servo.class, "outtakePivotLeft");
        outtakePivotRight = hardwareMap.get(Servo.class, "outtakePivotRight");
        outtakeClawLeft = hardwareMap.get(Servo.class, "outtakeClawLeft");
        outtakeClawRight = hardwareMap.get(Servo.class, "outtakeClawRight");
        outtakeClawRight.setDirection(Servo.Direction.REVERSE);
        outtakePivotRight.setDirection(Servo.Direction.REVERSE);
        outtakeClawLeft.scaleRange(0, 0.55);
        outtakeClawRight.scaleRange(0, 0.55);
        outtakeClawRight.setPosition(1);
        outtakeClawLeft.setPosition(1);
        outtakePivotLeft.setPosition(0.975);
        outtakePivotRight.setPosition(1);

        // Initialize the hardware variables for the linear slides
        leftHorizontal = hardwareMap.get(DcMotor.class, "left_slide");
        leftHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        rightHorizontal = hardwareMap.get(DcMotor.class, "right_slide");
        rightHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        leftVertical = hardwareMap.get(DcMotor.class, "left_vertical_slide");
        leftVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVertical = hardwareMap.get(DcMotor.class, "right_vertical_slide");


        // Set up encoders for horizontal slides
        leftHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        rightHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        leftVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive = new SampleMecanumDrive(hardwareMap);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(-22.5, 4.6, 0.765))
                .addTemporalMarker(0.05, () -> {
                    leftVertical.setPower(1);
                    rightVertical.setPower(1);
                })
                .addTemporalMarker(1.6, () -> {
                    leftVertical.setPower(0);
                    rightVertical.setPower(0);
                })
                .addTemporalMarker(1.7,() ->{
                    outtakePivotLeft.setPosition(0.5);
                    outtakePivotRight.setPosition(0.525);
                })
                .addTemporalMarker(3, () -> {
                    outtakePivotLeft.setPosition(0.5);
                    outtakePivotRight.setPosition(0.525);
                })
                .addTemporalMarker(3.2, () -> {
                    outtakeClawLeft.setPosition(0);
                    outtakeClawRight.setPosition(0);
                    outtakePivotLeft.setPosition(0.975);
                    outtakePivotRight.setPosition(1);
                })
                .addTemporalMarker(3.5, () -> {
                    leftVertical.setPower(-1);
                    rightVertical.setPower(-1);
                })
                .addTemporalMarker(5, () -> {
                    leftVertical.setPower(0);
                    rightVertical.setPower(0);
                })
                .waitSeconds(4)
                .lineToSplineHeading(new Pose2d(-12.85, 18.135, 1.563))

                .waitSeconds(100)

                /*
                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                    outtakeClawLeft.setPosition(0);
                    outtakeClawRight.setPosition(0);
                }).lineToSplineHeading(new Pose2d(-12.85,18.13, 1.56))
                .addTemporalMarker(0.1, () -> {
                    servoLeft.setPosition(1);
                    servoRight.setPosition(1);
                })
                .waitSeconds(0.5)
                .forward(10)
                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    servoClawLeft.setPosition(0.575);
                    servoClawRight.setPosition(0.575);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    servoLeft.setPosition(0);
                    servoRight.setPosition(0);
                }).UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    outtakeClawLeft.setPosition(1);
                    outtakeClawRight.setPosition(1);
                }).UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    servoClawLeft.setPosition(0.3);
                    servoClawRight.setPosition(0.3);
                })*/

                .build();

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectorySequence(traj1);
    }
    
    
}
