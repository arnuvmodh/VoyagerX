package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous()
public class autonomousCode extends LinearOpMode {
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
        outtakeClawRight.setPosition(0);
        outtakeClawLeft.setPosition(0);
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
                .strafeTo(new Vector2d(13, -34.37))
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
                })
                .waitSeconds(0.5)
                .back(6)
                .lineTo(new Vector2d(27.063, 48.622))
                .lineToSplineHeading(new Pose2d(4, 75.5, -0.793))
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    leftVertical.setPower(1);
                    rightVertical.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.95, () -> {
                    leftVertical.setPower(0);
                    rightVertical.setPower(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    outtakePivotLeft.setPosition(0.5);
                    outtakePivotRight.setPosition(0.525);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                    outtakeClawLeft.setPosition(0);
                    outtakeClawRight.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> {
                    outtakePivotLeft.setPosition(0.975);
                    outtakePivotRight.setPosition(1);
                })
                .waitSeconds(4)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    leftVertical.setPower(-1);
                    rightVertical.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.65, () -> {
                    leftVertical.setPower(0);
                    rightVertical.setPower(0);
                })
                .lineToSplineHeading(new Pose2d(6.081, 63.674, 0))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    servoLeft.setPosition(1);
                    servoRight.setPosition(1);
                })
                .waitSeconds(1)
                .forward(15)
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
                })
                .waitSeconds(0.75)
                .back(6)
                .lineToSplineHeading(new Pose2d(4, 75.5, -0.793))
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    leftVertical.setPower(1);
                    rightVertical.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.95, () -> {
                    leftVertical.setPower(0);
                    rightVertical.setPower(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    outtakePivotLeft.setPosition(0.5);
                    outtakePivotRight.setPosition(0.525);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                    outtakeClawLeft.setPosition(0);
                    outtakeClawRight.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.75, () -> {
                    outtakePivotLeft.setPosition(0.975);
                    outtakePivotRight.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(3, () ->{
                    leftVertical.setPower(-1);
                    rightVertical.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.2, () -> {
                    leftVertical.setPower(0);
                    rightVertical.setPower(0);
                })
                .waitSeconds(2.7)
                .lineToSplineHeading(new Pose2d(8.348, 63.54, 0))



                .waitSeconds(10)
                .lineToSplineHeading(new Pose2d(0, 0, 0))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectorySequence(traj1);
    }
    
    
}
