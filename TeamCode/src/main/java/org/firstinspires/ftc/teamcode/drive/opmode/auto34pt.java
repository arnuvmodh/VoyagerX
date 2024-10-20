package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous()
public class auto34pt extends LinearOpMode {
    enum State {
        idle,
        backUp,
        scoreSpecimen,
        scoreBasket,
        readyIntake,
        activateIntake,
        outtakeLift
    }


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
        leftHorizontal.setDirection(DcMotorSimple.Direction.FORWARD);
        rightHorizontal = hardwareMap.get(DcMotor.class, "right_slide");
        rightHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        leftVertical = hardwareMap.get(DcMotor.class, "left_vertical_slide");
        leftVertical.setDirection(DcMotorSimple.Direction.FORWARD);
        rightVertical = hardwareMap.get(DcMotor.class, "right_vertical_slide");
        rightVertical.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set up encoders for horizontal slides
        leftHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ElapsedTime timer = new ElapsedTime();

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0,0, Math.PI));

        State curState = State.idle;

        Trajectory backUp = drive.trajectoryBuilder(new Pose2d(0, 0, Math.PI))
                .back(29)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        timer.reset();
        drive.followTrajectoryAsync(backUp);
        curState = State.backUp;

        while (opModeIsActive()) {
            switch (curState) {
                case idle:
                    break;
                case backUp:
                    if (!drive.isBusy()){
                        curState = State.scoreSpecimen;
                        timer.reset();
                    }
                    break;
                case scoreSpecimen:
                    outtakePivotLeft.setPosition(0.5);
                    outtakePivotRight.setPosition(0.525);
                    if (rightVertical.getCurrentPosition() < 1500) {
                        leftVertical.setPower(0.3);
                        rightVertical.setPower(0.3);
                    } else {
                        outtakeClawLeft.setPosition(0);
                        outtakeClawRight.setPosition(0);
                        leftVertical.setPower(0);
                        rightVertical.setPower(0);
                        outtakePivotLeft.setPosition(0.975);
                        outtakePivotRight.setPosition(1);
                    }
                    if (timer.seconds() > 10) {
                        curState = State.idle;
                        leftVertical.setPower(0);
                        rightVertical.setPower(0);
                    }
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }


}
