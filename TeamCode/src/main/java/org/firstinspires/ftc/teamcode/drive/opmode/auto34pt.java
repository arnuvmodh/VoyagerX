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
        traj1,
        traj2,
        traj3,
        traj4,
        traj5,
        traj6,
        traj7,
        idle
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
        leftVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVertical = hardwareMap.get(DcMotor.class, "right_vertical_slide");
        rightVertical.setDirection(DcMotorSimple.Direction.FORWARD);


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

        drive.setPoseEstimate(new Pose2d(0,0, 0));

        State curState = State.idle;


        //IMPLEMENT THESE TRAJECTORY SEQUENCES ASYNCHRONOUSLY TO FOLLOW AUTO PATH
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToSplineHeading(new Pose2d(-22.34, 5.069, 0.9))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-18.157, 6.4293, 1.3414))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(-22.34, 5.069, 0.9))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(-20.3645, 4.6906, 1.6023))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(-22.34, 5.069, 0.9))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(-23.1245, 5.9236, 1.7832))
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(new Pose2d(-22.34, 5.069, 0.9))
                .build();
        waitForStart();
        if (isStopRequested()) return;

        timer.reset();
        drive.followTrajectoryAsync(traj1);
        curState = State.traj1;
        boolean riseVertSlides = true;
        boolean lowerVertSlides = false;
        boolean extendHorSlides = false;
        boolean retractHorSlides = false;
        boolean oneTimeSwitch = true;

        timer.reset();
        while (opModeIsActive()) {
            switch (curState) {
                case idle:
                    break;
                case traj1:
                    if (timer.seconds() > 1.6) {
                        outtakePivotLeft.setPosition(0.5);
                        outtakePivotRight.setPosition(0.525);
                    }
                    if (timer.seconds() > 3) {
                        outtakeClawLeft.setPosition(0);
                        outtakeClawRight.setPosition(0);
                    }
                    if (!drive.isBusy() && timer.seconds() > 4) {
                        outtakePivotLeft.setPosition(0.975);
                        outtakePivotRight.setPosition(1);
                        drive.followTrajectoryAsync(traj2);
                        curState = State.traj2;
                        timer.reset();

                    }
                    break;
                case traj2:
                    if (oneTimeSwitch && timer.seconds() > 0.5) {
                        lowerVertSlides = true;
                        oneTimeSwitch = false;
                    }
                    if (!drive.isBusy() && timer.seconds() > 10) {
                        drive.followTrajectoryAsync(traj3);
                        curState = State.traj3;
                        timer.reset();
                    }
                    break;

                //use the following blocks as reference for different functions
                /*
                flipOut:
                    outtakePivotLeft.setPosition(0.5);
                    outtakePivotRight.setPosition(0.525);

                fall:
                    lowerVertSlides = true;


                flipIn:
                    outtakePivotLeft.setPosition(0.975);
                    outtakePivotRight.setPosition(1);

                outtakeLetGo:
                    outtakeClawLeft.setPosition(0);
                    outtakeClawRight.setPosition(0);

                outtakeGrab:
                    outtakeClawRight.setPosition(1);
                    outtakeClawLeft.setPosition(1);

                horizontalExtend:
                    extendHorSlides = true;

                horizontalRetract:
                    retractHorSlides = true;

                intakeLetGo:
                    servoClawLeft.setPosition(0.3);
                    servoClawRight.setPosition(0.3);

                intakeGrab:
                    servoClawLeft.setPosition(0.525);
                    servoClawRight.setPosition(0.525);

                intakeFlipOut:
                    servoLeft.setPosition(1);
                    servoRight.setPosition(1);

                intakeFlipIn:
                    servoLeft.setPosition(0);
                    servoRight.setPosition(0);


                 */
            }

            if (leftHorizontal.getCurrentPosition() < 1000 && rightHorizontal.getCurrentPosition() < 1000 && extendHorSlides) {
                leftHorizontal.setPower(1);
                rightHorizontal.setPower(1);
            } else {
                leftHorizontal.setPower(0);
                rightHorizontal.setPower(0);
                extendHorSlides = false;
            }

            if (leftHorizontal.getCurrentPosition() > 0 && rightHorizontal.getCurrentPosition() > 0 && retractHorSlides) {
                leftHorizontal.setPower(-1);
                rightHorizontal.setPower(-1);
            } else {
                leftHorizontal.setPower(0);
                rightHorizontal.setPower(0);
                retractHorSlides = false;
            }

            if (rightVertical.getCurrentPosition() > -3100 && riseVertSlides) {
                leftVertical.setPower(1);
                rightVertical.setPower(1);
            } else if (rightVertical.getCurrentPosition() < 0 && lowerVertSlides) {
                leftVertical.setPower(-1);
                rightVertical.setPower(-1);
            } else {
                leftVertical.setPower(0);
                rightVertical.setPower(0);
                riseVertSlides = false;
                lowerVertSlides = false;
            }


            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("rightVertical", rightVertical.getCurrentPosition());
            telemetry.addData("leftVertical", leftVertical.getCurrentPosition());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }


}
