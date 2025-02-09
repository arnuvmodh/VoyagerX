package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class PivotTest extends LinearOpMode{

    private Robot robot;
    private SampleMecanumDrive drive;
    double pivotPos = 0.75;

    boolean downDDown = false;
    boolean upDDown = false;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if(gamepad1.dpad_up && !upDDown) {
                pivotPos+=0.0025;
            }
            upDDown = gamepad1.dpad_up;
            if(gamepad1.dpad_down && !downDDown) {
                pivotPos-=0.0025;
            }
            downDDown = gamepad1.dpad_down;
            if(gamepad1.a) {
                robot.spintake.spinIn(1);
                robot.intakePivot.flipTo(pivotPos);
            }
            if(gamepad1.b) {
                robot.spintake.spinOut(1);
                robot.intakePivot.flipTo(pivotPos);
            }
            telemetry.addData("Intake Pivot Position", pivotPos);
            telemetry.update();
        }
    }
}
