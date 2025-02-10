package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class IntakeTest extends LinearOpMode{

    private Robot robot;
    private SampleMecanumDrive drive;

    boolean aDown = false;
    double startTime = -1;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if(gamepad1.a && !aDown) {
                robot.intakePivot.flipOut();
                startTime = timer.seconds() + 1;
            }
            aDown = gamepad1.a;
            robot.intake.sampleFast(robot, timer, startTime);
        }
    }
}
