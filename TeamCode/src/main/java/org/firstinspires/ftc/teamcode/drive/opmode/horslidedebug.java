package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Horizontal Slide Debugging", group = "Test")
public class horslidedebug extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot
        robot = new Robot(hardwareMap);

        // Add telemetry to indicate initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                robot.horizontalSlide.goTo(1);
            } else if (gamepad1.b) {
                robot.horizontalSlide.goTo(0);
            }
            telemetry.addData("Left Slide Position", robot.horizontalSlide.getLeftPosition());
            telemetry.addData("Right Slide Position", robot.horizontalSlide.getRightPosition());
            telemetry.update();
        }
    }
}