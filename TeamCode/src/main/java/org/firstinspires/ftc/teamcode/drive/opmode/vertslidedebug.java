package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Vertical Slide Debugging", group = "Test")
public class vertslidedebug extends LinearOpMode {

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
                robot.verticalSlide.goTo(1600, 1);
            } else if (gamepad1.b) {
                // Move slides back to a lower position when B is pressed
                robot.verticalSlide.goTo(0, 1);
            }
        }
    }
}