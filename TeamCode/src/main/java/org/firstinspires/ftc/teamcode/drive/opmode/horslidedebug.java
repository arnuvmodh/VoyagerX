package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Outtake piv Debugging", group = "Test")
public class horslidedebug extends LinearOpMode {

    private Robot robot;
    private double outtakePosition = 1;
    private boolean aDown =  false;
    private boolean bDown = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot
        robot = new Robot(hardwareMap);

        // Add telemetry to indicate initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a && !aDown) {
                outtakePosition-=0.025;
            } else if (gamepad1.b && !bDown) {
                outtakePosition+=0.025;
            }
            aDown = gamepad1.a;
            bDown = gamepad1.b;
            robot.outtakePivot.flipTo(outtakePosition);
            telemetry.addData("Outtake Position", outtakePosition);
            telemetry.update();
        }
    }
}