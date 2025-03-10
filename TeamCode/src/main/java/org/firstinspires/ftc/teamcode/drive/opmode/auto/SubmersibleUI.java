package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class SubmersibleUI {
    private final int _id;
    private double _x = 0;
    private double _y = 0;
    private double _failStrafe = 5;
    private double increment = 1;

    public SubmersibleUI(int id) {
        _id = id;
    }

    private boolean upDown = false, downDown = false, leftDown = false, rightDown = false;
    private boolean aDown = false, bDown = false, xDown = false, yDown = false;
    private boolean leftBDown = false, rightBDown = false;
    boolean userInput(Gamepad gamepad, Telemetry telemetry) {
        telemetry.addData("Submersible Sample ID", _id);
        telemetry.addData("Increment", increment);
        telemetry.addData("X Position", _x);
        telemetry.addData("Intake Slide Position", _y);
        telemetry.addData("Fail Strafe", _failStrafe);
        telemetry.addLine("X - Confirm");
        telemetry.addLine("□ - Decrease Increment");
        telemetry.addLine("△ - Increase Increment");
        telemetry.addLine("Right Bumper - Increase Fail Strafe");
        telemetry.addLine("Left Bumper - Decrease Fail Strafe");
        telemetry.update();

        if(gamepad.dpad_up && !upDown) _y+=(increment/10);
        upDown = gamepad.dpad_up;
        if(gamepad.dpad_down && !downDown) _y-=(increment/10);
        downDown = gamepad.dpad_down;
        if(gamepad.dpad_left && !leftDown) _x-=increment;
        leftDown = gamepad.dpad_left;
        if(gamepad.dpad_right && !rightDown) _x+=increment;
        rightDown = gamepad.dpad_right;

//        if(gamepad.b && !bDown) {
//            _failStrafe = -1*_failStrafe;
//        }
//        bDown = gamepad.b;
        if(gamepad.left_bumper && !leftBDown) {
            _failStrafe -= increment;
        }
        leftBDown = gamepad.left_bumper;

        if(gamepad.right_bumper && !rightBDown) {
            _failStrafe += increment;
        }
        rightBDown = gamepad.right_bumper;

        if(gamepad.x && !xDown) {
            increment-=0.5;
        }
        xDown = gamepad.x;
        if(gamepad.y && !yDown) {
            increment+=0.5;
        }
        yDown = gamepad.y;

        if(gamepad.a && !aDown) {
            return true;
        }
        aDown = gamepad.a;
        return false;
    }

    Pose2d getTrajectory() {
        return new Pose2d(17.5, 60+_x, 6.2498);
    }

    Pose2d getPostSweepTrajectory() {
        return new Pose2d(16.75, 60+_x, 6.2498);
    }

    double getIntakeSlidePosition() {
        return _y;
    }

    double getFailStrafe() {
        return _failStrafe;
    }

    void reverseFailStrafe() {
        _failStrafe = -1*_failStrafe;
    }

    void offsetX() {
        _x+=_failStrafe;
    }


}
