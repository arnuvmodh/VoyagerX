package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.MotorSlide;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.ServoSlide;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.intake.IntakePivot;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.outtake.OuttakePivot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot {
    public final Claw intakeClaw;
    public final Claw outtakeClaw;
    public final Pivot intakePivot;
    public final Pivot outtakePivot;
    public final Pivot clawPivot;

    public final MotorSlide verticalSlide;
    public final ServoSlide horizontalSlide;

    private final SampleMecanumDrive drive;

    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        intakeClaw = new IntakeClaw(hardwareMap, "intakeClawLeft", "intakeClawRight", 0, 1);
        outtakeClaw = new OuttakeClaw(hardwareMap, "outtakeClawLeft", "outtakeClawRight", 0, 0.44);
        intakePivot = new IntakePivot(hardwareMap, "intakePivotLeft", "intakePivotRight", 0.205, 0.8);
        outtakePivot = new OuttakePivot(hardwareMap, "outtakePivotLeft", "outtakePivotRight", 0, 1, -0.025);
        clawPivot = new Pivot(hardwareMap, "servoPivot", 0, 1);
        verticalSlide = new MotorSlide(hardwareMap, "leftVerticalSlide", "rightVerticalSlide", 50, 3000);
        horizontalSlide = new ServoSlide(hardwareMap, "leftHorizontalSlide", "rightHorizontalSlide", 0, 0.5);
    }

    public double positionDifference(Pose2d currentPosition, Pose2d targetPosition) {
        double distance = Math.hypot(
                targetPosition.getX() - currentPosition.getX(),
                targetPosition.getY() - currentPosition.getY()
        );
        return distance;
    }
    public boolean isAt(Pose2d targetPosition, double tolerance) {
        Pose2d currentPosition = drive.getPoseEstimate();
        double positionDifference = positionDifference(currentPosition, targetPosition);
//        double headingDifference = Math.abs(targetPosition.getHeading() - currentPosition.getHeading());
//        headingDifference = (headingDifference + Math.PI) % (2 * Math.PI) - Math.PI;
        return positionDifference<=tolerance;
    }
}