package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.gobuilda.SpinTake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.MotorSlide;
import org.firstinspires.ftc.teamcode.subsystems.gobuilda.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.gobuilda.ServoSlide;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakePivot;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakePivot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot {
    public final SpinTake spintake;
    public final OuttakeClaw outtakeClaw;
    public final IntakePivot intakePivot;
    public final OuttakePivot outtakePivot;
    public final Pivot clawPivot;
    public final Servo hangPivot;

    public final MotorSlide verticalSlide;
    public final ServoSlide horizontalSlide;

    private final SampleMecanumDrive drive;

    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        spintake = new SpinTake(hardwareMap, "intakeClawLeft", "intakeClawRight");
        outtakeClaw = new OuttakeClaw(hardwareMap, "outtakeClawLeft", "outtakeClawRight", 0, 0.44);
        intakePivot = new IntakePivot(hardwareMap, "intakePivotLeft", "intakePivotRight", 0, 1);
        outtakePivot = new OuttakePivot(hardwareMap, "outtakePivotLeft", "outtakePivotRight", 0, 1, -0.025);
        clawPivot = new Pivot(hardwareMap, "servoPivot", 0, 1);
        hangPivot = hardwareMap.get(Servo.class, "servoHang");
        hangPivot.setDirection(Servo.Direction.REVERSE);

        verticalSlide = new MotorSlide(hardwareMap, "leftVerticalSlide", "rightVerticalSlide", 50, 3050);
        horizontalSlide = new ServoSlide(hardwareMap, "leftHorizontalSlide", "rightHorizontalSlide", 0, 0.4);
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