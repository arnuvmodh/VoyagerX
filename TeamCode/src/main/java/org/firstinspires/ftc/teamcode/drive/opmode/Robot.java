package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.MotorSlide;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.ServoSlide;


public class Robot {
    public final Claw intakeClaw;
    public final Claw outtakeClaw;
    public final Pivot intakePivot;
    public final Pivot outtakePivot;
    public final Pivot clawPivot;

    public final MotorSlide verticalSlide;
    public final ServoSlide horizontalSlide;

    public Robot(HardwareMap hardwareMap) {
        intakeClaw = new Claw(hardwareMap, "intakeClawLeft", "intakeClawRight", 0, 1);
        outtakeClaw = new Claw(hardwareMap, "outtakeClawLeft", "outtakeClawRight", 0, 0.43);
        intakePivot = new Pivot(hardwareMap, "intakePivotLeft", "intakePivotRight", 0.21, 0.8);
        outtakePivot = new Pivot(hardwareMap, "outtakePivotLeft", "outtakePivotRight", 0, 1, -0.025);
        clawPivot = new Pivot(hardwareMap, "servoPivot", 0, 1);
        verticalSlide = new MotorSlide(hardwareMap, "leftVerticalSlide", "rightVerticalSlide", 50, 3050);
        horizontalSlide = new ServoSlide(hardwareMap, "leftHorizontalSlide", "rightHorizontalSlide", 0, 0.5);

    }
}