package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.MotorSlide;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.ServoSlide;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.intake.IntakePivot;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.outtake.OuttakePivot;


public class Robot {
    public final Claw intakeClaw;
    public final Claw outtakeClaw;
    public final Pivot intakePivot;
    public final Pivot outtakePivot;
    public final Pivot clawPivot;

    public final MotorSlide verticalSlide;
    public final ServoSlide horizontalSlide;

    public Robot(HardwareMap hardwareMap) {
        intakeClaw = new IntakeClaw(hardwareMap, "intakeClawLeft", "intakeClawRight", 0, 1);
        outtakeClaw = new OuttakeClaw(hardwareMap, "outtakeClawLeft", "outtakeClawRight", 0, 0.44);
        intakePivot = new IntakePivot(hardwareMap, "intakePivotLeft", "intakePivotRight", 0.19, 0.8);
        outtakePivot = new OuttakePivot(hardwareMap, "outtakePivotLeft", "outtakePivotRight", 0, 1, -0.025);
        clawPivot = new Pivot(hardwareMap, "servoPivot", 0, 1);
        verticalSlide = new MotorSlide(hardwareMap, "leftVerticalSlide", "rightVerticalSlide", 50, 3000);
        horizontalSlide = new ServoSlide(hardwareMap, "leftHorizontalSlide", "rightHorizontalSlide", 0, 0.5);

    }
}