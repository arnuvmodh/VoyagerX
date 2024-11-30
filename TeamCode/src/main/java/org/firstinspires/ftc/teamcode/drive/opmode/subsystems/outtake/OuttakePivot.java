package org.firstinspires.ftc.teamcode.drive.opmode.subsystems.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.Pivot;

public class OuttakePivot extends Pivot {
    private static final double SPECIMEN_GRAB_POSITION = 0.2;
    private static final double SPECIMEN_SCORE_POSITION = 0.65;
    public OuttakePivot(HardwareMap hardwareMap, String left, double min, double max) {
        super(hardwareMap, left, min, max);
    }

    public OuttakePivot(HardwareMap hardwareMap, String left, String right, double min, double max) {
        super(hardwareMap, left, right, min, max);
    }

    public OuttakePivot(HardwareMap hardwareMap, String left, String right, double min, double max, double offset) {
        super(hardwareMap, left, right, min, max, offset);
    }

    public void flipSpecimenGrab() {
        flipTo(SPECIMEN_GRAB_POSITION);
    }
    public void flipSpecimenScore() {
        flipTo(SPECIMEN_SCORE_POSITION);
    }
}