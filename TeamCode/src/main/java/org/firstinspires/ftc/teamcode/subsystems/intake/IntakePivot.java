package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.axon.AxonPivot;
import org.firstinspires.ftc.teamcode.subsystems.gobuilda.Pivot;

public class IntakePivot extends AxonPivot {
    public static final double UNDER_BAR_POSITION = 0.85;
    public IntakePivot(HardwareMap hardwareMap, String left, double min, double max) {
        super(hardwareMap, left, min, max);
    }

    public IntakePivot(HardwareMap hardwareMap, String left, String right, double min, double max) {
        super(hardwareMap, left, right, min, max);
    }

    public IntakePivot(HardwareMap hardwareMap, String left, String right, double min, double max, double offset) {
        super(hardwareMap, left, right, min, max, offset);
    }

    public void underBar() {
        flipTo(UNDER_BAR_POSITION);
    }

    public void flipIn() {

    }

    public void flipOut() {

    }
}
