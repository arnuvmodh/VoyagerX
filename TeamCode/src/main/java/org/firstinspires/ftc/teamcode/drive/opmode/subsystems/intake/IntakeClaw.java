package org.firstinspires.ftc.teamcode.drive.opmode.subsystems.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.Claw;

public class IntakeClaw extends Claw {
    private static final double GRAB_POSITION = 0.55;
    private static final double RELEASE_POSITION = 0.1;

    public IntakeClaw(HardwareMap hardwareMap, String left, String right, double min, double max) {
        super(hardwareMap, left, right, min, max);
    }

    public void grab() {
        openTo(GRAB_POSITION);
    }

    public void release() {
        openTo(RELEASE_POSITION);
    }
}
