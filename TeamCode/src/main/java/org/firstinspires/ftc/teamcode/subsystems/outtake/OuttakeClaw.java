package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.gobuilda.Claw;

public class OuttakeClaw extends Claw {
    private static final double GRAB_POSITION = 0.9;
    private static final double RELEASE_POSITION = 0.6;
    public OuttakeClaw(HardwareMap hardwareMap, String left, String right, double min, double max) {
        super(hardwareMap, left, right, min, max);
    }
    public void grab() {
        openTo(GRAB_POSITION);
    }
    public void release() {
        openTo(RELEASE_POSITION);
    }
}
