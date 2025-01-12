package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.opmode.Robot;

@Disabled
public class MockUI extends LinearOpMode {
    private String repeat(String character, int count) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < count; i++) {
            builder.append(character);
        }
        return builder.toString();
    }

    private int[][] getSampleCoordinates(double x, double y, double rotation) {
        // Top Coordinate
        int topX = (int)x;
        int topY = (int)y;

        if(rotation>=45 && rotation<=135) x+=1;
        else if(rotation>=225 && rotation<=315) x-=1;

        if(rotation>=315 && rotation<=360 ||
        rotation>=0 && rotation<=45) y-=1;
        if(rotation>=135 && rotation<=225) y+=1;

        int[] top = {topX, topY};

        // Bottom Coordinate
        int[] bottom = {(int)Math.round(x), (int)Math.round(y)};

        return new int[][]{top, bottom};
    }

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        // 42.75:18x27.5:12
        char[][] submersible = {
                {'⬛', '⬛', '⬛', '⬛', '◾', '⬛', '⬛', '⬛', '◾', '◾', '⬛', '⬛', '⬛', '◾', '⬛', '⬛', '⬛', '⬛'},
                {'⬛', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬛'},
                {'⬛', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬛'},
                {'◾', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '◾'},
                {'⬛', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬛'},
                {'◾', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '◾'},
                {'◾', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '◾'},
                {'⬛', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬛'},
                {'◾', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '◾'},
                {'⬛', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬛'},
                {'⬛', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬛'},
                {'⬛', '⬛', '⬛', '⬛', '◾', '⬛', '⬛', '⬛', '◾', '◾', '⬛', '⬛', '⬛', '◾', '⬛', '⬛', '⬛', '⬛'}
        };
        telemetry.addLine("5 Sample Solo");
        double sampleX = 5;
        double sampleY = 5;
        double sampleRot = 0;
        int[][] sampleCoord = new int[2][2];
        boolean exit = false;

        boolean upDown = false, downDown = false, leftDown = false, rightDown = false;
        boolean aDown = false, bDown = false;
        while(!exit && !isStopRequested()) {
            if(gamepad1.dpad_up && !upDown) {
                sampleY -= 1;
            }
            upDown = gamepad1.dpad_up;
            if(gamepad1.dpad_down && !downDown) {
                sampleY += 1;
            }
            downDown = gamepad1.dpad_down;
            if(gamepad1.dpad_left && !leftDown) {
                sampleX-=1;
            }
            leftDown = gamepad1.dpad_left;
            if(gamepad1.dpad_right && !rightDown) {
                sampleX+=1;
            }
            rightDown = gamepad1.dpad_right;

            if(gamepad1.a && !aDown) {
                sampleRot-=45;
                if(sampleRot<=0) sampleRot=360+sampleRot;
            }
            aDown = gamepad1.a;
            if(gamepad1.b && !bDown) {
                sampleRot+=45;
                if(sampleRot>=360) sampleRot=sampleRot-360;
            }
            bDown = gamepad1.b;
            sampleCoord = getSampleCoordinates(sampleX, sampleY, sampleRot);

            for(int y=0;y<submersible.length;y++) {
                StringBuilder row = new StringBuilder();
                for(int x=0;x<submersible[y].length;x++) {
                    if(x==sampleCoord[0][0]&&y==sampleCoord[0][1]) {
                        row.append("\uD83D\uDD34");
                        continue;
                    }
                    if(x==sampleCoord[1][0]&&y==sampleCoord[1][1]) {
                        row.append("\uD83D\uDFE5");
                        continue;
                    }
                    row.append(submersible[y][x]);
                }
                telemetry.addLine(row.toString());
            }
            robot.horizontalSlide.goTo(((8-sampleY)/4)+(0.25));
            telemetry.addData("Y", sampleY);
            telemetry.addData("Calculated", ((8-sampleY)/4)+(0.25));
            telemetry.update();
            exit = gamepad1.share;
        }
        robot.horizontalSlide.goTo(0);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            // 4 to 9
            robot.horizontalSlide.goTo((9-sampleY)*0.2);
            telemetry.addLine("Running");
            telemetry.addData("Y", sampleY);
            telemetry.update();
        }
    }
}
