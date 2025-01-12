package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Submersible {
    // 42.75:18x27.5:12
    public final char[][] grid = {
            {'⬛', '⬛', '⬛', '⬛', '◾', '⬛', '⬛', '⬛', '◾', '◾', '⬛', '⬛', '⬛', '◾', '⬛', '⬛', '⬛', '⬛'},
            {'⬛', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬛'},
            {'◾', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '◾'},
            {'◾', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '◾'},
            {'⬛', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬛'},
            {'◾', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '◾'},
            {'◾', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '◾'},
            {'⬛', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬛'},
            {'◾', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '◾'},
            {'◾', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '◾'},
            {'⬛', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬜', '⬛'},
            {'⬛', '⬛', '⬛', '⬛', '◾', '⬛', '⬛', '⬛', '◾', '◾', '⬛', '⬛', '⬛', '◾', '⬛', '⬛', '⬛', '⬛'}
    };
    int[][] sampleCoord = new int[2][2];
    double sampleX = 5;
    double sampleY = 5;
    double sampleRotation = 0;

    public int[][] getSampleCoordinates(double x, double y, double rotation) {
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

    public String getDisplay() {
        StringBuilder display = new StringBuilder();
        for(int y=0;y<grid.length;y++) {
            for(int x=0;x<grid[y].length;x++) {
                if(x==sampleCoord[0][0]&&y==sampleCoord[0][1]) {
                    display.append("\uD83D\uDD34");
                    continue;
                }
                if(x==sampleCoord[1][0]&&y==sampleCoord[1][1]) {
                    display.append("\uD83D\uDFE5");
                    continue;
                }
                display.append(grid[y][x]);
            }
            display.append('\n');
        }
        return display.toString();
    }

    public void setSampleX(double x) {
        sampleX = x;
    }
    public void setSampleY(double y) {
        sampleY = y;
    }
    public void setSampleRotation(double rotation) {
        sampleRotation = rotation;
    }

    public double getSampleX() {
        return sampleX;
    }
    public double getSampleY() {
        return sampleY;
    }
    public double getSampleRotation() {
        return sampleRotation;
    }

    public void addX(double x) {
        sampleX += x;
    }
    public void addY(double y) {
        sampleY += y;
    }
    public void addRotation(double rotation) {
        sampleRotation += rotation;
    }

    public void update() {
        sampleCoord = getSampleCoordinates(sampleX, sampleY, sampleRotation);
    }

    public Pose2d xToPose() {
        return new Pose2d(11.5, 52+((13-sampleX)*2), 0);
    }
    public double yToSlide() {
        return ((8-sampleY)/4)+(0.25);
    }
    public double rotToPivot() {
        if(sampleRotation==90) return 0.2;
        if(sampleRotation==45) return 0.35;
        if(sampleRotation==0) return 0.5;
        if(sampleRotation==315) return 0.65;
        if(sampleRotation==270) return 0.8;
        return 0.5;
    }
}
