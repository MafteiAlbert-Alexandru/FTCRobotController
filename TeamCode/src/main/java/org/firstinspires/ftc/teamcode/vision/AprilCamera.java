package org.firstinspires.ftc.teamcode.vision;

public enum AprilCamera {
    C920(578.272,578.272,402.145, 221.506);

    private final double fx,fy,cx,cy;
    AprilCamera(double fx, double fy, double cx, double cy) {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
    }

    public double getCx() {
        return cx;
    }

    public double getCy() {
        return cy;
    }

    public double getFx() {
        return fx;
    }

    public double getFy() {
        return fy;
    }
}
