package org.firstinspires.ftc.teamcode.vision;

public enum CameraConfig {
    C920(578.272,578.272,402.145, 221.506, 78.0);

    private final double fx,fy,cx,cy, fovX;
    private int resolutionX, resolutionY;
    CameraConfig(double fx, double fy, double cx, double cy, double fovX) {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.fovX=fovX;
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

    public double getFovX() {
        return fovX;
    }
    public void setResolutionX(int resolutionX) {
        this.resolutionX = resolutionX;
    }

    public void setResolutionY(int resolutionY) {
        this.resolutionY = resolutionY;
    }

    public int getResolutionX() {
        return resolutionX;
    }
    public int getResolutionY(){
        return resolutionY;
    }
}
