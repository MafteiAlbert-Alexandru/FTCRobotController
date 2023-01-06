    //package org.firstinspires.ftc.teamcode.experiments;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotor;
//import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotorEx;
//
//@TeleOp
//@Config
//public class MrKrabs extends LinearOpMode {
//
//    SmartMotorEx baseMotor;
//    DcMotorEx jointMotor;
//
//    private PIDController controller;
//
//    public static double p = 0, i = 0, d = 0;
//
//    public static double f = 0.1;
//
//    public static double targetJoint = 0;
//
//    public static double minPosJoint = 0;
//
//    public static double maxPosJoint = 250;
//
//    final double ticks_in_degree_joint = 312 / 360.0;
//
//    public static double targetBase = 0;
//
////    public static double minPosBase = -200;
////
////    public static double maxPosBase = 200;
////
////    final double ticks_in_degree_Base = 537 / 180.0;
//
//    public static double powJoint = 0.002;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        baseMotor = new SmartMotorEx(hardwareMap, "base", SmartMotor.GoBILDA.RPM_223, SmartMotor.MotorDirection.REVERSE);
//
//        jointMotor = hardwareMap.get(DcMotorEx.class, "joint");
//        jointMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        jointMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        controller = new PIDController(p, i, d);
//
//        baseMotor.setRunMode(SmartMotor.RunMode.PositionControl);
//        baseMotor.resetEncoder();
//
//        waitForStart();
//
//
//        while (opModeIsActive()){
//
//            baseMotor.setTargetPosition((int)targetBase);
//            baseMotor.set(powJoint);
//
//            telemetry.addData("base pos", baseMotor.getCurrentPosition());
//            telemetry.addData("base target", targetBase);
//
//            PIDF_Arm();
//            telemetry.update();
//
//        }
//    }
//
//    public void PIDF_Arm() {
//
//        targetJoint = clamp(targetJoint, minPosJoint, maxPosJoint);
//
//        controller.setPID(p, i, d);
//        int armPos = jointMotor.getCurrentPosition();
//        double pid = controller.calculate(armPos, targetJoint);
//        double ff = Math.cos(Math.toRadians(targetJoint / ticks_in_degree_joint)) * f;
//
//
//        double power = pid + ff;
//
//        jointMotor.setPower(power);
//
//        telemetry.addData("pos joint", armPos);
//        telemetry.addData("targetJoint joint", targetJoint);
//        telemetry.addData("ff", ff);
//    }
//
//    public static double clamp(double val, double min, double max) {
//        return Math.max(min, Math.min(max, val));
//    }
//}
