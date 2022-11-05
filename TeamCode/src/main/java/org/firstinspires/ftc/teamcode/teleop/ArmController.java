package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotorEx;

@TeleOp
@Config
public class ArmController extends LinearOpMode {

    SmartMotorEx jointMotor_1;
    SmartMotorEx jointMotor_2;

//    public static double kp_1=1;
//    public static double kp_2=1;
//
//    public static double tolerance_1=16;
//    public static double tolerance_2=16;
//
//    public static double DPP_1=1;
//    public static double DPP_2=1;
//
    public static int target_1 = 0;
    public static int target_2 = 0;
//
//    public static int startPose_1 = 0;
//    public static int startPose_2 = 0;
//
//    public static int groundPose_1 = 0;
//    public static int groundPose_2 = 0;
//
//    public static int lowPose_1 = 0;
//    public static int lowPose_2 = 0;
//
//    public static int midPose_1 = 0;
//    public static int midPose_2 = 0;
//
//    public static int highPose_1 = 0;
//    public static int highPose_2 = 0;

    public static double power = 0;
//
//    public static double p1 = SmartMotor.p;
//    public static double i1 = SmartMotor.i;
//    public static double d1 = SmartMotor.d;
//
//    public static double p2 = SmartMotor.p;
//    public static double i2 = SmartMotor.i;
//    public static double d2 = SmartMotor.d;

    RunMode runMode = RunMode.DeveloperInput;
    public enum RunMode {
        ControllerInput, DeveloperInput, EmergencyStop
    }

    Level level;
    public enum Level {
        StartLevel, GroundLevel, LowLevel, MidLevel, HighLevel
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        jointMotor_1 = new SmartMotorEx(hardwareMap, "joint1", SmartMotor.GoBILDA.RPM_312);
        jointMotor_2 = new SmartMotorEx(hardwareMap, "joint2", SmartMotor.GoBILDA.RPM_312);

        jointMotor_1.setRunMode(SmartMotor.RunMode.PositionControl);
        jointMotor_2.setRunMode(SmartMotor.RunMode.PositionControl);

        jointMotor_1.setZeroPowerBehavior(SmartMotor.ZeroPowerBehavior.BRAKE);
        jointMotor_2.setZeroPowerBehavior(SmartMotor.ZeroPowerBehavior.BRAKE);

//        jointMotor_1.setPositionCoefficient(kp_1);
//        jointMotor_2.setPositionCoefficient(kp_2);
//
//        jointMotor_1.setPositionTolerance(tolerance_1);
//        jointMotor_2.setPositionTolerance(tolerance_2);
//
//        jointMotor_1.setDistancePerPulse(DPP_1);
//        jointMotor_2.setDistancePerPulse(DPP_2);

        jointMotor_1.setInverted(false);
        jointMotor_2.setInverted(false);

        jointMotor_1.resetEncoder();
        jointMotor_2.resetEncoder();

        waitForStart();

        while (opModeIsActive()){

            RunModeSetter();
            ArmManager();

            jointMotor_1.set(power);
            jointMotor_2.set(power);

            TelemetryManager();

        }
    }

    public void MotorManager(int t1, int t2){
//        jointMotor_1.setPositionCoefficient(kp_1);
//        jointMotor_2.setPositionCoefficient(kp_2);
//
//        jointMotor_1.setPositionTolerance(tolerance_1);
//        jointMotor_2.setPositionTolerance(tolerance_2);
//
//        jointMotor_1.setDistancePerPulse(DPP_1);
//        jointMotor_2.setDistancePerPulse(DPP_2);

        MotorPositionManager(t1, t2);
    }

    public void StopMotors(){
        jointMotor_1.stopMotor();
        jointMotor_2.stopMotor();
    }

    public void MotorPositionManager(int t1, int t2){
        jointMotor_1.setTargetPosition(t1);
        jointMotor_2.setTargetPosition(t2);
    }

    public void RunModeSetter(){
        if(gamepad2.dpad_left) runMode = RunMode.DeveloperInput;
        else if(gamepad2.dpad_up) runMode = RunMode.ControllerInput;
        else if(gamepad2.touchpad) runMode = RunMode.EmergencyStop;
    }

    public void ArmManager(){
        if(runMode==RunMode.DeveloperInput){
            MotorManager(target_1, target_2);
        }
        else if(runMode==RunMode.ControllerInput){
//            if(level==Level.StartLevel) MotorManager();
//            else if(level==Level.GroundLevel) MotorManager();
//            else if(level==Level.LowLevel) MotorManager();
//            else if(level==Level.MidLevel) MotorManager();
//            else if(level==Level.HighLevel) MotorManager();

        }
        else if(runMode==RunMode.EmergencyStop){
            StopMotors();
        }
    }

    public void TelemetryManager(){
        telemetry.addData("joint 1 pose", jointMotor_1.getCurrentPosition());
        telemetry.addData("joint 2 pose", jointMotor_2.getCurrentPosition());
        telemetry.addData("target1", target_1);
        telemetry.addData("target2", target_2);
        telemetry.addData("joint 1 power", jointMotor_1.motor.getPower());
        telemetry.addData("joint 2 power", jointMotor_2.motor.getPower());
        telemetry.update();
    }

}
