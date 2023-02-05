//package org.firstinspires.ftc.teamcode.teleOp;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//
//@TeleOp
//@Config
//public class IntakeTeleOP extends LinearOpMode {
//    public static double IntakePosition=0.62;
//    public static double IntakePower=0;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        try{
//            DcMotorEx leftIntake = hardwareMap.get(DcMotorEx.class, "leftIntake");
//            DcMotorEx rightIntake = hardwareMap.get(DcMotorEx.class, "rightIntake");
//            rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
//            Servo armServo = hardwareMap.get(Servo.class, q"arm");
//
//            waitForStart();
//            while(opModeIsActive()&&!isStopRequested())
//            {
//                armServo.setPosition(IntakePosition);
//                leftIntake.setPower(IntakePower);
//                rightIntake.setPower(IntakePower);
//                telemetry.addData("leftIntakeCurrent", leftIntake.getCurrent(CurrentUnit.MILLIAMPS));
//                telemetry.addData("rightIntakeCurrent", rightIntake.getCurrent(CurrentUnit.MILLIAMPS));
//                telemetry.update();
//            }
//        }catch (Exception e) {
//            telemetry.addLine(e.toString());
//            telemetry.update();
//        }
//
//
//
//
//
//    }
//}
