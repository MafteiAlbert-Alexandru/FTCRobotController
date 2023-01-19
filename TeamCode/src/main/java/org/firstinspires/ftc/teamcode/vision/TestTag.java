//package org.firstinspires.ftc.teamcode.vision;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@TeleOp
//public class TestTag extends LinearOpMode {
//
//    int autoCase;
//
//    @Override
//    public void runOpMode() {
//        try {
//            telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//            AprilTagUtil aprilTagUtil = new AprilTagUtil(this);
//            aprilTagUtil.open();
//            waitForStart();
//            autoCase=aprilTagUtil.getId(new int[]{1,2,3});
//            aprilTagUtil.close();
//
//            while (opModeIsActive()){
//                telemetry.addData("case", autoCase);
//                telemetry.update();
//            }
//        }
//        catch (Exception e){
//            telemetry.addLine(e.toString());
//            telemetry.update();
//        }
//    }
//}
