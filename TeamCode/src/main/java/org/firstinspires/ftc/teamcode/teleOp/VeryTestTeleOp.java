package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.junction.JunctionAdjuster;
import org.firstinspires.ftc.teamcode.subsystem.MovementSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SubsystemData;
import org.firstinspires.ftc.teamcode.vision.WebcamUtil;

@TeleOp
public class VeryTestTeleOp extends LinearOpMode {

    public MovementSubsystem movementSubsystem = new MovementSubsystem();
    public GamepadEx driverGamepad;

    boolean homing = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driverGamepad = new GamepadEx(this.gamepad1);
        VeryTestTeleOp opMode = this;
        movementSubsystem.initSubsystem(this);

        try{
            WebcamUtil webcamUtil = new WebcamUtil(opMode.hardwareMap, opMode.telemetry);
            JunctionAdjuster junctionAdjuster = new JunctionAdjuster(webcamUtil, 2.54, opMode.telemetry, 45);
            webcamUtil.registerListener(junctionAdjuster);
            webcamUtil.start(true);
            opMode.telemetry.update();

            waitForStart();

            while(opModeIsActive()&&!isStopRequested()){
                driverGamepad.readButtons();
                if(driverGamepad.wasJustPressed(GamepadKeys.Button.B) && homing == false){
                    homing = true;
                    junctionAdjuster.start();
                }

                if(homing == false){
                    if (driverGamepad.getLeftX() != 0 || driverGamepad.getLeftY() != 0 || driverGamepad.getRightX() != 0 || driverGamepad.getRightY() != 0 || junctionAdjuster.inReach()) {
                        homing = false;
                    }
                }

                if(!homing){
                    movementSubsystem.run(new SubsystemData() {{
                        this.driverGamepad = opMode.driverGamepad;
                    }});
                    junctionAdjuster.moveCamera();
                }else{
                    JunctionAdjuster.visionResults results = junctionAdjuster.value();
                    movementSubsystem.move(results.movementData.getY(), results.movementData.getX(), results.turn);
                }

                telemetry.update();
            }
        }catch (Exception e){
            telemetry.addData("error", e);
            telemetry.update();
            throw e;
        }
    }
}
