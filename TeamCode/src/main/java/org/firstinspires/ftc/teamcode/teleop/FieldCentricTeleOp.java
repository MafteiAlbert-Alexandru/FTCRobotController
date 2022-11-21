package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;


@TeleOp(group = "advanced")
public class FieldCentricTeleOp extends LinearOpMode {

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        double targetAngle = 180;

        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            Pose2d poseEstimate = drive.getPoseEstimate();

            Vector2d input = new Vector2d(
                    InputLeftY(),
                    InputLeftX()
            ).rotated(-poseEstimate.getHeading());


            switch (currentMode) {
                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    input.getX(),
                                    input.getY(),
                                    InputRightX()
                            )
                    );

                    if (gamepad1.square) {
                        drive.turnAsync(Angle.normDelta(targetAngle + poseEstimate.getHeading()));

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.touchpad) {
                        //drive.breakFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }

            drive.update();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    double InputLeftX(){
        if(!gamepad1.left_bumper){
            return  -gamepad1.left_stick_x/2;
        }
        else return -gamepad1.left_stick_x;
    }

    double InputLeftY(){
        if(!gamepad1.left_bumper){
            return  -gamepad1.left_stick_y/2;
        }
        else return -gamepad1.left_stick_y;
    }

    double InputRightX(){
        if(!gamepad1.right_bumper){
            return  -gamepad1.right_stick_x/2;
        }
        else return -gamepad1.right_stick_x;
    }
}