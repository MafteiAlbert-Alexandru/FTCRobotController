//package org.firstinspires.ftc.teamcode.subsystem;
//
//import static org.firstinspires.ftc.teamcode.util.DriveConstants.MOTOR_VELO_PID;
//import static org.firstinspires.ftc.teamcode.util.DriveConstants.RUN_USING_ENCODER;
//import static org.firstinspires.ftc.teamcode.util.DriveConstants.TRACK_WIDTH;
//import static org.firstinspires.ftc.teamcode.util.DriveConstants.kA;
//import static org.firstinspires.ftc.teamcode.util.DriveConstants.kStatic;
//import static org.firstinspires.ftc.teamcode.util.DriveConstants.kV;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.drive.DriveSignal;
//import com.acmerobotics.roadrunner.drive.MecanumDrive;
//import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
//import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
//import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
//import org.firstinspires.ftc.teamcode.util.TrajectorySequenceRunnerCancelable;
//
//import java.util.ArrayList;
//import java.util.Arrays;
//import java.util.List;
//
//@Config
//
//public class MovementSubsystem extends SmartSubsystem{
//
//
//     /* These are motor constants that should be listed online for your motors.
//            */
//    public static final double TICKS_PER_REV = 537.6;
//    public static final double MAX_RPM = 312;
//
//    /*
//     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
//     * Set this flag to false if drive encoders are not present and an alternative localization
//     * method is in use (e.g., tracking wheels).
//     *
//     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
//     * from DriveVelocityPIDTuner.
//     */
//    public static final boolean RUN_USING_ENCODER = true;
//    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(10, 0, 2,
//            //getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV)
//            12.780802019843486);
//
//    /*
//     * These are physical constants that can be determined from your robot (including the track
//     * width; it will be tune empirically later although a rough estimate is important). Users are
//     * free to chose whichever linear distance unit they would like so long as it is consistently
//     * used. The default values were selected with inches in mind. Road runner uses radians for
//     * angular distances although most angular parameters are wrapped in Math.toRadians() for
//     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
//     */
//    public static double WHEEL_RADIUS = 1.8898; // in
//    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
//    public static double TRACK_WIDTH = 13.3858; // in
//
//    /*
//     * These are the feedforward parameters used to model the drive motor behavior. If you are using
//     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
//     * motor encoders or have elected not to use them for velocity control, these values should be
//     * empirically tuned.
//     */
//    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
//    public static double kA = 0;
//    public static double kStatic = 0;
//
//    /*
//     * These values are used to generate the trajectories for you robot. To ensure proper operation,
//     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
//     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
//     * small and gradually increase them later after everything is working. All distance units are
//     * inches.
//     */
//    /*
//     * Note from LearnRoadRunner.com:
//     * The velocity and acceleration constraints were calculated based on the following equation:
//     * ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI) * 0.85
//     * Resulting in 52.48291908330528 in/s.
//     * This is only 85% of the theoretical maximum velocity of the bot, following the recommendation above.
//     * This is capped at 85% because there are a number of variables that will prevent your bot from actually
//     * reaching this maximum velocity: voltage dropping over the game, bot weight, general mechanical inefficiencies, etc.
//     * However, you can push this higher yourself if you'd like. Perhaps raise it to 90-95% of the theoretically
//     * max velocity. The theoretically maximum velocity is 61.74461068624151 in/s.
//     * Just make sure that your bot can actually reach this maximum velocity. Path following will be detrimentally
//     * affected if it is aiming for a velocity not actually possible.
//     *
//     * The maximum acceleration is somewhat arbitrary and it is recommended that you tweak this yourself based on
//     * actual testing. Just set it at a reasonable value and keep increasing until your path following starts
//     * to degrade. As of now, it simply mirrors the velocity, resulting in 52.48291908330528 in/s/s
//     *
//     * Maximum Angular Velocity is calculated as: maximum velocity / trackWidth * (180 / Math.PI) but capped at 360°/s.
//     * You are free to raise this on your own if you would like. It is best determined through experimentation.
//
//     */
//    public static double MAX_VEL = 10;
//    public static double MAX_ACCEL = 10;
//    public static double MAX_ANG_VEL = 2;
//    public static double MAX_ANG_ACCEL = 2;
//
//
//    public static double encoderTicksToInches(double ticks) {
//        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
//    }
//
//    public static double rpmToVelocity(double rpm) {
//        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
//    }
//
//    public static double getMotorVelocityF(double ticksPerSecond) {
//        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
//        return 32767 / ticksPerSecond;
//    }
//    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
//    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);
//
//    public static double LATERAL_MULTIPLIER = 1;
//
//    public static double VX_WEIGHT = 1;
//    public static double VY_WEIGHT = 1;
//    public static double OMEGA_WEIGHT = 1;
//    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = MovementMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
//    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = MovementMecanumDrive.getAccelerationConstraint(MAX_ACCEL);
//    public class MovementMecanumDrive extends MecanumDrive {
//        private TrajectoryFollower follower;
//        private BNO055IMU imu;
//        private VoltageSensor batteryVoltageSensor;
//        private TrajectorySequenceRunnerCancelable trajectorySequenceRunner;
//        private List<DcMotorEx> motors;
//
//        public MovementMecanumDrive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight)
//        {
//            super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
//            follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
//                            new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
//            LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
//            LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
//
//            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
//
//            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
//                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//            }
//
//            // TODO: adjust the names of the following hardware devices to match your configuration
//            imu = hardwareMap.get(BNO055IMU.class, "imu");
//            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//            imu.initialize(parameters);
//
//            // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
//            // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
//            //
//            //             | +Z axis
//            //             |
//            //             |
//            //             |
//            //      _______|_____________     +Y axis
//            //     /       |_____________/|__________
//            //    /   REV / EXPANSION   //
//            //   /       / HUB         //
//            //  /_______/_____________//
//            // |_______/_____________|/
//            //        /
//            //       / +X axis
//            //
//            // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
//            // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
//            //
//            // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
//            // BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);
//
//            motors = Arrays.asList(frontLeft, backLeft, backRight, frontRight);
//
//            for (DcMotorEx motor : motors) {
//                MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
//                motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//                motor.setMotorType(motorConfigurationType);
//            }
//
//            if (RUN_USING_ENCODER) {
//                setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            }
//
//            setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//            if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
//                setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
//            }
//
//            // TODO: reverse any motors using DcMotorEx.setDirection()
//
//            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
//            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//
//            // TODO: if desired, use setLocalizer() to change the localization method
//            // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
//
//            trajectorySequenceRunner = new TrajectorySequenceRunnerCancelable(follower, HEADING_PID);
//        }
//        public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
//            return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
//        }
//
//        public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
//            return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
//        }
//
//        public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
//            return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
//        }
//
//        public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
//            return new TrajectorySequenceBuilder(
//                    startPose,
//                    VEL_CONSTRAINT, ACCEL_CONSTRAINT,
//                    MAX_ANG_VEL, MAX_ANG_ACCEL
//            );
//        }
//
//        public void turnAsync(double angle) {
//            trajectorySequenceRunner.followTrajectorySequenceAsync(
//                    trajectorySequenceBuilder(getPoseEstimate())
//                            .turn(angle)
//                            .build()
//            );
//        }
//
//        public void turn(double angle) {
//            turnAsync(angle);
//            waitForIdle();
//        }
//
//        public void followTrajectoryAsync(Trajectory trajectory) {
//            trajectorySequenceRunner.followTrajectorySequenceAsync(
//                    trajectorySequenceBuilder(trajectory.start())
//                            .addTrajectory(trajectory)
//                            .build()
//            );
//        }
//
//        public void followTrajectory(Trajectory trajectory) {
//            followTrajectoryAsync(trajectory);
//            waitForIdle();
//        }
//
//        public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
//            trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
//        }
//
//        public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
//            followTrajectorySequenceAsync(trajectorySequence);
//            waitForIdle();
//        }
//
//        public Pose2d getLastError() {
//            return trajectorySequenceRunner.getLastPoseError();
//        }
//
//        public void update() {
//            updatePoseEstimate();
//            DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
//            if (signal != null) setDriveSignal(signal);
//        }
//
//        public void waitForIdle() {
//            while (!Thread.currentThread().isInterrupted() && isBusy())
//                update();
//        }
//
//        public boolean isBusy() {
//            return trajectorySequenceRunner.isBusy();
//        }
//
//        public void setMode(DcMotor.RunMode runMode) {
//            for (DcMotorEx motor : motors) {
//                motor.setMode(runMode);
//            }
//        }
//
//        public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
//            for (DcMotorEx motor : motors) {
//                motor.setZeroPowerBehavior(zeroPowerBehavior);
//            }
//        }
//
//        public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
//            PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
//                    coefficients.p, coefficients.i, coefficients.d,
//                    coefficients.f * 12 / batteryVoltageSensor.getVoltage()
//            );
//
//            for (DcMotorEx motor : motors) {
//                motor.setPIDFCoefficients(runMode, compensatedCoefficients);
//            }
//        }
//
//        public void setWeightedDrivePower(Pose2d drivePower) {
//            Pose2d vel = drivePower;
//
//            if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
//                    + Math.abs(drivePower.getHeading()) > 1) {
//                // re-normalize the powers according to the weights
//                double denom = VX_WEIGHT * Math.abs(drivePower.getX())
//                        + VY_WEIGHT * Math.abs(drivePower.getY())
//                        + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());
//
//                vel = new Pose2d(
//                        VX_WEIGHT * drivePower.getX(),
//                        VY_WEIGHT * drivePower.getY(),
//                        OMEGA_WEIGHT * drivePower.getHeading()
//                ).div(denom);
//            }
//
//            setDrivePower(vel);
//        }
//
//        @NonNull
//        @Override
//        public List<Double> getWheelPositions() {
//            List<Double> wheelPositions = new ArrayList<>();
//            for (DcMotorEx motor : motors) {
//                wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
//            }
//            return wheelPositions;
//        }
//
//        @Override
//        public List<Double> getWheelVelocities() {
//            List<Double> wheelVelocities = new ArrayList<>();
//            for (DcMotorEx motor : motors) {
//                wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
//            }
//            return wheelVelocities;
//        }
//
//        @Override
//        public void setMotorPowers(double v, double v1, double v2, double v3) {
//            frontLeft.setPower(v);
//            backLeft.setPower(v1);
//            backRight.setPower(v2);
//            frontRight.setPower(v3);
//        }
//
//        @Override
//        public double getRawExternalHeading() {
//            return imu.getAngularOrientation().firstAngle;
//        }
//
//        @Override
//        public Double getExternalHeadingVelocity() {
//            return (double) imu.getAngularVelocity().zRotationRate;
//        }
//
//        public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
//            return new MinVelocityConstraint(Arrays.asList(
//                    new AngularVelocityConstraint(maxAngularVel),
//                    new MecanumVelocityConstraint(maxVel, trackWidth)
//            ));
//        }
//
//        public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
//            return new ProfileAccelerationConstraint(maxAccel);
//        }
//    };
//    private DcMotorEx frontLeft;
//    private DcMotorEx frontRight;
//    private DcMotorEx backLeft;
//    private DcMotorEx backRight;
//
//    @Override
//    public void initSubsystem(LinearOpMode linearOpMode, HardwareMap hardwareMap) {
//        super.initSubsystem(linearOpMode, hardwareMap);
//
//        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
//        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
//        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
//        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
//        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
//        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
//        backRight.setDirection(DcMotorEx.Direction.FORWARD);
//        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
//        MecanumDrive mecanum = new SampleMecanumDrive(frontLeft, frontRight,
//                backLeft, backRight);
//    }
//    public void run(GamepadEx gamepad){
//
//        double Forward = gamepad.getLeftY();
//        double Strafe = gamepad.getLeftX();
//        double Turn = gamepad.getRightX();
//
//        if(!gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)){
//            Forward /= 2;
//            Strafe  /= 2;
//        }
//        if(!gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
//            Turn /= 2;
//        }
//
//        double r = Math.hypot(Strafe, Forward);
//
//        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;
//
//        final double v1 = (r * Math.cos(robotAngle)) + Turn;
//        final double v2 = (r * Math.sin(robotAngle)) - Turn;
//        final double v3 = (r * Math.sin(robotAngle)) + Turn;
//        final double v4 = (r * Math.cos(robotAngle)) - Turn;
//        opMode.telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
//        opMode.telemetry.addData("frontRight", frontRight.getCurrentPosition());
//        opMode.telemetry.addData("backLeft", backLeft.getCurrentPosition());
//        opMode.telemetry.addData("backRight", backRight.getCurrentPosition());
//        frontLeft.setPower(v1);
//        frontRight.setPower(v2);
//        backLeft.setPower(v3);
//        backRight.setPower(v4);
//    }
//}
