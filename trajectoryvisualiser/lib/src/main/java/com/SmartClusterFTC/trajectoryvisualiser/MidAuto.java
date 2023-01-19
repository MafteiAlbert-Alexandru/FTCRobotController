//package com.SmartClusterFTC.trajectoryvisualiser;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.noahbres.meepmeep.MeepMeep;
//import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
//import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
//import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//
//
//public class MidAuto {
//
//    public static void main(String args[]){
//
//        MeepMeep mm = new MeepMeep(950);
//
//        Pose2d startPoseLeft = new Pose2d(36, -60, Math.toRadians(90));
//        Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));
//
//        Pose2d leftBlueMidPos = new Pose2d(36, -12, Math.toRadians(0));
//        Pose2d leftBlueStack = new Pose2d(55, -12, Math.toRadians(0));
//
//        Pose2d rightBlueJunction = new Pose2d(12, -12, Math.toRadians(0));
//
//
//        RoadRunnerBotEntity myBotLeft = new DefaultBotBuilder(mm)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(55, 55, 3, 3, 13.5)
//                .setDimensions(16, 17)
//                .setColorScheme(new ColorSchemeRedDark())
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(startPoseLeft)
//                                .forward(48)
//                                .turn(Math.toRadians(-90))
//                                //Pun pe stalp preload ul
//
//                                //Start cycle
//                                //1)
//                                .lineToLinearHeading(leftBlueStack)
////                                .lineToLinearHeading(leftBlueMidPos)
//                                .lineToLinearHeading(rightBlueJunction, SampleMecanumDriveCancelable.getVelocityConstraint(speedBack, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                                .strafeRight(12)
//                                .strafeLeft(12)
//
//                                //2)
//                                .lineToLinearHeading(leftBlueStack)
////                                .lineToLinearHeading(leftBlueMidPos)
//                                .lineToSplineHeading(rightBlueJunction)
//                                .strafeRight(12)
//                                .strafeLeft(12)
//
//                                //3)
//                                .lineToLinearHeading(leftBlueStack)
////                                .lineToLinearHeading(leftBlueMidPos)
//                                .lineToSplineHeading(rightBlueJunction)
//                                .strafeRight(12)
//                                .strafeLeft(12)
//
//                                //4)
//                                .lineToLinearHeading(leftBlueStack)
////                                .lineToLinearHeading(leftBlueMidPos)
//                                .lineToLinearHeading(rightBlueJunction)
//                                .strafeRight(12)
//                                .strafeLeft(12)
//
//                                //5)
//                                .lineToLinearHeading(leftBlueStack)
////                                .lineToLinearHeading(leftBlueMidPos)
//                                .lineToLinearHeading(rightBlueJunction)
//                                .strafeRight(12)
//                                .strafeLeft(12)
//
//                                //Park
//                                .build()
//                );
//
//        RoadRunnerBotEntity myBotLeft2 = new DefaultBotBuilder(mm)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(55, 55, 3, 3, 13.5)
//                .setDimensions(16, 17)
//                .setColorScheme(new ColorSchemeRedDark())
//                .followTrajectorySequence(drive ->
//                                drive.trajectorySequenceBuilder(startPoseLeft)
//                                        .forward(48)
//                                        .turn(Math.toRadians(90))
//                                        //Pun pe stalp preload ul
//
//                                        //Start cycle
//                                        //1)
//                                        .lineToLinearHeading(leftBlueStack)
////                                .lineToLinearHeading(leftBlueMidPos)
//                                        .lineToSplineHeading(leftBlueMidPos)
//                                        .turn(Math.toRadians(-135))
//                                        .turn(Math.toRadians(135))
//
//                                        //2)
//                                        .lineToLinearHeading(leftBlueStack)
////                                .lineToLinearHeading(leftBlueMidPos)
//                                        .lineToSplineHeading(leftBlueMidPos)
//                                        .turn(Math.toRadians(-135))
//                                        .turn(Math.toRadians(135))
//
//                                        //3)
//                                        .lineToLinearHeading(leftBlueStack)
////                                .lineToLinearHeading(leftBlueMidPos)
//                                        .lineToSplineHeading(leftBlueMidPos)
//                                        .turn(Math.toRadians(-135))
//                                        .turn(Math.toRadians(135))
//
//                                        //4)
//                                        .lineToLinearHeading(leftBlueStack)
////                                .lineToLinearHeading(leftBlueMidPos)
//                                        .lineToLinearHeading(leftBlueMidPos)
//                                        .turn(Math.toRadians(-135))
//                                        .turn(Math.toRadians(135))
//
//                                        //5)
//                                        .lineToLinearHeading(leftBlueStack)
////                                .lineToLinearHeading(leftBlueMidPos)
//                                        .lineToLinearHeading(leftBlueMidPos)
//                                        .turn(Math.toRadians(-135))
//                                        .turn(Math.toRadians(135))
//
//                                        //Park
//                                        .build()
//                );
//
//        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
//                .setDarkMode(true)
//                .setBackgroundAlpha(1)
//                .addEntity(myBotLeft)
////                .addEntity(myBotLeft2)
//                .start();
//    }
//}