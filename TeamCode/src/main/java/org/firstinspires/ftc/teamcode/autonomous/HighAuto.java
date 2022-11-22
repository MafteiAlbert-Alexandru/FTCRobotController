//package org.firstinspires.ftc.teamcode.autonomous;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.noahbres.meepmeep.MeepMeep;
//import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
//import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
//import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//
//public class HighAuto {
//
//    public static void main(String args[]){
//
//        MeepMeep meepMeep = new MeepMeep(900);
//
//        Pose2d startPoseLeft = new Pose2d(-36, -60, Math.toRadians(90));
//
//        Pose2d leftBlueJunction = new Pose2d(-34, -7, Math.toRadians(30));
//
//        Pose2d leftBlueStack = new Pose2d(-55, -12, Math.toRadians(180));
//
//        RoadRunnerBotEntity myBotLeft = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(50, 50, 2, 2, 13.3)
//                .setDimensions(15, 16)
//                .setColorScheme(new ColorSchemeBlueDark())
//                .followTrajectorySequence(drive ->
//                                drive.trajectorySequenceBuilder(startPoseLeft)
//                                        .forward(52)//merg in fata cat sa indepartez signal ul
//                                        .back(4)//ma intorc in centru tile ului
//                                        .lineToLinearHeading(leftBlueJunction)
//
//                                        .splineToLinearHeading(leftBlueStack, Math.toRadians(140))
//                                        .splineToLinearHeading(leftBlueJunction, Math.toRadians(-200))
//
//                                        .splineToLinearHeading(leftBlueStack, Math.toRadians(140))
//                                        .splineToLinearHeading(leftBlueJunction, Math.toRadians(-200))
//
//
//                                        .splineToLinearHeading(leftBlueStack, Math.toRadians(140))
//                                        .splineToLinearHeading(leftBlueJunction, Math.toRadians(-200))
//
//
//                                        .splineToLinearHeading(leftBlueStack, Math.toRadians(140))
//                                        .splineToLinearHeading(leftBlueJunction, Math.toRadians(-200))
//
//
//                                        .splineToLinearHeading(leftBlueStack, Math.toRadians(140))
//                                        .splineToLinearHeading(leftBlueJunction, Math.toRadians(-200))
//
//
//                                        .build()
//                );
//
//
//        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.5f)
//                .addEntity(myBotLeft)
//                .start();
//    }
//}