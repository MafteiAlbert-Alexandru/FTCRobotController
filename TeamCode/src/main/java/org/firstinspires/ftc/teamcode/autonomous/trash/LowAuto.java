//package org.firstinspires.ftc.teamcode.autonomous;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.noahbres.meepmeep.MeepMeep;
//import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
//import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
//import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//
//public class LowAuto {
//
//    public static void main(String args[]){
//
//        MeepMeep mm = new MeepMeep(950);
//
//        Pose2d startPoseright = new Pose2d(-36, -60, Math.toRadians(90));
//        Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));
//
//        Pose2d rightBlueJunction = new Pose2d(-24, -12, Math.toRadians(90));
//        Pose2d rightBlueStack = new Pose2d(-55, -13, Math.toRadians(175));
//
//        RoadRunnerBotEntity myBotright = new DefaultBotBuilder(mm)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(55, 55, 3, 3, 13.5)
//                .setDimensions(15, 16)
//                .setColorScheme(new ColorSchemeRedDark())
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(startPoseright)
//                                .forward(48)
//                                //Pun pe stalp preload ul
//                                .lineToLinearHeading(rightBlueJunction)
//
//                                //Start cycle
//                                //1)
//                                .lineToLinearHeading(rightBlueStack)
//
//                                .lineToLinearHeading(rightBlueJunction)
//                                //2)
//                                .lineToLinearHeading(rightBlueStack)
//                                .lineToLinearHeading(rightBlueJunction)
//                                //3)
//                                .lineToLinearHeading(rightBlueStack)
//                                .lineToLinearHeading(rightBlueJunction)
//                                //4)
//                                .lineToLinearHeading(rightBlueStack)
//                                .lineToLinearHeading(rightBlueJunction)
//                                //5)
//                                .lineToLinearHeading(leftBlueStack)
//                                .lineToLinearHeading(leftBlueJunction)
//                                //Park
//                                .strafeLeft(12)
//                                .build()
//                );
//
//        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
//                .setDarkMode(true)
//                .setBackgroundAlpha(1)
//                .addEntity(myBotLeft)
//                .start();
//    }
//}