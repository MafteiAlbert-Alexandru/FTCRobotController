package com.SmartClusterFTC.trajectoryvisualiser;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class DasAuto {

    public static void main(String args[]){

        MeepMeep mm = new MeepMeep(950);

        Pose2d startPoseLeft = new Pose2d(-36, -60, Math.toRadians(90));
//        Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));

        Vector2d leftBlueTransfer = new Vector2d(-8, -12);

//        Pose2d leftBlueCenter = new Pose2d(-36, -12, Math.toRadians(180));

        Vector2d leftBlueStack = new Vector2d(-55, -12);

        Pose2d leftBlueJunction = new Pose2d(-14, -8, Math.toRadians(135));

        Pose2d leftBlueTranferBack = new Pose2d(-8, -12, Math.toRadians(180));

//
//        Vector2d vector2d = new Vector2d(-12, 0);
//
//        Pose2d leftBlueTransfer = new Pose2d(-48, -12, Math.toRadians(180));

        RoadRunnerBotEntity myBotLeft = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 55, 3, 3, 13.5)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseLeft)
                                .forward(48)
                                .turn(Math.toRadians(90))
                                //Pun pe stalp preload ul
                                .lineToConstantHeading(leftBlueTransfer)
                                .lineToLinearHeading(leftBlueJunction)
                                .lineToLinearHeading(leftBlueTranferBack)

                                .lineToConstantHeading(leftBlueStack)

                                .lineToConstantHeading(leftBlueTransfer)
                                .lineToLinearHeading(leftBlueJunction)
                                .lineToLinearHeading(leftBlueTranferBack)
                                .lineToConstantHeading(leftBlueStack)

                                .lineToConstantHeading(leftBlueTransfer)
                                .lineToLinearHeading(leftBlueJunction)
                                .lineToLinearHeading(leftBlueTranferBack)
                                .lineToConstantHeading(leftBlueStack)

                                .lineToConstantHeading(leftBlueTransfer)
                                .lineToLinearHeading(leftBlueJunction)
                                .lineToLinearHeading(leftBlueTranferBack)
                                .lineToConstantHeading(leftBlueStack)

                                .lineToConstantHeading(leftBlueTransfer)
                                .lineToLinearHeading(leftBlueJunction)
                                .lineToLinearHeading(leftBlueTranferBack)
                                .lineToConstantHeading(leftBlueStack)

                                .lineToConstantHeading(leftBlueTransfer)
                                .lineToLinearHeading(leftBlueJunction)
                                .lineToLinearHeading(leftBlueTranferBack)
                                .lineToConstantHeading(leftBlueStack)



//                                .lineToLinearHeading(leftBlueCenter)
//
//                                .lineToLinearHeading(leftBlueStack)
//
//                                .lineToLinearHeading(leftBlueTransfer)
//                                .lineToLinearHeading(leftBlueJunction)
//
//                                //Start cycle
//                                //1)
//                                .lineToLinearHeading(leftBlueStack)
//
//                                .lineToLinearHeading(leftBlueJunction)
//                                //2)
//                                .lineToLinearHeading(leftBlueStack)
//                                .lineToLinearHeading(leftBlueJunction)
//                                //3)
//                                .lineToLinearHeading(leftBlueStack)
//                                .lineToLinearHeading(leftBlueJunction)
//                                //4)
//                                .lineToLinearHeading(leftBlueStack)
//                                .lineToLinearHeading(leftBlueJunction)
//                                //5)
//                                .lineToLinearHeading(leftBlueStack)
//                                .lineToLinearHeading(leftBlueJunction)
//                                //Park
//                                .strafeLeft(12)
                                .build()
                );

        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1)
                .addEntity(myBotLeft)
                .start();
    }
}