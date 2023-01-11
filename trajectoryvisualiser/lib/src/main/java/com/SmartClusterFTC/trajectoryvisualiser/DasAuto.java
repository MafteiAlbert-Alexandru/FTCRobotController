package com.SmartClusterFTC.trajectoryvisualiser;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class DasAuto {

    public static void main(String[] args){

        MeepMeep mm = new MeepMeep(950);

        Pose2d startPoseLeft = new Pose2d(-36, -60, Math.toRadians(90));
//        Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));

        Vector2d leftBlueTransfer = new Vector2d(-8, -12);

//        Pose2d leftBlueCenter = new Pose2d(-36, -12, Math.toRadians(180));

        Vector2d leftBlueStack = new Vector2d(-55, -12);

        Pose2d leftBlueJunction = new Pose2d(-14, -8, Math.toRadians(135));

        Pose2d leftBlueTranferBack = new Pose2d(-8, -12, Math.toRadians(180));


        Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));

        Vector2d rightBlueTransfer = new Vector2d(8, -12);


        Vector2d rightBlueStack = new Vector2d(55, -12);

        Pose2d rightBlueJunction = new Pose2d(14, -8, Math.toRadians(180-135));

        Pose2d rightBlueTranferBack = new Pose2d(8, -12, Math.toRadians(0));


        RoadRunnerBotEntity myBotLeft = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 13.5)
                .setDimensions(17.5, 17.5)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseLeft)
                                .forward(48)
                                .turn(Math.toRadians(-90))
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
                                .build()
                );


        RoadRunnerBotEntity myBotRight = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 13.5)
                .setDimensions(17.5, 17.5)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseRight)
                                .forward(48)
                                .turn(Math.toRadians(90))
                                //Pun pe stalp preload ul
                                .lineToConstantHeading(rightBlueTransfer)
                                .lineToLinearHeading(rightBlueJunction)
                                .lineToLinearHeading(rightBlueTranferBack)

                                .lineToConstantHeading(rightBlueStack)

                                .lineToConstantHeading(rightBlueTransfer)
                                .lineToLinearHeading(rightBlueJunction)
                                .lineToLinearHeading(rightBlueTranferBack)
                                .lineToConstantHeading(rightBlueStack)

                                .lineToConstantHeading(rightBlueTransfer)
                                .lineToLinearHeading(rightBlueJunction)
                                .lineToLinearHeading(rightBlueTranferBack)
                                .lineToConstantHeading(rightBlueStack)

                                .lineToConstantHeading(rightBlueTransfer)
                                .lineToLinearHeading(rightBlueJunction)
                                .lineToLinearHeading(rightBlueTranferBack)
                                .lineToConstantHeading(rightBlueStack)

                                .lineToConstantHeading(rightBlueTransfer)
                                .lineToLinearHeading(rightBlueJunction)
                                .lineToLinearHeading(rightBlueTranferBack)
                                .lineToConstantHeading(rightBlueStack)

                                .lineToConstantHeading(rightBlueTransfer)
                                .lineToLinearHeading(rightBlueJunction)
                                .lineToLinearHeading(rightBlueTranferBack)
                                .lineToConstantHeading(rightBlueStack)

                                .build()
                );


        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1)
                .addEntity(myBotLeft)
                .addEntity(myBotRight)
                .start();
    }
}