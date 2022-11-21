package com.SmartClusterFTC.trajectoryvisualiser;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class CycleCenter {

    public static void main(String[] args){

        MeepMeep mm = new MeepMeep(950);

        Pose2d BlueStartPoseLeft = new Pose2d(-36, -60, Math.toRadians(90));

        Pose2d blueLeftCenterJunction = new Pose2d(-12, -11, Math.toRadians(135));

        Pose2d blueLeftStoragePose = new Pose2d(-55, -12, Math.toRadians(180));

        Pose2d blueLeftTransitionPoint = new Pose2d(-36, -12, Math.toRadians(90));

        Pose2d blueLeftParkMid = new Pose2d(-36, -36, Math.toRadians(90));

        Pose2d BlueStartPoseRight = new Pose2d(36, -60, Math.toRadians(90));

        Pose2d blueRightCenterJunction = new Pose2d(24, -12, Math.toRadians(90));

        Pose2d blueRightStoragePose = new Pose2d(55, -12, Math.toRadians(0));

        Pose2d blueRightTransitionPoint = new Pose2d(36, -12, Math.toRadians(90));

        Pose2d blueRightParkMid = new Pose2d(36, -36, Math.toRadians(90));

        Pose2d blueLeftParkRight = new Pose2d(-12, -36, Math.toRadians(90));

        Pose2d blueLeftParkLeft = new Pose2d(-60, -36, Math.toRadians(90));

        Pose2d tiganPose = new Pose2d(-12, 38, Math.toRadians(45));

        RoadRunnerBotEntity myBotLeft = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(360), Math.toRadians(360), 13)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(BlueStartPoseLeft)
                                .lineToLinearHeading(blueLeftTransitionPoint)

                                //Preload
                                .lineToLinearHeading(blueLeftCenterJunction)

                                //Start sycle
                                //1)
                                .lineToLinearHeading(blueLeftStoragePose)
                                .lineToLinearHeading(blueLeftCenterJunction)
                                //2)
                                .lineToLinearHeading(blueLeftStoragePose)
                                .lineToLinearHeading(blueLeftCenterJunction)
                                //3)
                                .lineToLinearHeading(blueLeftStoragePose)
                                .lineToLinearHeading(blueLeftCenterJunction)
                                //4)
                                .lineToLinearHeading(blueLeftStoragePose)
                                .lineToLinearHeading(blueLeftCenterJunction)
                                //5)
                                .lineToLinearHeading(blueLeftStoragePose)
                                .back(42)
                                .lineToLinearHeading(tiganPose)
                                //Park

                                .build()
                );

        RoadRunnerBotEntity myBotRight = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 13)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeBlueLight())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(BlueStartPoseRight)
                                .lineToLinearHeading(blueRightTransitionPoint)

                                //Preload
                                .lineToLinearHeading(blueRightCenterJunction)

                                //Start sycle
                                //1)
                                .strafeRight(36)
                                .strafeLeft(36)
                                //2)
                                .strafeRight(36)
                                .strafeLeft(36)
                                //3)
                                .strafeRight(36)
                                .strafeLeft(36)
                                //4)
                                .strafeRight(36)
                                .strafeLeft(36)
                                //5)
                                .strafeRight(36)
                                .strafeLeft(36)

                                //Park

                                .build()
                );


        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBotLeft)
                .addEntity(myBotRight)
                .start();
    }
}