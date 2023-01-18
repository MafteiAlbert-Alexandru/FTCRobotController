package com.SmartClusterFTC.trajectoryvisualiser;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class LowAuto {

    public static void main(String args[]){

        MeepMeep mm = new MeepMeep(950);

        Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));
//        Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));

        Pose2d rightBlueMid = new Pose2d(22, -13, Math.toRadians(0));

        Pose2d rightBlueJunction = new Pose2d(16, 0, Math.toRadians(0));

        Pose2d rightBlueStack = new Pose2d(55, -12, Math.toRadians(0));

        RoadRunnerBotEntity myBotLeft = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 55, 3, 3, 13.5)
                .setDimensions(17, 17)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseRight)
                                .forward(48)
                                .strafeLeft(12)
                                .strafeRight(12)
                                .turn(Math.toRadians(-90))
                                .lineToLinearHeading(rightBlueStack)
                                .lineToLinearHeading(rightBlueMid)
                                .setReversed(true)
                                .splineToLinearHeading(rightBlueJunction, Math.toRadians(-45))

                                .build()
                );

        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1)
                .addEntity(myBotLeft)
//                .addEntity(myBotLeft2)
                .start();
    }
}