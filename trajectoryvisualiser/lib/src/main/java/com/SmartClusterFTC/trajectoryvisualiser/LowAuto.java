package com.SmartClusterFTC.trajectoryvisualiser;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

        //smash auto
        RoadRunnerBotEntity myBotLeft = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, 1, 1, 13.5)
                .setDimensions(17, 17)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-32.98, -63.41, Math.toRadians(180.00)))
                                .splineToConstantHeading(new Vector2d(-34.87, -59.92), Math.toRadians(90.00))
                                .splineToConstantHeading(new Vector2d(-34.29, -17.11), Math.toRadians(90.00))
                                .splineToConstantHeading(new Vector2d(-54.24, -12.45), Math.toRadians(7.77))
                                .waitSeconds(0.1)
                                .splineToConstantHeading(new Vector2d(-13.76, -11.87), Math.toRadians(4.76))
                                .splineToConstantHeading(new Vector2d(-11.28, -0.95), Math.toRadians(89.62))
                                .build()
                )
//                .followTrajectorySequence(drive)
                ;

        RoadRunnerBotEntity secondBot = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, 1, 1, 13.5)
                .setDimensions(17, 17)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-12.74, -0.66, Math.toRadians(-75.74)))
                                .splineToConstantHeading(new Vector2d(-55.11, -12.45), Math.toRadians(182.86))
                                .build()
                )
//                .followTrajectorySequence(drive)
                ;


        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1)
                .addEntity(secondBot)
//                .addEntity(myBotLeft2)
                .start();
    }
}