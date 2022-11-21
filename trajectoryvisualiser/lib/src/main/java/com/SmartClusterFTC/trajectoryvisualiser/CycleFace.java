package com.SmartClusterFTC.trajectoryvisualiser;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import javax.imageio.ImageTranscoder;

public class CycleFace {

    public static void main(String args[]){

        MeepMeep mm = new MeepMeep(950, 144);

        Pose2d startPoseLeft = new Pose2d(-36, -60, Math.toRadians(90));

        Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));

        RoadRunnerBotEntity myBotLeft = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 13.5)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseLeft)
                                .forward(48)
//                                .back(4)
                                //.strafeRight(24)
                                //.forward(48)
                                //.lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(45)))


                                //Pun pe stalp preload ul
                                .lineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(90)))

                                //Start cycle
                                //1)
                                .lineToLinearHeading(new Pose2d(-55, -13, Math.toRadians(175)))

                                .lineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(90)))
                                //2)
                                .lineToLinearHeading(new Pose2d(-55, -13, Math.toRadians(175)))
                                .lineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(90)))
                                //3)
                                .lineToLinearHeading(new Pose2d(-55, -13, Math.toRadians(175)))
                                .lineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(90)))
                                //4)
                                .lineToLinearHeading(new Pose2d(-55, -13, Math.toRadians(175)))
                                .lineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(90)))
                                //5)
                                .lineToLinearHeading(new Pose2d(-55, -13, Math.toRadians(175)))
                                .lineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(90)))
                                //Park
//                                .back(24)
                                .build()
                );

        RoadRunnerBotEntity myBotRight = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 13.5)
                .setDimensions(15, 17)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseRight)
                                .forward(48)
//                                .back(4)
                                //.strafeRight(24)
                                //.forward(48)
                                //.lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(45)))

                                //Pun pe stalp preload ul
                                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(45)))

                                //Start cycle
                                //1)
                                .lineToLinearHeading(new Pose2d(55, -12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(45)))
                                //2)
                                .lineToLinearHeading(new Pose2d(55, -12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(45)))
                                //3)
                                .lineToLinearHeading(new Pose2d(55, -12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(45)))
                                //4)
                                .lineToLinearHeading(new Pose2d(55, -12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(45)))
                                //5)
                                .lineToLinearHeading(new Pose2d(55, -12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(45)))
                                //Park
                                .build()
                );

//        RoadRunnerBotEntity path = new DefaultBotBuilder(mm)
//                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 13.3)
//                .setDimensions(15, 17)
//                .setColorScheme(new ColorSchemeBlueLight())
//                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-21.93, -4.89, Math.toRadians(-30.26)))
//                        .splineTo(new Vector2d(-60.30, -12.59), Math.toRadians(180.00))
//                        .turn(Math.toRadians(180))
//                        .splineTo(new Vector2d(-38.22, -8.44), Math.toRadians(23.30))
//                        .build()
//                );

        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.5f)
                .addEntity(myBotRight)
                .addEntity(myBotLeft)
//                .addEntity(path)
                .start();
    }
}