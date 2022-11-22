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


public class MidAuto {

    public static void main(String args[]){

        MeepMeep mm = new MeepMeep(950);

        Pose2d startPoseLeft = new Pose2d(-36, -60, Math.toRadians(90));
        Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));

        Pose2d leftBlueJunction = new Pose2d(-24, -12, Math.toRadians(90));
        Pose2d leftBlueMidPos = new Pose2d(-34, -12, Math.toRadians(180));
        Pose2d leftBlueStack = new Pose2d(-55, -13, Math.toRadians(175));

        RoadRunnerBotEntity myBotLeft = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 55, 3, 3, 13.5)
                .setDimensions(16, 17)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseLeft)
                                .forward(48)
                                //Pun pe stalp preload ul
                                .lineToLinearHeading(leftBlueJunction)
                                .lineToLinearHeading(leftBlueMidPos)


                                //Start cycle
                                //1)
                                .lineToLinearHeading(leftBlueStack)
                                .lineToLinearHeading(leftBlueMidPos)

                                .lineToLinearHeading(leftBlueJunction)
                                .lineToLinearHeading(leftBlueMidPos)
                                //2)
                                .lineToLinearHeading(leftBlueStack)
                                .lineToLinearHeading(leftBlueMidPos)
                                .lineToLinearHeading(leftBlueJunction)
                                .lineToLinearHeading(leftBlueMidPos)
                                //3)
                                .lineToLinearHeading(leftBlueStack)
                                .lineToLinearHeading(leftBlueMidPos)
                                .lineToLinearHeading(leftBlueJunction)
                                .lineToLinearHeading(leftBlueMidPos)
                                //4)
                                .lineToLinearHeading(leftBlueStack)
                                .lineToLinearHeading(leftBlueMidPos)
                                .lineToLinearHeading(leftBlueJunction)
                                .lineToLinearHeading(leftBlueMidPos)
                                //5)
                                .lineToLinearHeading(leftBlueStack)
                                .lineToLinearHeading(leftBlueMidPos)
                                .lineToLinearHeading(leftBlueJunction)
                                .lineToLinearHeading(leftBlueMidPos)
                                //Park
                                .build()
                );

        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1)
                .addEntity(myBotLeft)
                .start();
    }
}