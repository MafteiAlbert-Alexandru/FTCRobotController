package com.SmartClusterFTC.trajectoryvisualiser;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class CycleAngle {

    public static void main(String args[]){

        MeepMeep meepMeep = new MeepMeep(900);

        Pose2d startPoseLeft = new Pose2d(-36, -60, Math.toRadians(90));

        Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));

        Pose2d blueRightPrePark = new Pose2d(36, -37, Math.toRadians(90));

        Pose2d blueRightJunctionPose = new Pose2d(32, -11, Math.toRadians(100));

        Pose2d blueRightSignalPushPose = new Pose2d(36, -10, Math.toRadians(90));

        Pose2d blueRightStoragePose = new Pose2d(60, -12, Math.toRadians(180));

        Vector2d blueRightJunction = new Vector2d(28, -11);

        Vector2d blueRightStorage = new Vector2d(55, -12);

        Pose2d transitionPoseRight = new Pose2d(36, -12, Math.toRadians(90));

        Vector2d redRightJunction = new Vector2d(-36, -11);

        Vector2d redRightStorage = new Vector2d(-55, -12);

        Pose2d blueLeftTransition = new Pose2d(-40, -15, Math.toRadians(180));

        RoadRunnerBotEntity myBotLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseLeft)
                                .forward(52)
                                .back(4)
//                                .strafeRight(24)
//                                .forward(48)
                                .lineToLinearHeading(new Pose2d(-36, -13, Math.toRadians(45)))

                                //Pun pe stalp preload ul
                                .lineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(45)))
                                .lineToLinearHeading(blueLeftTransition)
                                //Start cycle
                                //1)
//                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                                .lineToLinearHeading(blueLeftTransition)
                                .lineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(45)))
                                .lineToLinearHeading(blueLeftTransition)
                                //2)
                                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                                .lineToLinearHeading(blueLeftTransition)
                                .lineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(45)))
                                .lineToLinearHeading(blueLeftTransition)
                                //3)
                                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                                .lineToLinearHeading(blueLeftTransition)
                                .lineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(45)))
                                .lineToLinearHeading(blueLeftTransition)
                                //4)
                                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                                .lineToLinearHeading(blueLeftTransition)
                                .lineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(45)))
                                .lineToLinearHeading(blueLeftTransition)
                                //5)
                                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                                .lineToLinearHeading(blueLeftTransition)
                                .lineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(45)))
                                .lineToLinearHeading(blueLeftTransition)
                                .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(90)))

                                //Park
                                .back(24)
                                .build()
                );

        RoadRunnerBotEntity myBotLeft_v3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, 2, 2, 13.3)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startPoseLeft)
                                        .forward(52)
                                        .back(4)
                                        .lineToLinearHeading(new Pose2d(-36, -13, Math.toRadians(45)))

                                        //Pun pe stalp preload ul
                                        .lineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(45)))
                                        .splineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)), Math.toRadians(140))
                                        .splineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(45)), Math.toRadians(-200))

                                        .splineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)), Math.toRadians(140))
                                        .splineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(45)), Math.toRadians(-200))

                                        .splineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)), Math.toRadians(140))
                                        .splineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(45)), Math.toRadians(-200))

                                        .splineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)), Math.toRadians(140))
                                        .splineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(45)), Math.toRadians(-200))

                                        .splineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)), Math.toRadians(140))
                                        .splineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(45)), Math.toRadians(-200))


//                                        .lineToLinearHeading(blueLeftTransition)
//                                        .lineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(45)))
//                                        .lineToLinearHeading(blueLeftTransition)
//                                        //2)
//                                        .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
//                                        .lineToLinearHeading(blueLeftTransition)
//                                        .lineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(45)))
//                                        .lineToLinearHeading(blueLeftTransition)
//                                        //3)
//                                        .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
//                                        .lineToLinearHeading(blueLeftTransition)
//                                        .lineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(45)))
//                                        .lineToLinearHeading(blueLeftTransition)
//                                        //4)
//                                        .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
//                                        .lineToLinearHeading(blueLeftTransition)
//                                        .lineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(45)))
//                                        .lineToLinearHeading(blueLeftTransition)
//                                        //5)
//                                        .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
//                                        .lineToLinearHeading(blueLeftTransition)
//                                        .lineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(45)))
//                                        .lineToLinearHeading(blueLeftTransition)
//                                        .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(90)))
//
//                                        //Park
//                                        .back(24)
                                        .build()
                );

        RoadRunnerBotEntity myBotRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(360), Math.toRadians(360), 13)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startPoseRight)
//
                                        .lineToLinearHeading(blueRightSignalPushPose)
                                        //Pre load)
                                        .lineToLinearHeading(blueRightJunctionPose)
                                        //1)
                                        .lineToLinearHeading(blueRightStoragePose)
                                        .lineToLinearHeading(blueRightJunctionPose)
                                        //2
                                        .lineToLinearHeading(blueRightStoragePose)
                                        .lineToLinearHeading(blueRightJunctionPose)
                                        //3)
                                        .lineToLinearHeading(blueRightStoragePose)
                                        .lineToLinearHeading(blueRightJunctionPose)
                                        //4)
                                        .lineToLinearHeading(blueRightStoragePose)
                                        .lineToLinearHeading(blueRightJunctionPose)
                                        //5)
                                        .lineToLinearHeading(blueRightStoragePose)
                                        .lineToLinearHeading(blueRightJunctionPose)

                                        .lineToLinearHeading(blueRightPrePark)

                                        .build()
                );


        RoadRunnerBotEntity myBotRight_2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(360), Math.toRadians(360), 15)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startPoseRight)
//
//                                        .forward(48)
                                        .lineToLinearHeading(blueRightSignalPushPose)
                                        //Pre load)
                                        .lineToConstantHeading(blueRightJunction)
                                        .turn(Math.toRadians(45))
                                        .turn(Math.toRadians(-45))
                                        //1)
                                        .lineToConstantHeading(blueRightStorage)
                                        .lineToConstantHeading(blueRightJunction)
                                        .turn(Math.toRadians(45))
                                        .turn(Math.toRadians(-45))
                                        //2
                                        .lineToConstantHeading(blueRightStorage)
                                        .lineToConstantHeading(blueRightJunction)
                                        .turn(Math.toRadians(45))
                                        .turn(Math.toRadians(-45))
                                        //3)
                                        .lineToConstantHeading(blueRightStorage)
                                        .lineToConstantHeading(blueRightJunction)
                                        .turn(Math.toRadians(45))
                                        .turn(Math.toRadians(-45))
                                        //4)
                                        .lineToConstantHeading(blueRightStorage)
                                        .lineToConstantHeading(blueRightJunction)
                                        .turn(Math.toRadians(45))
                                        .turn(Math.toRadians(-45))
                                        //5)
                                        .lineToConstantHeading(blueRightStorage)
                                        .lineToConstantHeading(blueRightJunction)
                                        .turn(Math.toRadians(45))
                                        .turn(Math.toRadians(-45))

                                        .lineToConstantHeading(blueRightStorage)
                                        .lineToConstantHeading(blueRightJunction)


                                        .build()
                );

        RoadRunnerBotEntity myBotLeft_2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(360), Math.toRadians(360), 15)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startPoseLeft)
                                        .forward(52)
                                        .back(4)
//                                .strafeRight(24)
//                                .forward(48)
                                        .lineToLinearHeading(new Pose2d(-36, -13, Math.toRadians(45)))

                                        //Pun pe stalp preload ul
                                        .lineToLinearHeading(new Pose2d(-35, -13, Math.toRadians(45)))

                                        //Start cycle
                                        //1)
                                        .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                                        .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(45)))
                                        //2)
                                        .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                                        .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(45)))
                                        //3)
                                        .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                                        .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(45)))
                                        //4)
                                        .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                                        .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(45)))
                                        //5)
                                        .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                                        .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(45)))

                                        .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(90)))

                                        //Park
                                        .back(24)
                                        .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.5f)
//                .addEntity(myBotRight)
                .addEntity(myBotLeft_v3)
//                .addEntity(myBotRight_2)
                .start();
    }
}