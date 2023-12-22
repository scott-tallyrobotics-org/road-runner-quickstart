package com.example.lib; // I didn't change the name so the directory is different

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity RedBot1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 7)
                .setDimensions(12.25,12.75)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->

// front initial position
                        drive.trajectorySequenceBuilder(new Pose2d(-40, -64, Math.toRadians(90)))

// back initial position
//                        drive.trajectorySequenceBuilder(new Pose2d(17, -64, Math.toRadians(90)))



// initial setup
                                .forward(38)

///////////////////////////////////////////////////////////////

// center (purple)
//                                .waitSeconds(1)

// left (purple)
                                // back
//                                .strafeLeft(9)
//                                .waitSeconds(1)
//                                .strafeRight(9)

                                // front
//                                .strafeLeft(13)
//                                .waitSeconds(1)

// right (purple)
                                // back
//                                .strafeRight(13)
//                                .waitSeconds(1)

                                // front
                                .strafeRight(9)
                                .waitSeconds(1)
                                .strafeLeft(9)

//////////////////////////////////////////////////////////////////////

// front setup to yellow
                                    .back(33)
                                    .lineTo(new Vector2d(20, -59))

// back setup to yellow
                                // none

///////////////////////////////////////////////////////////////////

// center (yellow)
//                                .lineToLinearHeading(new Pose2d(50,-35, 0))

// left (yellow)
//                                .lineToLinearHeading(new Pose2d(50,-29, 0))

// right (yellow)
                                .lineToLinearHeading(new Pose2d(50,-41, 0))

// servo (yellow)
                                .waitSeconds(1)

// arm (yellow)
//                                .waitSeconds(3)

///////////////////////////////////////////////////////////////////

//// far (park)
                                .lineTo(new Vector2d(50, -14))
                                .lineTo(new Vector2d(62, -14))

// near (park)
//                                .lineTo(new Vector2d(50, -62))
//                                .lineTo(new Vector2d(62, -62))

                                .build()

                );

        RoadRunnerBotEntity RedBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 7)
                .setDimensions(12.25,12.75)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->

// front initial position
//                            drive.trajectorySequenceBuilder(new Pose2d(-40, -64, Math.toRadians(90)))

// back initial position
                            drive.trajectorySequenceBuilder(new Pose2d(17, -64, Math.toRadians(90)))



// initial setup
                                    .forward(38)

///////////////////////////////////////////////////////////////

// center (purple)
//                                    .waitSeconds(1)

// left (purple)
                                        // back
//                                    .strafeLeft(9)
//                                    .waitSeconds(1)
//                                    .strafeRight(9)

                                        // front
//                                    .strafeLeft(13)
//                                    .waitSeconds(1)

// back right (purple)
                                        // back
                                    .strafeRight(13)
                                    .waitSeconds(1)

                                        // front
//                                    .strafeRight(9)
//                                    .waitSeconds(1)
//                                    .strafeLeft(9)

//////////////////////////////////////////////////////////////////////

// front setup to yellow
//                                    .back(33)
//                                    .lineTo(new Vector2d(20, -59))
// back setup to yellow
                                        // none

///////////////////////////////////////////////////////////////////

// center (yellow)
//                                    .lineToLinearHeading(new Pose2d(50,-35, 0))

// left (yellow)
//                                    .lineToLinearHeading(new Pose2d(50,-29, 0))

// right (yellow)
                                    .lineToLinearHeading(new Pose2d(50,-41, 0))

// servo (yellow)
                                    .waitSeconds(1)

// arm (yellow)
//                                    .waitSeconds(3)

///////////////////////////////////////////////////////////////////

//// far (park)
//                                    .lineTo(new Vector2d(50, -14))
//                                    .lineTo(new Vector2d(62, -14))

// near (park)
                                    .lineTo(new Vector2d(50, -62))
                                    .lineTo(new Vector2d(62, -62))

                                    .build()

                );













        RoadRunnerBotEntity BlueBot1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 7)
                .setDimensions(12.25,12.75)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->

// front initial position
                        drive.trajectorySequenceBuilder(new Pose2d(-40, 64, Math.toRadians(270)))

// back initial position
//                        drive.trajectorySequenceBuilder(new Pose2d(17, 64, Math.toRadians(270)))



// initial setup
                        .forward(38)

///////////////////////////////////////////////////////////////

// center (purple)
                        // none
//                        .waitSeconds(1)

// left (purple)
                        // back
//                        .strafeLeft(13)
//                        .waitSeconds(1)

                        // front
//                        .strafeLeft(9)
//                        .waitSeconds(1)
//                        .strafeRight(9)

// right (purple)
                        // back
//                        .strafeRight(9)
//                        .waitSeconds(1)
//                        .strafeLeft(9)
//
                        // front
                        .strafeRight(13)
                        .waitSeconds(1)

//////////////////////////////////////////////////////////////////////

// front setup to yellow
                        .back(33)
                        .lineTo(new Vector2d(20, 59))

// back setup to yellow
                        // none

///////////////////////////////////////////////////////////////////

// center (yellow)
//                        .lineToLinearHeading(new Pose2d(50,35, 0))

// left (yellow)
//                        .lineToLinearHeading(new Pose2d(50,41, 0))

// right (yellow)
                        .lineToLinearHeading(new Pose2d(50,29, 0))

// servo (yellow)
                        .waitSeconds(1)

// arm (yellow)
//                        .waitSeconds(3)

///////////////////////////////////////////////////////////////////

// far (park)
//                        .lineTo(new Vector2d(50, 14))
//                        .lineTo(new Vector2d(62, 14))

// near (park)
                        .lineTo(new Vector2d(50, 62))
                        .lineTo(new Vector2d(62, 62))

                        .build()

                );

        RoadRunnerBotEntity BlueBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 7)
                .setDimensions(12.25,12.75)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->

// front initial position
//                        drive.trajectorySequenceBuilder(new Pose2d(-40, 64, Math.toRadians(270)))

// back initial position
                        drive.trajectorySequenceBuilder(new Pose2d(17, 64, Math.toRadians(270)))



// initial setup
                                .forward(38)

///////////////////////////////////////////////////////////////

// center (purple)
                                        // none
//                                .waitSeconds(1)

// left (purple)
                                        // back
//                                .strafeLeft(13)
//                                .waitSeconds(1)

                                        // front
//                                .strafeLeft(9)
//                                .waitSeconds(1)
//                                .strafeRight(9)

// right (purple)
                                        // back
                                .strafeRight(9)
                                .waitSeconds(1)
                                .strafeLeft(9)

                                        // front
//                                .strafeRight(13)
//                                .waitSeconds(1)

//////////////////////////////////////////////////////////////////////

// front setup to yellow
//                                .back(33)
//                                .lineTo(new Vector2d(20, 59))

// back setup to yellow
                                        // none

///////////////////////////////////////////////////////////////////

// center (yellow)
//                                .lineToLinearHeading(new Pose2d(50,35, 0))

// left (yellow)
//                                .lineToLinearHeading(new Pose2d(50,41, 0))

// right (yellow)
                                .lineToLinearHeading(new Pose2d(50,29, 0))

// servo (yellow)
                                .waitSeconds(1)

// arm (yellow)
//                                .waitSeconds(3)

///////////////////////////////////////////////////////////////////

// far (park)
                                .lineTo(new Vector2d(50, 14))
                                .lineTo(new Vector2d(62, 14))

// near (park)
//                                .lineTo(new Vector2d(50, 62))
//                                .lineTo(new Vector2d(62, 62))

                                .build()

                );

    meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.98f)

    // Add both of our declared bot entities
        .addEntity(RedBot1)
        .addEntity(RedBot2)
        .addEntity(BlueBot1)
        .addEntity(BlueBot2)
        .start();
    }
}