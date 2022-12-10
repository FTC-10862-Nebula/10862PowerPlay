package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main() {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55,
                        55,
                        OwnMath.toRadians(300.704976),
                        OwnMath.toRadians(300.704976),
                        14.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                .forward(20)
                                .turn(3)
                                .turn(OwnMath.toRadians(60))
                                .forward(10)
                                .turn(OwnMath.toRadians(235))
                                .forward(35)
                                //Slow forward
                                .forward(5)
                                .forward(-7)
                                .turn(OwnMath.toRadians(210))
                                .forward(6)
                                .turn(OwnMath.toRadians(210))
                                .forward(31)

                                //Wait time

                                .forward(-2)
                                .turn(OwnMath.toRadians(90))
                                .forward(-32)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}