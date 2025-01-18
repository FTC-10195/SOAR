package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBucket {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(360), Math.toRadians(360), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-24, -63.75, Math.toRadians(180)))
                //Deploy
                .strafeToLinearHeading(new Vector2d(-56, -56), Math.toRadians(225))
                //Intake until grabbed (<= .5 seconds)
                .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(90))
                        .waitSeconds(.5)
                //Deploy
                .strafeToLinearHeading(new Vector2d(-56, -56), Math.toRadians(225))
                //Intake until grabbed (<= .5 seconds)
                        .strafeToLinearHeading(new Vector2d(-58, -48), Math.toRadians(90))
                        .waitSeconds(.5)
                //Deploy
                .strafeToLinearHeading(new Vector2d(-56, -56), Math.toRadians(225))
                //Intake until grabbed (<= .5 seconds)
                .strafeToLinearHeading(new Vector2d(-54, -41), Math.toRadians(135))
                        .waitSeconds(.5)
                //Deploy
                .strafeToLinearHeading(new Vector2d(-56, -56), Math.toRadians(225))
                //Go to submersible
                        .strafeToLinearHeading(new Vector2d(-54, -10), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-24, -10), Math.toRadians(0))
                //Eventually pick up a piece
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
