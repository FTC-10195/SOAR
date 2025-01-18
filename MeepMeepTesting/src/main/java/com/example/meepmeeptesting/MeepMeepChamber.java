package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepChamber {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(360), Math.toRadians(360), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8, -63.75, Math.toRadians(90)))
                //Deploy (maybe <= .5)
                .strafeToLinearHeading(new Vector2d(8, -34), Math.toRadians(90))
                .waitSeconds(.5)
                //Pick up piece and launch it backwards
                .strafeToLinearHeading(new Vector2d(48, -46), Math.toRadians(90))
                .waitSeconds(.5)
                //Pick up piece and launch it backwards
                .strafeToLinearHeading(new Vector2d(58, -46), Math.toRadians(90))
                .waitSeconds(.5)
                //Pick up piece
                .strafeToLinearHeading(new Vector2d(54, -46), Math.toRadians(60))
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(48, -46), Math.toRadians(270))
                //Shoot forward & pick up specimen #2
                .strafeToLinearHeading(new Vector2d(48, -52), Math.toRadians(270))
                //Deploy
                .strafeToLinearHeading(new Vector2d(4, -34), Math.toRadians(90))
                .waitSeconds(.5)
                //Pick up specimen #3
                .strafeToLinearHeading(new Vector2d(58, -46), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(58, -52), Math.toRadians(270))
                //Deploy
                .strafeToLinearHeading(new Vector2d(0, -34), Math.toRadians(90))
                //Pick up specimen #4 (this one will be placed same as specimen #2
                .strafeToLinearHeading(new Vector2d(48, -46), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(48, -52), Math.toRadians(270))
                //Deploy
                .strafeToLinearHeading(new Vector2d(-4, -34), Math.toRadians(90))
                .waitSeconds(.5)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
