package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        double ROBOT_LENGTH = 17.1;
        double ROBOT_WIDTH = 16.38;
        double TILE_SIZE = 24;
        double BARRIER_OFFSET = 0; //Probably not necessary
        double EXTENDO_REACH = 2; //Used as an offset
        Pose2d beginPose = new Pose2d((ROBOT_WIDTH/2),-72+(ROBOT_LENGTH/2) , Math.toRadians(90)); //Robot starts facing forwards towards the submersible
        //Pre-score
        Vector2d preScoreVec = new Vector2d(beginPose.position.x,beginPose.position.y+(TILE_SIZE - ROBOT_LENGTH/2)); //In between row 1 and row 2
        double preScoreRot = 90;
        Pose2d preScorePos = new Pose2d(preScoreVec, Math.toRadians(preScoreRot));
        Vector2d scoreVec = new Vector2d(preScoreVec.x, preScoreVec.y + 11); //drives forward enough to score
        double scoreRot = preScoreRot;
        Pose2d scorePos = new Pose2d(scoreVec, Math.toRadians(scoreRot));
        Vector2d prePushVec1 = new Vector2d(preScoreVec.x + TILE_SIZE + 4, preScoreVec.y); //drives left enough to avoid hitting the submersible
        double prePushRot1 =0;
        Pose2d prePushPos1 = new Pose2d(prePushVec1, Math.toRadians(prePushRot1));
        Vector2d prePushVec2 = new Vector2d(prePushVec1.x, prePushVec1.y+ (TILE_SIZE + (TILE_SIZE/2))); //drives forward enough, ex 36 inches, to push the samples
        double prePushRot2 = 0;
        Pose2d prePushPos2 = new Pose2d(prePushVec2, Math.toRadians(prePushRot2));
        Vector2d prePushVec3 = new Vector2d(prePushVec2.x + (TILE_SIZE/2) - 3, prePushVec2.y); //drives above the first sample
        double prePushRot3 = 0;
        Pose2d prePushPos3 = new Pose2d(prePushVec3, Math.toRadians(prePushRot3));
        Vector2d pushVec1 = new Vector2d(prePushVec3.x , prePushVec3.y - 38); //drives above the first sample
        double pushRot1 = 0;
        Pose2d pushPos1 = new Pose2d(pushVec1, Math.toRadians(pushRot1));

        Vector2d prePushVec4 = new Vector2d(prePushVec2.x + (TILE_SIZE/2) + 7, prePushVec2.y); //drives above the first sample
        double prePushRot4 = 0;
        Pose2d prePushPos4 = new Pose2d(prePushVec4, Math.toRadians(prePushRot4));

        Vector2d pushVec2 = new Vector2d(prePushVec4.x , prePushVec4.y - 38); //drives above the first sample
        double pushRot2 = 0;
        Pose2d pushPos2 = new Pose2d(pushVec2, Math.toRadians(pushRot2));

        Vector2d prePushVec5 = new Vector2d(prePushVec2.x + (TILE_SIZE/2) + 12, prePushVec4.y); //drives above the first sample
        double prePushRot5 = 0;
        Pose2d prePushPos5 = new Pose2d(prePushVec5, Math.toRadians(prePushRot5));

        Vector2d pushVec3 = new Vector2d(prePushVec5.x , prePushVec5.y - 38); //drives above the first sample
        double pushRot3 = 0;
        Pose2d pushPos3 = new Pose2d(pushVec2, Math.toRadians(pushRot2));

    Vector2d intakeVec1 = new Vector2d(preScoreVec.x + (TILE_SIZE*2) - (ROBOT_WIDTH/2) , preScoreVec.y); //drives above the first sample
        double intakeRot1 = 90;
        Pose2d intakePos1 = new Pose2d(intakeVec1, Math.toRadians(intakeRot1));

        Vector2d intakeVec2 = new Vector2d(preScoreVec.x + (TILE_SIZE*2) - (ROBOT_WIDTH/2) , preScoreVec.y - 10); //drives above the first sample
        double intakeRot2 = 90;
        Pose2d intakePos2 = new Pose2d(intakeVec2, Math.toRadians(intakeRot2));

        Vector2d parkVec = new Vector2d(preScoreVec.x + (TILE_SIZE*2) - (ROBOT_WIDTH/2) , preScoreVec.y-10); //drives above the first sample
        double parkRot = 90;
        Pose2d parkPos = new Pose2d(parkVec, Math.toRadians(parkRot));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(beginPose)
                .setTangent(Math.PI/2)
                .splineToConstantHeading(preScoreVec,Math.toRadians(preScoreRot))
                .splineToConstantHeading(scoreVec,Math.toRadians(scoreRot))
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(prePushPos1,Math.toRadians(prePushRot1))
                        .setTangent(Math.PI/2)
                .splineToConstantHeading(prePushVec2,Math.PI/2)
                .splineToConstantHeading(prePushVec3,Math.toRadians(prePushRot3))
                .setTangent(-Math.PI/2)
                .splineToConstantHeading(pushVec1,-Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToConstantHeading(prePushVec3,Math.PI/2)
                .splineToConstantHeading(prePushVec4,Math.toRadians(prePushRot4))
                .setTangent(-Math.PI/2)
                .splineToConstantHeading(pushVec2,-Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToConstantHeading(prePushVec4,Math.PI/2)
                .splineToConstantHeading(prePushVec5,Math.toRadians(prePushRot5))
                .setTangent(-Math.PI/2)
                .splineToConstantHeading(pushVec3,-Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(intakePos1,Math.toRadians(intakeRot1))
                .setTangent(-Math.PI/2)
                .splineToConstantHeading(intakeVec2,Math.toRadians(intakeRot2))
                .setTangent(Math.PI/2)
                .splineToConstantHeading(scoreVec,Math.toRadians(scoreRot))
                //back to intake
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(intakePos1,Math.toRadians(intakeRot1))
                .setTangent(-Math.PI/2)
                .splineToConstantHeading(intakeVec2,Math.toRadians(intakeRot2))
                //Go score
                .setTangent(Math.PI/2)
                .splineToConstantHeading(scoreVec,Math.toRadians(scoreRot))
                //back to intake
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(intakePos1,Math.toRadians(intakeRot1))
                .setTangent(-Math.PI/2)
                .splineToConstantHeading(intakeVec2,Math.toRadians(intakeRot2))
                //Go score
                .setTangent(Math.PI/2)
                .splineToConstantHeading(scoreVec,Math.toRadians(scoreRot))
                //Go park (idk if enough time in reality)
                .setTangent(0)
                .strafeToLinearHeading(parkVec,Math.toRadians(parkRot))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
