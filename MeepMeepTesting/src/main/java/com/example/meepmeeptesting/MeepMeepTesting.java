package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.Map;

public class MeepMeepTesting {


    public static void main(String[] args) {

        Map stateMap;

        Vector2d endParking;


        // Locations - For Red /////////////////////////////////////////////////////////////////////
        Pose2d startPosition = new Pose2d(-34, -64, Math.toRadians(90));
        Vector2d depositPreloadonLow = new Vector2d(-34, -48);
        Vector2d collectionApporach = new Vector2d(-36, -20);
        Pose2d highPoleDepositingPosition = new Pose2d(-23.25, -11.75, Math.toRadians(0));
        Pose2d highPoleDepositingPosition2 = new Pose2d(-24.3, -11.5, Math.toRadians(0));
        Pose2d lowPoleDepositingPosition = new Pose2d(-47.5, -11.5, Math.toRadians(0));
        Pose2d secondHighPoleDepositingPosition = new Pose2d(0, -11.5, Math.toRadians(0));
        Pose2d highPoleDepositingIntermediatePoint = new Pose2d(highPoleDepositingPosition.getX() - 2, highPoleDepositingPosition.getY(), highPoleDepositingPosition.getHeading());
        Pose2d collectConesPosition = new Pose2d(-56, -11.75, Math.toRadians(0));
        Pose2d depositOnHighPole1approach = new Pose2d(-35, -12, Math.toRadians(0));
        Pose2d depositOnHighPole1 = new Pose2d(-24, -12, Math.toRadians(0));
        Pose2d depositOnHighPole2 = new Pose2d(-25, -12, Math.toRadians(0));
        Pose2d depositPreloadForward = new Pose2d(-62, -36, Math.toRadians(100));
        Vector2d approachVector = new Vector2d(-60, -24);
        double approachHeading = Math.toRadians(90);
        Vector2d depositPreLoadForwardVector = new Vector2d(-57, -13.5);
        double depositPreLoadForwardHeading = Math.toRadians(100);
        String turretPickupPosition;
        String turretDeliveryPosition;

        int initialTurn = -90;

        Vector2d parking3 = new Vector2d(-12, -12.5);
        Vector2d parking2 = new Vector2d(-36, -12.5);
        Vector2d parking1 = new Vector2d(-60, -12.5);

        int initialTangent = -80;
        int collectConeTangent = 90;
        int initialApproachTangent = 90;
        int highPoleDepositingPositionTangent = 90;
        int depositPreloadSpline2Tangent = 125;
        String extensionCollectGoTo;
        String lowTurretDeliveryPosition;


        // Async Vars /////////////////////////////////////////////////////////////////////
        boolean step1 = false;
        boolean step2 = false;
        boolean step3 = false;
        boolean step4 = false;
        boolean step4a = false;
        boolean step5 = false;
        boolean step5a = false;

        int parking;

        ArrayList liftCollectionHeights;

        TrajectorySequence autoTrajectorySequence;


        // Open CV //////////////////////////////////////////////////////////////////////////
        final double FEET_PER_METER = 3.28084;
        int Ending_Location = 1;
        double fx = 578.272;
        double fy = 1000;
        double cx = 100;
        double cy = 221.506;
        double tagsize = 0.00037;
        int LEFT = 1;
        int MIDDLE = 2;
        int RIGHT = 3;

        String side = "Right";

        if (side == "Left") {
            initialTangent = -80;
            collectConeTangent = 90;
            initialApproachTangent = 90;
            highPoleDepositingPositionTangent = 0;
            depositPreloadSpline2Tangent = 25;
            endParking = new Vector2d(parking2.getX(), parking2.getY());
        } else if (side == "Right") {
            depositPreloadonLow = new Vector2d(depositPreloadonLow.getX(), -depositPreloadonLow.getY());
            initialTangent = 80;
            collectConeTangent = -90;
            initialApproachTangent = -90;
            highPoleDepositingPositionTangent = 0;
            depositPreloadSpline2Tangent = -115;
            startPosition = new Pose2d(startPosition.getX(), -startPosition.getY(), Math.toRadians(-90));
            initialTurn = 90;
            collectionApporach = new Vector2d(collectionApporach.getX(), -collectionApporach.getY());
            highPoleDepositingPosition = new Pose2d(highPoleDepositingPosition.getX(), -highPoleDepositingPosition.getY(), Math.toRadians(180));
            highPoleDepositingPosition2 = new Pose2d(highPoleDepositingPosition2.getX(), -highPoleDepositingPosition2.getY(), Math.toRadians(180));
            lowPoleDepositingPosition = new Pose2d(lowPoleDepositingPosition.getX(), -lowPoleDepositingPosition.getY(), Math.toRadians(180));
            highPoleDepositingIntermediatePoint = new Pose2d(highPoleDepositingPosition.getX() + 2, highPoleDepositingPosition.getY(), highPoleDepositingPosition.getHeading());
            depositPreloadForward = new Pose2d(depositPreloadForward.getX(), -depositPreloadForward.getY(), -depositPreloadForward.getHeading());
            depositPreLoadForwardVector = new Vector2d(depositPreLoadForwardVector.getX(), -depositPreLoadForwardVector.getY());
            depositPreLoadForwardHeading = Math.toRadians(-100);
            approachVector = new Vector2d(approachVector.getX(), -approachVector.getY());
            approachHeading = Math.toRadians(-90);
            collectConesPosition = new Pose2d(collectConesPosition.getX(), -collectConesPosition.getY(), Math.toRadians(180));
            depositOnHighPole1approach = new Pose2d(depositOnHighPole1approach.getX(), -depositOnHighPole1approach.getY(), Math.toRadians(180));
            depositOnHighPole1 = new Pose2d(depositOnHighPole1.getX(), -depositOnHighPole1.getY(), Math.toRadians(180));
            depositOnHighPole2 = new Pose2d(depositOnHighPole2.getX(), -depositOnHighPole2.getY(), Math.toRadians(180));
            secondHighPoleDepositingPosition = new Pose2d(secondHighPoleDepositingPosition.getX(), -secondHighPoleDepositingPosition.getY(), Math.toRadians(180));
            parking1 = new Vector2d(-12, 12.5);
            parking2 = new Vector2d(-36, 12.5);
            parking3 = new Vector2d(-60, 12.5);
            endParking = new Vector2d(-36, 12.5);
        }

        MeepMeep meepMeep = new MeepMeep(900, 60);

        Pose2d finalHighPoleDepositingPosition2 = highPoleDepositingPosition2;
        int finalHighPoleDepositingPositionTangent = highPoleDepositingPositionTangent;
        Pose2d finalCollectConesPosition = collectConesPosition;
        Pose2d finalHighPoleDepositingPosition1 = highPoleDepositingPosition;
        Vector2d finalInitialApproach = depositPreloadonLow;
        int finalInitialTangent = initialTangent;
        int finalCollectConesTangent = collectConeTangent;
        Pose2d finalStartPosition = startPosition;
        int finalInitialApproachTangent = initialApproachTangent;
        Vector2d finalDepositPreloadonLow = depositPreloadonLow;
        Vector2d finalcollectionApporach = collectionApporach;
        Pose2d finallowPoleDepositingPosition = lowPoleDepositingPosition;
        Pose2d finalSecondHighPoleDepositingPosition = secondHighPoleDepositingPosition;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(48, 50, 4.5, 4.5, 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(finalStartPosition)
                                .setTangent(finalInitialTangent)
                                .UNSTABLE_addTemporalMarkerOffset(0.83, () -> {

                                })

                                //Preload
                                .splineToConstantHeading(finalDepositPreloadonLow, Math.toRadians(finalInitialApproachTangent))
                                .waitSeconds(1)
                                .splineToConstantHeading(finalcollectionApporach, Math.toRadians(-100))
                                .splineToSplineHeading(finalCollectConesPosition, Math.toRadians(180))
                                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {

                                })

                                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {

                                })
                                .waitSeconds(0.1)

                                //FIRST CONE LOW POLE

                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(finallowPoleDepositingPosition.getX(), finallowPoleDepositingPosition.getY()), Math.toRadians(180))
                                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {

                                })


                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(finalCollectConesPosition.getX(), finalCollectConesPosition.getY()), Math.toRadians(180))
                                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {

                                })

                                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {

                                })
                                .waitSeconds(0.1)

                                //SECOND CONE MEDIUM POLE

                                .splineToSplineHeading(finalHighPoleDepositingPosition1, Math.toRadians(finalHighPoleDepositingPositionTangent),
                                        SampleMecanumDrive.getVelocityConstraint(50, 4.5, 12),
                                        SampleMecanumDrive.getAccelerationConstraint(50))
                                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {

                                })


                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {

                                })

                                .waitSeconds(1)

                                //CONE COLLECTION

                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(finalCollectConesPosition.getX(), finalCollectConesPosition.getY()), Math.toRadians(180 - finalHighPoleDepositingPositionTangent),
                                        SampleMecanumDrive.getVelocityConstraint(60, 4.5, 12),
                                        SampleMecanumDrive.getAccelerationConstraint(60))

                                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {

                                })

                                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {

                                })
                                .waitSeconds(0.1)
                                .setReversed(true)

                                //THIRD CONE HIGH POLE

                                .splineToConstantHeading(new Vector2d(finalHighPoleDepositingPosition2.getX(), finalHighPoleDepositingPosition2.getY()),
                                        Math.toRadians(finalHighPoleDepositingPositionTangent),
                                        SampleMecanumDrive.getVelocityConstraint(45, 4.5, 12),
                                        SampleMecanumDrive.getAccelerationConstraint(45))
                                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {

                                })


                                .waitSeconds(1)
//
//                                //CONE COLLECTION

                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(finalCollectConesPosition.getX(), finalCollectConesPosition.getY()), Math.toRadians(180 - finalHighPoleDepositingPositionTangent),
                                        SampleMecanumDrive.getVelocityConstraint(60, 4.5, 12),
                                        SampleMecanumDrive.getAccelerationConstraint(60))

                                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {

                                })

                                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {

                                })
                                .waitSeconds(0.1)
                                .setReversed(true)

                                //FOURTH CONE HIGH POLE

                                .splineToConstantHeading(new Vector2d(finalSecondHighPoleDepositingPosition.getX(), finalSecondHighPoleDepositingPosition.getY()),
                                        Math.toRadians(finalHighPoleDepositingPositionTangent),
                                        SampleMecanumDrive.getVelocityConstraint(60, 4.5, 12),
                                        SampleMecanumDrive.getAccelerationConstraint(60))
                                .UNSTABLE_addTemporalMarkerOffset(-0.85, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {

                                })


                                .waitSeconds(1)

                                //CONE COLLECTION

                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(finalCollectConesPosition.getX(), finalCollectConesPosition.getY()), Math.toRadians(180 - finalHighPoleDepositingPositionTangent),
                                        SampleMecanumDrive.getVelocityConstraint(60, 4.5, 12),
                                        SampleMecanumDrive.getAccelerationConstraint(60))

                                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {

                                })

                                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {

                                })
                                .waitSeconds(0.1)
                                .setReversed(true)

                                //FIFTH CONE DEPOSITING

                                .splineToConstantHeading(new Vector2d(finalSecondHighPoleDepositingPosition.getX(), finalSecondHighPoleDepositingPosition.getY()),
                                        Math.toRadians(finalHighPoleDepositingPositionTangent),
                                        SampleMecanumDrive.getVelocityConstraint(60, 4.5, 12),
                                        SampleMecanumDrive.getAccelerationConstraint(60))
                                .UNSTABLE_addTemporalMarkerOffset(-0.85, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {

                                })


                                .waitSeconds(1)


//
//                                .setReversed(false)
//                                .splineToConstantHeading(finalCollectConesPosition, Math.toRadians(180 - finalHighPoleDepositingPositionTangent),
//                                        SampleMecanumDrive.getVelocityConstraint(60, 4.5, 12),
//                                        SampleMecanumDrive.getAccelerationConstraint(60))
//
//                                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
//
//                                })
//
//                                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//
//                                })
//                                .waitSeconds(0.1)
//                                .setReversed(true)
//                                .splineToConstantHeading(new Vector2d(finalHighPoleDepositingPosition.getX(), finalHighPoleDepositingPosition.getY()),
//                                        Math.toRadians(finalHighPoleDepositingPositionTangent),
//                                        SampleMecanumDrive.getVelocityConstraint(45, 4.5, 12),
//                                        SampleMecanumDrive.getAccelerationConstraint(45))
//                                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//
//                                })
//
//
//                                .waitSeconds(1)
//
//                                //CYCLE THREE
//
//                                .setReversed(false)
//                                .splineToConstantHeading(finalCollectConesPosition, Math.toRadians(180 - finalHighPoleDepositingPositionTangent),
//                                        SampleMecanumDrive.getVelocityConstraint(60, 4.5, 12),
//                                        SampleMecanumDrive.getAccelerationConstraint(60))
//
//                                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
//
//                                })
//
//                                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//
//                                })
//                                .waitSeconds(0.1)
//                                .setReversed(true)
//                                .splineToConstantHeading(new Vector2d(finalHighPoleDepositingPosition.getX(), finalHighPoleDepositingPosition.getY()),
//                                        Math.toRadians(finalHighPoleDepositingPositionTangent),
//                                        SampleMecanumDrive.getVelocityConstraint(45, 4.5, 12),
//                                        SampleMecanumDrive.getAccelerationConstraint(45))
//                                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//
//                                })
//
//                                .waitSeconds(1)
//
//                                //CYCLE FOUR
//
//                                .setReversed(false)
//                                .splineToConstantHeading(finalCollectConesPosition, Math.toRadians(180 - finalHighPoleDepositingPositionTangent),
//                                        SampleMecanumDrive.getVelocityConstraint(60, 4.5, 12),
//                                        SampleMecanumDrive.getAccelerationConstraint(60))
//
//                                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
//                                })
//
//                                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                                })
//                                .waitSeconds(0.1)
//                                .setReversed(true)
//                                .splineToConstantHeading(new Vector2d(finalHighPoleDepositingPosition.getX(), finalHighPoleDepositingPosition.getY()),
//                                        Math.toRadians(finalHighPoleDepositingPositionTangent),
//                                        SampleMecanumDrive.getVelocityConstraint(45, 4.5, 12),
//                                        SampleMecanumDrive.getAccelerationConstraint(45))
//                                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//
//                                })
//
//
//                                .waitSeconds(1)
//
//                                //CYCLE FIVE
//                                .setReversed(false)
//                                .splineToConstantHeading(finalCollectConesPosition, Math.toRadians(180 - finalHighPoleDepositingPositionTangent),
//                                        SampleMecanumDrive.getVelocityConstraint(60, 4.5, 12),
//                                        SampleMecanumDrive.getAccelerationConstraint(60))
//
//                                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
//
//                                })
//
//                                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//
//                                })
//                                .waitSeconds(0.1)
//                                .setReversed(true)
//                                .splineToConstantHeading(new Vector2d(finalHighPoleDepositingPosition.getX(), finalHighPoleDepositingPosition.getY()),
//                                        Math.toRadians(finalHighPoleDepositingPositionTangent),
//                                        SampleMecanumDrive.getVelocityConstraint(45, 4.5, 12),
//                                        SampleMecanumDrive.getAccelerationConstraint(45))
//                                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
//
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
//
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//
//                                })


                                .waitSeconds(1).build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
