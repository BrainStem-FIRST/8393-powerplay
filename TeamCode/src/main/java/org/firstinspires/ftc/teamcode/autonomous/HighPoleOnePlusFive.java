
package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.imagecv.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.AutoBrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class HighPoleOnePlusFive extends LinearOpMode {
    private final AutoOrientation side;
    private Map stateMap;

    // Locations - For Left /////////////////////////////////////////////////////////////////////
    private Pose2d startPosition = new Pose2d(-36.00, -64.00, Math.toRadians(-90));
    private Pose2d initialApproach = new Pose2d(-36.00, -24.00, Math.toRadians(-90));
    private Pose2d highPoleDepositingPositionRight = new Pose2d(-21.25, 12.00, Math.toRadians(180));
    private Pose2d highPoleDepositingPositionLeft = new Pose2d(-22.50, -12.00, Math.toRadians(180));
    private Pose2d highPoleDepositingPosition;
    private Pose2d lowPoleDepositingPosition = new Pose2d(-47.50, -11.50, Math.toRadians(0));
//    private Vector2d collectConesPosition = new Vector2d(-55.50, -12.00); //-55.5
    private Vector2d collectConesPositionPreload = new Vector2d(-56.5, -12.00); //-55.5
    private Vector2d collectConesPositionApproach = new Vector2d(-49, -12.00); //-55.5
    private Pose2d depositOnHighPole1approach = new Pose2d(-35, -12, Math.toRadians(0));
    private Pose2d depositOnHighPole1 = new Pose2d(-24, -12, Math.toRadians(0));
    private Pose2d depositOnHighPole2 = new Pose2d(-25, -12, Math.toRadians(0));
    private Pose2d depositPreloadForward = new Pose2d(-62, -36, Math.toRadians(100));
    private Vector2d approachVector = new Vector2d(-60, -24);
    private double approachHeading = Math.toRadians(90);
    private Vector2d depositPreLoadForwardVector = new Vector2d(-57, -13.5);
    private double depositPreLoadForwardHeading = Math.toRadians(100);
    private String turretPickupPosition;
    private String turretDeliveryPosition;

    private String extensionDeliverySide;

    private String depositingExtension;
    private boolean LEFTSIDE;
    private int initialTurn = -90;


    private Pose2d parking3 = new Pose2d(-12, -12.5, startPosition.getHeading());
    private Pose2d parking2 = new Pose2d(-36, -12.5, startPosition.getHeading());
    private Pose2d parking1 = new Pose2d(-60, -12.5, Math.toRadians(180));
    private Pose2d endParking;


    private int initialTangent = -80;
    private int initialApproachTangent = 180;
    private int highPoleDepositingPositionTangent = 90;
    private int depositPreloadSpline2Tangent = 125;
    private String extensionCollectGoTo;
    private String lowTurretDeliveryPosition;


    // Async Vars /////////////////////////////////////////////////////////////////////
    private boolean step1 = false;
    private boolean step2 = false;
    private boolean step3 = false;
    private boolean step4 = false;
    private boolean step4a = false;
    private boolean step5 = false;
    private boolean step5a = false;

    private int parking;

    private boolean trajectoryCalculated = false;

    private ArrayList liftCollectionHeights;
    Constants constants = new Constants();

    TrajectorySequence autoTrajectorySequence;


    // Open CV //////////////////////////////////////////////////////////////////////////
    private ParkingLocation location = ParkingLocation.LEFT;
    public OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    public int Ending_Location = 1;
    double fx = 578.272;
    double fy = 1000;
    double cx = 100;
    double cy = 221.506;
    double tagsize = 0.00037;
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    AprilTagDetection tagOfInterest = null;


    private enum ParkingLocation {
        LEFT, MID, RIGHT
    }
    public enum AutoOrientation {
        RIGHT, LEFT
    }

    public HighPoleOnePlusFive(AutoOrientation side) {
        this.side = side;
        switch (side) {
            case RIGHT:
                LEFTSIDE = false;
                initialApproach = new Pose2d(initialApproach.getX() , -initialApproach.getY() , Math.toRadians(120));
                initialTangent = 80;
                initialApproachTangent = -90;
                highPoleDepositingPositionTangent = 0;
                depositPreloadSpline2Tangent = -115;
                startPosition = new Pose2d(startPosition.getX(), -startPosition.getY(), Math.toRadians(90));
                initialTurn = 90;
                highPoleDepositingPosition = new Pose2d(highPoleDepositingPositionRight.getX(), highPoleDepositingPositionRight.getY(), Math.toRadians(180));
                lowPoleDepositingPosition = new Pose2d(lowPoleDepositingPosition.getX(), -lowPoleDepositingPosition.getY(), Math.toRadians(180));
                depositPreloadForward = new Pose2d(depositPreloadForward.getX(), -depositPreloadForward.getY(), -depositPreloadForward.getHeading());
                depositPreLoadForwardVector = new Vector2d(depositPreLoadForwardVector.getX(), -depositPreLoadForwardVector.getY());
                depositPreLoadForwardHeading = Math.toRadians(-100);
                approachVector = new Vector2d(approachVector.getX(), -approachVector.getY());
                approachHeading = Math.toRadians(-90);
//                collectConesPosition = new Vector2d(collectConesPosition.getX(), -collectConesPosition.getY());
                collectConesPositionPreload = new Vector2d(collectConesPositionPreload.getX(), -collectConesPositionPreload.getY());
                collectConesPositionApproach = new Vector2d(collectConesPositionApproach.getX(), -collectConesPositionApproach.getY());
                depositOnHighPole1approach = new Pose2d(depositOnHighPole1approach.getX(), -depositOnHighPole1approach.getY(), Math.toRadians(180));
                depositOnHighPole1 = new Pose2d(depositOnHighPole1.getX(), -depositOnHighPole1.getY(), Math.toRadians(180));
                depositOnHighPole2 = new Pose2d(depositOnHighPole2.getX(), -depositOnHighPole2.getY(), Math.toRadians(180));


                parking1 = new Pose2d(-11.5, 12.5, startPosition.getHeading() + 20);
                parking2 = new Pose2d(-36, 12.5, startPosition.getHeading() + 20);
                parking3 = new Pose2d(-60, 12.5, Math.toRadians(180));
                break;
            case LEFT:
                highPoleDepositingPosition = new Pose2d(highPoleDepositingPositionLeft.getX(), highPoleDepositingPositionLeft.getY(), Math.toRadians(180));
                LEFTSIDE = true;
                initialTangent = -80;
                initialApproachTangent = 100;
                highPoleDepositingPositionTangent = 0;
                depositPreloadSpline2Tangent = 25;
                break;
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {

        // Timers ////////////////////////////////////////////////////////////////////////
        ElapsedTime runTime = new ElapsedTime();
        ElapsedTime totalTime = new ElapsedTime();

        // Hardwhare ///////////////////////////////////////////////////////////////////
        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        this.stateMap = new HashMap<String, String>() {
        };
        AutoBrainSTEMRobot robot = new AutoBrainSTEMRobot(this.hardwareMap, this.telemetry, this.stateMap, true);

        switch (side) {
            case LEFT:
                lowTurretDeliveryPosition = robot.turret.LEFT_POSITION;
                extensionCollectGoTo = robot.arm.AUTO_EXTENSION_COLLECT_LEFT;
                turretPickupPosition = robot.turret.LEFT_PICKUP_AUTO;
                turretDeliveryPosition = robot.turret.RIGHT_POSITION;
                extensionDeliverySide = robot.arm.RIGHT_SIDE_EXTENDED_AUTO;
                endParking = new Pose2d(parking3.getX(), parking3.getY(), startPosition.getHeading());
                break;
            case RIGHT:
                lowTurretDeliveryPosition = robot.turret.RIGHT_POSITION;
                extensionCollectGoTo = robot.arm.AUTO_EXTENSION_COLLECT_RIGHT;
                turretPickupPosition = robot.turret.RIGHT_PICKUP_AUTO;
                turretDeliveryPosition = robot.turret.LEFT_POSITION;
                extensionDeliverySide = robot.arm.LEFT_SIDE_EXTENDED_AUTO;
                endParking = new Pose2d(parking1.getX(), parking1.getY(), startPosition.getHeading());
                break;
        }

        // State Map ////////////////////////////////////////////////////////////////
        this.stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
        this.stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
        this.stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
        this.stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);

        // Open CV //////////////////////////////////////////////////////////////////
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam-2"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        robot.grabber.close();
        robot.poleAligner.poleAlignerServo.setPosition(0.201);
        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
        stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_GRABBER, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CONE_CYCLE, constants.STATE_NOT_STARTED);
        stateMap.put(robot.poleAligner.SYSTEM_NAME, robot.poleAligner.INIT);
        drive.setPoseEstimate(startPosition);

        while (!this.opModeIsActive() && !this.isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {

                    if (tag.id == MIDDLE) {
                        tagOfInterest = tag;
                        location = ParkingLocation.MID;
                        if (parking != 2) {
                            trajectoryCalculated = false;
                        }
                        parking = 2;
                        tagFound = true;
                        telemetry.addData("Open CV :", "Mid");
                        endParking = parking2;
                        break;

                    } else if (tag.id == RIGHT) {
                        tagOfInterest = tag;
                        location = ParkingLocation.RIGHT;
                        if (parking != 3) {
                            trajectoryCalculated = false;
                        }
                        parking = 3;
                        tagFound = true;
                        telemetry.addData("Open CV :", "Right");
                        endParking = parking3;
                        break;

                    } else {
                        tagOfInterest = tag;
                        location = ParkingLocation.LEFT;
                        tagFound = true;
                        if (parking != 1) {
                            trajectoryCalculated = false;
                        }
                        parking = 1;
                        telemetry.addData("Open CV :", "Left");
                        endParking = parking1;
                        break;

                    }
                }
            }

            if (trajectoryCalculated == false) {
                autoTrajectorySequence = initializeTrajectories(robot, drive);
                trajectoryCalculated = true;
            }

            if (endParking != null) {
                telemetry.addData("Parking", endParking);
            }
            telemetry.update();
        }

        this.waitForStart();
        camera.closeCameraDevice();
        robot.updateSystems();
        drive.followTrajectorySequenceAsync(autoTrajectorySequence);

        while (opModeIsActive()) {
            telemetry.addData("Parking", endParking);
            telemetry.update();
            drive.update();
            robot.updateSystems();
        }
    }

    private TrajectorySequence initializeTrajectories(AutoBrainSTEMRobot robot, SampleMecanumDrive drive) {

        TrajectorySequence deliverPreload = drive.trajectorySequenceBuilder(startPosition)
                .setReversed(true)
                .setTangent(initialTangent)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.STACK_4);
                    stateMap.put(robot.poleAligner.SYSTEM_NAME, robot.poleAligner.UP);
                })
                .splineToSplineHeading(initialApproach, Math.toRadians(initialApproachTangent),
                        SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .splineToSplineHeading(highPoleDepositingPosition, Math.toRadians(highPoleDepositingPositionTangent),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(45))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_HIGH);
                    stateMap.put(robot.poleAligner.SYSTEM_NAME, robot.poleAligner.DOWN);
                    robot.lift.setSubheight(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {

                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_COLLECTING_VALUE);
                    stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, extensionDeliverySide);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.lift.setSubheight(1.2);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.FULLY_OPEN);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.74, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_COLLECTING_VALUE);

                })

                .UNSTABLE_addTemporalMarkerOffset(0.84, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);

                })
                .UNSTABLE_addTemporalMarkerOffset(0.93, () -> {
                    robot.lift.setSubheight(0);

                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.COLLECTING_OPEN_AUTO);
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.STACK_5);
                })



                .waitSeconds(0.5)

                //CYCLE ONE


                .setReversed(false)
                .splineToConstantHeading(collectConesPositionApproach, Math.toRadians(180 - highPoleDepositingPositionTangent),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(53))
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
                    stateMap.put(robot.poleAligner.SYSTEM_NAME, robot.poleAligner.UP);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.LEFT_SIDE_EXTENDED_AUTO);
                })
                .forward(Math.abs(Math.abs(collectConesPositionPreload.getX()) - Math.abs(collectConesPositionApproach.getX())),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
//                .splineToConstantHeading(collectConesPositionPreload, Math.toRadians(180 - highPoleDepositingPositionTangent),
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(23))
                .UNSTABLE_addTemporalMarkerOffset(-0.55, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_COLLECTING_VALUE);
                })
                //.waitSeconds(0.02)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(highPoleDepositingPosition.getX(), highPoleDepositingPosition.getY()),
                        Math.toRadians(highPoleDepositingPositionTangent),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(50))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_HIGH);
                    stateMap.put(robot.poleAligner.SYSTEM_NAME, robot.poleAligner.DOWN);
                    robot.lift.setSubheight(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, extensionDeliverySide);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.lift.setSubheight(1.2);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.FULLY_OPEN);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.74, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_COLLECTING_VALUE);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.83, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.93, () -> {
                    robot.lift.setSubheight(0);
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.COLLECTING_OPEN_AUTO);
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.STACK_5);
                })


                .waitSeconds(0.5)


                //CYCLE TWO

                .setReversed(false)
                .splineToConstantHeading(collectConesPositionApproach, Math.toRadians(180 - highPoleDepositingPositionTangent),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(53))
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
                    stateMap.put(robot.poleAligner.SYSTEM_NAME, robot.poleAligner.UP);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.55, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.STACK_4);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, extensionCollectGoTo);
                })
                .forward(Math.abs(Math.abs(collectConesPositionPreload.getX()) - Math.abs(collectConesPositionApproach.getX())),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
//                .splineToConstantHeading(collectConesPositionPreload, Math.toRadians(180 - highPoleDepositingPositionTangent),
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(23))
                .UNSTABLE_addTemporalMarkerOffset(-0.55, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_COLLECTING_VALUE);
                })
                //.waitSeconds(0.02)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(highPoleDepositingPosition.getX(), highPoleDepositingPosition.getY()),
                        Math.toRadians(highPoleDepositingPositionTangent),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(50))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_HIGH);
                    stateMap.put(robot.poleAligner.SYSTEM_NAME, robot.poleAligner.DOWN);
                    robot.lift.setSubheight(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, extensionDeliverySide);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {



                    robot.lift.setSubheight(1.2);

                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.FULLY_OPEN);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.74, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_COLLECTING_VALUE);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.83, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.93, () -> {
                    robot.lift.setSubheight(0);
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.COLLECTING_OPEN_AUTO);
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.STACK_5);
                })


                .waitSeconds(0.5)

                //CYCLE THREE

                .setReversed(false)
                .splineToConstantHeading(collectConesPositionApproach, Math.toRadians(180 - highPoleDepositingPositionTangent),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(53))
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
                    stateMap.put(robot.poleAligner.SYSTEM_NAME, robot.poleAligner.UP);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.55, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.STACK_3);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, extensionCollectGoTo);
                })
                .forward(Math.abs(Math.abs(collectConesPositionPreload.getX()) - Math.abs(collectConesPositionApproach.getX())),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
//                .splineToConstantHeading(collectConesPositionPreload, Math.toRadians(180 - highPoleDepositingPositionTangent),
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(23))
                .UNSTABLE_addTemporalMarkerOffset(-0.55, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_COLLECTING_VALUE);
                })
                //.waitSeconds(0.02)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(highPoleDepositingPosition.getX(), highPoleDepositingPosition.getY()),
                        Math.toRadians(highPoleDepositingPositionTangent),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(50))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_HIGH);
                    stateMap.put(robot.poleAligner.SYSTEM_NAME, robot.poleAligner.DOWN);
                    robot.lift.setSubheight(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, extensionDeliverySide);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {



                    robot.lift.setSubheight(1.2);

                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.FULLY_OPEN);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.74, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_COLLECTING_VALUE);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.84, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.93, () -> {
                    robot.lift.setSubheight(0);
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.COLLECTING_OPEN_AUTO);
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.STACK_5);
                })

                .waitSeconds(0.5)

                //CYCLE FOUR

                .setReversed(false)
                .splineToConstantHeading(collectConesPositionApproach, Math.toRadians(180 - highPoleDepositingPositionTangent),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(53))
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
                    stateMap.put(robot.poleAligner.SYSTEM_NAME, robot.poleAligner.UP);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.55, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.STACK_2);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.LAST_TWO_CYCLES);
                })
                .forward(Math.abs(Math.abs(collectConesPositionPreload.getX()) - Math.abs(collectConesPositionApproach.getX())),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
//                .splineToConstantHeading(collectConesPositionPreload, Math.toRadians(180 - highPoleDepositingPositionTangent),
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(23))
                .UNSTABLE_addTemporalMarkerOffset(-0.55, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_COLLECTING_VALUE);
                })
                //.waitSeconds(0.02)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(highPoleDepositingPosition.getX(), highPoleDepositingPosition.getY()),
                        Math.toRadians(highPoleDepositingPositionTangent),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(50))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_HIGH);
                    stateMap.put(robot.poleAligner.SYSTEM_NAME, robot.poleAligner.DOWN);
                    robot.lift.setSubheight(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, extensionDeliverySide);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {



                    robot.lift.setSubheight(1.2);

                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.FULLY_OPEN);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.74, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_COLLECTING_VALUE);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.84, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.93, () -> {
                    robot.lift.setSubheight(0);
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.COLLECTING_OPEN_AUTO);
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.STACK_5);
                })


                .waitSeconds(0.5)

                //CYCLE FIVE
                .setReversed(false)
                .splineToConstantHeading(collectConesPositionApproach, Math.toRadians(180 - highPoleDepositingPositionTangent),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(53))
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
                    stateMap.put(robot.poleAligner.SYSTEM_NAME, robot.poleAligner.UP);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.55, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.STACK_1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, extensionCollectGoTo);
                })
                .forward(Math.abs(Math.abs(collectConesPositionPreload.getX()) - Math.abs(collectConesPositionApproach.getX())),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
//                .splineToConstantHeading(collectConesPositionPreload, Math.toRadians(180 - highPoleDepositingPositionTangent),
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(23))
                .UNSTABLE_addTemporalMarkerOffset(-0.55, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_COLLECTING_VALUE);
                })
                //.waitSeconds(0.02)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(highPoleDepositingPosition.getX(), highPoleDepositingPosition.getY()),
                        Math.toRadians(highPoleDepositingPositionTangent),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(50))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_HIGH);
                    stateMap.put(robot.poleAligner.SYSTEM_NAME, robot.poleAligner.DOWN);
                    robot.lift.setSubheight(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, extensionDeliverySide);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {



                    robot.lift.setSubheight(1.2);


                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.FULLY_OPEN);
                }).UNSTABLE_addTemporalMarkerOffset(0.74, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_COLLECTING_VALUE);

                })

                .UNSTABLE_addTemporalMarkerOffset(0.84, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.93, () -> {
                    robot.lift.setSubheight(0);
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW);

                })


                .waitSeconds(0.6)
                .lineToLinearHeading(endParking,
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60))

                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
                    stateMap.put(robot.poleAligner.SYSTEM_NAME, robot.poleAligner.UP);

                })

                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);

                })

                .build();

        return deliverPreload;
    }

    private void resetLift() {
        stateMap.put(constants.CONE_CYCLE, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_GRABBER, constants.STATE_NOT_STARTED);
    }
}