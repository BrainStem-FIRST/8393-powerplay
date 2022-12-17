
package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.imagecv.AprilTagDetectionPipeline;
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

public class AutoCore extends LinearOpMode {
    private final AutoOrientation side;
    private Map stateMap;

    private Pose2d endParking;


    // Locations - For Red /////////////////////////////////////////////////////////////////////
    private Pose2d startPosition = new Pose2d(-36, -64, Math.toRadians(-90));
    private Pose2d signalConeKnockout = new Pose2d(-36, -24, Math.toRadians(0));
    private Pose2d strafeToDeposit = new Pose2d(-60, -58, Math.toRadians(-90));
    private Vector2d depositPreloadSpline = new Vector2d(-60, -54.5);
    private Vector2d depositerPreloadSplineIntermediatePoint = new Vector2d(-58, -30);
    private Vector2d depositPreloadSpline2 = new Vector2d(-55.25, -11.75);
    private Pose2d depositPreLoad = new Pose2d(-57, -12, Math.toRadians(-100));
    private Pose2d approachPosition = new Pose2d(-60, -24, Math.toRadians(-90));
    private Pose2d collectConesPosition = new Pose2d(-64.25, -12, Math.toRadians(0));
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

    private int initialTurn = -90;

    private Pose2d parking3 = new Pose2d(12, 12.5, Math.toRadians(-90));
    private Pose2d parking2 = new Pose2d(36, 12.5, Math.toRadians(-90));
    private Pose2d parking1 = new Pose2d(60, 12.5, Math.toRadians(-90));

    private int initialTangent = -80;
    private int depositPreloadSplineTangent = 90;
    private int depositorPreloadSplineIntermediatePointTangent = 90;
    private int depositPreloadSpline2Tangent = 125;
    private String extensionCollectGoTo;


    // Async Vars /////////////////////////////////////////////////////////////////////
    private boolean step1 = false;
    private boolean step2 = false;
    private boolean step3 = false;
    private boolean step4 = false;
    private boolean step4a = false;
    private boolean step5 = false;
    private boolean step5a = false;

    private int parking;

    private ArrayList liftCollectionHeights;
    Constants constants = new Constants();


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

    public AutoCore(AutoOrientation side) {
        this.side = side;
        switch (side) {
            case LEFT:
                initialTangent = -80;
                depositPreloadSplineTangent = 90;
                depositorPreloadSplineIntermediatePointTangent = 90;
                depositPreloadSpline2Tangent = -25;
                break;
            case RIGHT:
                initialTangent = 80;
                depositPreloadSplineTangent = -90;
                depositorPreloadSplineIntermediatePointTangent = -90;
                depositPreloadSpline2Tangent = -115;
                startPosition = new Pose2d(startPosition.getX(), -startPosition.getY(), Math.toRadians(90));
                initialTurn = 90;
                strafeToDeposit = new Pose2d(strafeToDeposit.getX(), -strafeToDeposit.getY(), -strafeToDeposit.getHeading());
                depositPreloadSpline = new Vector2d(depositPreloadSpline.getX(), -depositPreloadSpline.getY());
                depositerPreloadSplineIntermediatePoint = new Vector2d(depositerPreloadSplineIntermediatePoint.getX(), -depositerPreloadSplineIntermediatePoint.getY());
                depositPreloadSpline2 = new Vector2d(depositPreloadSpline2.getX(), -depositPreloadSpline2.getY());
                depositPreloadForward = new Pose2d(depositPreloadForward.getX(), -depositPreloadForward.getY(), -depositPreloadForward.getHeading());
                depositPreLoadForwardVector = new Vector2d(depositPreLoadForwardVector.getX(), -depositPreLoadForwardVector.getY());
                depositPreLoadForwardHeading = Math.toRadians(-100);
                approachVector = new Vector2d(approachVector.getX(), -approachVector.getY());
                approachHeading = Math.toRadians(-90);
                depositPreLoad = new Pose2d(depositPreLoad.getX(), -depositPreLoad.getY(), -depositPreLoad.getHeading());
                approachPosition = new Pose2d(approachPosition.getX(), -approachPosition.getY(), -approachPosition.getHeading());
                collectConesPosition = new Pose2d(collectConesPosition.getX(), -collectConesPosition.getY(), Math.toRadians(90));
                depositOnHighPole1approach = new Pose2d(depositOnHighPole1approach.getX(), -depositOnHighPole1approach.getY(), Math.toRadians(180));
                depositOnHighPole1 = new Pose2d(depositOnHighPole1.getX(), -depositOnHighPole1.getY(), Math.toRadians(180));
                depositOnHighPole2 = new Pose2d(depositOnHighPole2.getX(), -depositOnHighPole2.getY(), Math.toRadians(180));


                parking1 = new Pose2d(-12, 12.5, Math.toRadians(90));
                parking2 = new Pose2d(-36, 12.5, Math.toRadians(90));
                parking3 = new Pose2d(-60, 12.5, Math.toRadians(90));
                break;
        }
        endParking = new Pose2d(parking2.getX(), parking2.getY(), parking2.getHeading());

    }


    @Override
    public void runOpMode() throws InterruptedException {

        // Timers ////////////////////////////////////////////////////////////////////////
        ElapsedTime runTime = new ElapsedTime();
        ElapsedTime totalTime = new ElapsedTime();

        // Hardwhare ///////////////////////////////////////////////////////////////////
        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        this.stateMap = new HashMap<String, String>() {{
        }};
        AutoBrainSTEMRobot robot = new AutoBrainSTEMRobot(this.hardwareMap, this.telemetry, this.stateMap, true);

        switch (side) {
            case LEFT:
                extensionCollectGoTo = robot.arm.AUTO_EXTENSION_COLLECT_LEFT;
                turretPickupPosition = robot.turret.LEFT_PICKUP_AUTO;
                turretDeliveryPosition = robot.turret.LEFT_DELIVERY_AUTO;
                break;
            case RIGHT:
                extensionCollectGoTo = robot.arm.AUTO_EXTENSION_COLLECT_RIGHT;
                turretPickupPosition = robot.turret.RIGHT_PICKUP_AUTO;
                turretDeliveryPosition = robot.turret.RIGHT_DELIVERY_AUTO;
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
        while (!this.opModeIsActive() && !this.isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {

                    if (tag.id == MIDDLE) {
                        tagOfInterest = tag;
                        location = ParkingLocation.MID;
                        parking = 2;
                        tagFound = true;
                        telemetry.addData("Open CV :", "Mid");
                        telemetry.update();
                        endParking = new Pose2d(parking2.getX(), parking2.getY(), parking2.getHeading());
                        break;

                    } else if (tag.id == RIGHT) {
                        tagOfInterest = tag;
                        location = ParkingLocation.RIGHT;
                        parking = 3;
                        tagFound = true;
                        telemetry.addData("Open CV :", "Right");
                        telemetry.update();
                        endParking = new Pose2d(parking3.getX(), parking3.getY(), parking3.getHeading());
                        break;

                    } else {
                        tagOfInterest = tag;
                        location = ParkingLocation.LEFT;
                        tagFound = true;
                        parking = 1;
                        telemetry.addData("Open CV :", "Left");
                        telemetry.update();
                        endParking = new Pose2d(parking1.getX(), parking1.getY(), parking1.getHeading());
                        break;

                    }
                }
            }
        }
        this.waitForStart();
        camera.closeCameraDevice();
        drive.setPoseEstimate(startPosition);
        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
        stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_GRABBER, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CONE_CYCLE, constants.STATE_NOT_STARTED);
        robot.updateSystems();

        totalTime.reset();

        TrajectorySequence deliverPreload = drive.trajectorySequenceBuilder(startPosition)
                .setReversed(true)
                .setTangent(initialTangent)
                .waitSeconds(0.0)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW);
                    robot.lift.setSubheight(0.9);
                })
                .splineToConstantHeading(depositPreloadSpline, Math.toRadians(depositPreloadSplineTangent), SampleMecanumDrive.getVelocityConstraint(24, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToConstantHeading(depositerPreloadSplineIntermediatePoint, Math.toRadians(depositorPreloadSplineIntermediatePointTangent), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .splineToConstantHeading(depositPreloadSpline2, Math.toRadians(depositPreloadSpline2Tangent), SampleMecanumDrive.getVelocityConstraint(24, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW);
                    robot.lift.setSubheight(0);
                })

//            FAKE

               .waitSeconds(0.5)
               .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                   stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW);
             })

               .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                 stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);
               })
               .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                  stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.AUTO_EXTENSION_DEPOSIT);
              })
               .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                   robot.lift.setSubheight(0.7);
               })
               .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                  stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.FULLY_OPEN);
               })
              .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                  robot.lift.setSubheight(0.0);
              })

//                REAL

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.AUTO_EXTENSION_DEPOSIT);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.lift.setSubheight(0.7);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.FULLY_OPEN);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    robot.lift.setSubheight(0.0);
                })

                //CYCLE ONE
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, turretPickupPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_SIDE_EXTENDED_AUTO);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.lift.setSubheight(0.95);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, extensionCollectGoTo);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.3, () -> {
                    robot.lift.setSubheight(0);
                })

                .waitSeconds(2.5)

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.AUTO_EXTENSION_DEPOSIT);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.lift.setSubheight(0.7);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.FULLY_OPEN);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    robot.lift.setSubheight(0.0);
                })

                //CYCLE 2
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, turretPickupPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_SIDE_EXTENDED_AUTO);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.lift.setSubheight(1.1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, extensionCollectGoTo);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.3, () -> {
                    robot.lift.setSubheight(0);
                })

                .waitSeconds(2.5)

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.AUTO_EXTENSION_DEPOSIT);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.lift.setSubheight(0.7);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.FULLY_OPEN);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    robot.lift.setSubheight(0.0);
                })

                //CYCLE 3
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, turretPickupPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_SIDE_EXTENDED_AUTO);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.lift.setSubheight(1.25);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, extensionCollectGoTo);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.3, () -> {
                    robot.lift.setSubheight(0);
                })

                .waitSeconds(2.5)

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.AUTO_EXTENSION_DEPOSIT);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.lift.setSubheight(0.7);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.FULLY_OPEN);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    robot.lift.setSubheight(0.0);
                })

                //CYCLE 4
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, turretPickupPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_SIDE_EXTENDED_AUTO);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.lift.setSubheight(1.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, extensionCollectGoTo);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.3, () -> {
                    robot.lift.setSubheight(0);
                })

                .waitSeconds(2.5)

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.AUTO_EXTENSION_DEPOSIT);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.lift.setSubheight(0.7);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.FULLY_OPEN);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    robot.lift.setSubheight(0.0);
                })

                //CYCLE 5
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, turretPickupPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_SIDE_EXTENDED_AUTO);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.lift.setSubheight(1.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, extensionCollectGoTo);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.3, () -> {
                    robot.lift.setSubheight(0);
                })

                .waitSeconds(2.5)

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.AUTO_EXTENSION_DEPOSIT);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.lift.setSubheight(0.7);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.FULLY_OPEN);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    robot.lift.setSubheight(0.0);
                })


                .waitSeconds(5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
                })


//                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.turret.SYSTEM_NAME, turretPickupPosition); })
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE); })
//                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.STACK_5); })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.AUTO_EXTENSION_COLLECT); })
//                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> { robot.grabber.close(); })
//                .UNSTABLE_addTemporalMarkerOffset(1.65, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW); })
//                .waitSeconds(2.5)
//
//
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);})
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.AUTO_EXTENSION_DEPOSIT_TILTED); })
//                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> { stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS); })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW); })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> { resetLift(); })
//                .waitSeconds(1.25)
//
//
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.turret.SYSTEM_NAME, turretPickupPosition); })
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE); })
//                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.STACK_4); })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.AUTO_EXTENSION_COLLECT); })
//                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> { robot.grabber.close(); })
//                .UNSTABLE_addTemporalMarkerOffset(1.65, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW); })
//                .waitSeconds(2.5)
//
//
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);})
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.AUTO_EXTENSION_DEPOSIT_TILTED); })
//                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> { stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS); })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW); })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> { resetLift(); })
//                .waitSeconds(1.25)
//
//
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.turret.SYSTEM_NAME, turretPickupPosition); })
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE); })
//                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.STACK_3); })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.AUTO_EXTENSION_COLLECT); })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> { robot.grabber.close(); })
//                .UNSTABLE_addTemporalMarkerOffset(1.75, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW); })
//                .waitSeconds(2.5)
//
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);})
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.AUTO_EXTENSION_DEPOSIT_TILTED); })
//                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> { stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS); })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW); })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> { resetLift(); })
//                .waitSeconds(1.25)
//
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.turret.SYSTEM_NAME, turretPickupPosition); })
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE); })
//                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.STACK_2); })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.AUTO_EXTENSION_COLLECT); })
//                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> robot.lift.setAllMotorPowers(-0.45))
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> robot.lift.setAllMotorPowers(0))
//                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> { robot.grabber.close(); })
//                .UNSTABLE_addTemporalMarkerOffset(1.75, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW); })
//                .waitSeconds(2.75)
//
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);})
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.AUTO_EXTENSION_DEPOSIT_TILTED); })
//                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> { stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS); })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW); })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> { resetLift(); })
//                .waitSeconds(1.25)
//
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.turret.SYSTEM_NAME, turretPickupPosition); })
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE); })
//                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.STACK_1); })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.AUTO_EXTENSION_COLLECT); })
//                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> robot.lift.setAllMotorPowers(-0.55))
//                .UNSTABLE_addTemporalMarkerOffset(1.65, () -> robot.lift.setAllMotorPowers(0))
//                .UNSTABLE_addTemporalMarkerOffset(1.8, () -> { robot.grabber.close(); })
//                .UNSTABLE_addTemporalMarkerOffset(2, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW); })
//                .waitSeconds(3.0)
//
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.turret.SYSTEM_NAME, turretDeliveryPosition);})
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.AUTO_EXTENSION_DEPOSIT_TILTED); })
//                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> { stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS); })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW); })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> { resetLift(); })
//                .waitSeconds(1.25)


                .build();
        // second cycle

        drive.followTrajectorySequenceAsync(deliverPreload);

        // at 29 seconds the lift runs down in auto

        while (opModeIsActive()) {

            if (totalTime.seconds() < 27) {
                drive.update();
                robot.updateSystems();
                telemetry.update();
            } else {

                stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
                stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
                stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
                stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
                stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
                robot.updateSystems();

                Trajectory parking = drive.trajectoryBuilder(drive.getPoseEstimate())
                       .lineToLinearHeading(endParking)
                        .build();
               drive.followTrajectory(parking);


            }
        }


    }

    private void resetLift() {
        stateMap.put(constants.CONE_CYCLE, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_GRABBER, constants.STATE_NOT_STARTED);
    }
}