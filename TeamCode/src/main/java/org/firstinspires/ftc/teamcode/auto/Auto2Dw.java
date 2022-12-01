package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.imagecv.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class Auto2Dw extends LinearOpMode {
    private final AllianceColor color;
    private Map stateMap;

    private Pose2d endParking;


    // Locations - For Red /////////////////////////////////////////////////////////////////////
    private Pose2d startPosition = new Pose2d(-36, -64, Math.toRadians(90));
    private Pose2d signalConeKnockout = new Pose2d(-36, -24, Math.toRadians(180));
    private Pose2d centerOfChannelInitial = new Pose2d(-36, -12.5, Math.toRadians(90));
    private Pose2d depositPreLoad = new Pose2d(-48, -12, Math.toRadians(180));
    private Pose2d approachPosition = new Pose2d(-56, -12, Math.toRadians(180));
    private Pose2d collectConesPosition = new Pose2d(-63, -12, Math.toRadians(180));
    private Pose2d depositOnHighPole1approach = new Pose2d(-35, -12, Math.toRadians(180));
    private Pose2d depositOnHighPole1 = new Pose2d(-23.5, -12, Math.toRadians(180));
    private Pose2d depositOnHighPole2 = new Pose2d(-25, -12, Math.toRadians(180));

    private int initialTurn = 90;

    private Pose2d parkingLeft = new Pose2d(-12, -12.5, Math.toRadians(-90));
    private Pose2d parkingMid = new Pose2d(-36, -12.5, Math.toRadians(-90));
    private Pose2d parkingRight = new Pose2d(-60, -12.5, Math.toRadians(-90));

    // Async Vars /////////////////////////////////////////////////////////////////////
    private boolean step1 = false;
    private boolean step2 = false;
    private boolean step3 = false;
    private boolean step4 = false;
    private boolean step4a = false;
    private boolean step5 = false;
    private boolean step5a = false;

    private int parking;

    private boolean isRed = true;

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

    public enum AllianceColor {
        RED, BLUE
    }

    public Auto2Dw(AllianceColor color) {
        this.color = color;
        switch (color) {
            case RED:
                isRed = true;
                break;
            case BLUE:
                isRed = false;

                initialTurn = -90;
                startPosition = new Pose2d(startPosition.getX(), -startPosition.getY(), Math.toRadians(-90));
                signalConeKnockout = new Pose2d(signalConeKnockout.getX(), -signalConeKnockout.getY(), Math.toRadians(180));
                centerOfChannelInitial = new Pose2d(centerOfChannelInitial.getX(), -centerOfChannelInitial.getY(), Math.toRadians(-90));
                depositPreLoad = new Pose2d(depositPreLoad.getX(), -depositPreLoad.getY(), Math.toRadians(180));
                approachPosition = new Pose2d(approachPosition.getX(), -approachPosition.getY(), Math.toRadians(180));
                collectConesPosition = new Pose2d(collectConesPosition.getX(), -collectConesPosition.getY(), Math.toRadians(180));
                depositOnHighPole1approach = new Pose2d(depositOnHighPole1approach.getX(), -depositOnHighPole1approach.getY(), Math.toRadians(180));
                depositOnHighPole1 = new Pose2d(depositOnHighPole1.getX(), -depositOnHighPole1.getY(), Math.toRadians(180));
                depositOnHighPole2 = new Pose2d(depositOnHighPole2.getX(), -depositOnHighPole2.getY(), Math.toRadians(180));


                parkingLeft = new Pose2d(-12, 12.5, Math.toRadians(90));
                parkingMid = new Pose2d(-36, 12.5, Math.toRadians(90));
                parkingRight = new Pose2d(-60, 12.5, Math.toRadians(90));
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
        this.stateMap = new HashMap<String, String>() {{}};
        BrainSTEMRobot robot = new BrainSTEMRobot(this.hardwareMap, this.telemetry, this.stateMap);

        liftCollectionHeights = new ArrayList();
        liftCollectionHeights.add(robot.lift.LIFT_POSITION_AUTO_CYCLE_1);
        liftCollectionHeights.add(robot.lift.LIFT_POSITION_AUTO_CYCLE_2);
        liftCollectionHeights.add(robot.lift.LIFT_POSITION_AUTO_CYCLE_3);
        liftCollectionHeights.add(robot.lift.LIFT_POSITION_AUTO_CYCLE_4);
        liftCollectionHeights.add(robot.lift.LIFT_POSITION_AUTO_CYCLE_5);
        liftCollectionHeights.add(0);



        // State Map ////////////////////////////////////////////////////////////////
        this.stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
        this.stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
        this.stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
        this.stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        this.stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);

        // Open CV //////////////////////////////////////////////////////////////////
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam-2"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        robot.grabber.close();
        while (!this.opModeIsActive() && !this.isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if(currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {

                    if (tag.id == MIDDLE) {
                        tagOfInterest = tag;
                        location = ParkingLocation.MID;
                        parking = 2;
                        tagFound = true;
                        telemetry.addData("Open CV :", "Mid");
                        telemetry.update();
                        endParking = new Pose2d(parkingMid.getX(), parkingMid.getY(), parkingMid.getHeading());
                        break;

                    } else if (tag.id == RIGHT) {
                        tagOfInterest = tag;
                        location = ParkingLocation.RIGHT;
                        parking = 3;
                        tagFound = true;
                        telemetry.addData("Open CV :", "Right");
                        telemetry.update();
                        endParking = new Pose2d(parkingRight.getX(), parkingRight.getY(), parkingRight.getHeading());
                        break;

                    } else {
                        tagOfInterest = tag;
                        location = ParkingLocation.LEFT;
                        tagFound = true;
                        parking = 1;
                        telemetry.addData("Open CV :", "Left");
                        telemetry.update();
                        endParking = new Pose2d(parkingLeft.getX(), parkingLeft.getY(), parkingLeft.getHeading());
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
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW);
        stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_GRABBER, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CONE_CYCLE, constants.STATE_NOT_STARTED);
        robot.updateSystems();

        totalTime.reset();

        TrajectorySequence deliverPreload = drive.trajectorySequenceBuilder(startPosition)
                .lineToLinearHeading(centerOfChannelInitial)
                .turn(Math.toRadians(initialTurn))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW); })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND); })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.RIGHT_POSITION); })
                .lineToLinearHeading(depositPreLoad)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> { stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS); })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW); })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> { resetLift(); })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE); })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> { stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION); })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.STACK_5); })
                .lineToLinearHeading(approachPosition)
                .lineToLinearHeading(collectConesPosition, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> { robot.grabber.close(); })
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW); })
                .lineToLinearHeading(depositOnHighPole1approach)
                .lineToLinearHeading(depositOnHighPole1, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_HIGH); })
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.LEFT_POSITION); })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND); })
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> { stateMap.put(constants.CONE_CYCLE, constants.STATE_IN_PROGRESS); })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> { resetLift(); })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> { stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION); })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> { stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE); })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> { stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.STACK_5); })
                .waitSeconds(0.25)
                .lineToLinearHeading(approachPosition)
                .lineToLinearHeading(collectConesPosition, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> { robot.grabber.close(); })
                .waitSeconds(0.25)
                .build();

        drive.followTrajectorySequenceAsync(deliverPreload);

        while (opModeIsActive()) {
            drive.update();
            robot.updateSystems();
        }



    }

    private void resetLift() {
        stateMap.put(constants.CONE_CYCLE, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_UP, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_LIFT_DOWN, constants.STATE_NOT_STARTED);
        stateMap.put(constants.CYCLE_GRABBER, constants.STATE_NOT_STARTED);
    }
}
