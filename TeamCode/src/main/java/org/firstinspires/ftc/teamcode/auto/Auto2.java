package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.imagecv.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.HashMap;
import java.util.Map;

public class Auto2 extends LinearOpMode {
    private final AllianceColor color;
    private Map stateMap;


    // Locations - For Red /////////////////////////////////////////////////////////////////////
    private Pose2d startPosition = new Pose2d(-36, -65, Math.toRadians(90));
    private Pose2d centerofBlueChannel = new Pose2d(-36, -12.5, Math.toRadians(180));
    private Pose2d depositPreLoad = new Pose2d(-48, -12.5, Math.toRadians(180));
    private Pose2d collectConesPosition = new Pose2d(-64, -12.5, Math.toRadians(180));
    private Pose2d depositOnHighPole = new Pose2d(-23, -12.5, Math.toRadians(180));


    // Async Vars /////////////////////////////////////////////////////////////////////
    private boolean step1 = false;
    private boolean step2 = false;
    private boolean step3 = false;
    private boolean step4 = false;
    private boolean step5 = false;
    private boolean step5a = false;

    private boolean isRed = true;

    // Open CV Stuff
    public OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    public int Ending_Location = 1;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 1000;
    double cx = 100;
    double cy = 221.506;

    // UNITS ARE METERS
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

    public Auto2(AllianceColor color) {
        this.color = color;
        switch (color) {
            case RED:
                isRed = true;
                break;
            case BLUE:
                isRed = false;

                startPosition = new Pose2d(startPosition.getX(), -startPosition.getY(), Math.toRadians(-90));
                centerofBlueChannel = new Pose2d(centerofBlueChannel.getX(), -centerofBlueChannel.getY(), Math.toRadians(180));
                depositPreLoad = new Pose2d(depositPreLoad.getX(), -depositPreLoad.getY(), Math.toRadians(180));
                collectConesPosition = new Pose2d(collectConesPosition.getX(), -collectConesPosition.getY(), Math.toRadians(180));
                depositOnHighPole = new Pose2d(depositOnHighPole.getX(), -depositOnHighPole.getY(), Math.toRadians(180));
                break;
        }

    }


    @Override
    public void runOpMode() throws InterruptedException {

        // Timers ////////////////////////////////////////////////////////////////////////
        ElapsedTime runTime = new ElapsedTime();
        ElapsedTime totalTime = new ElapsedTime();

        // Hardwhare ///////////////////////////////////////////////////////////////////
        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(this.hardwareMap);
        this.stateMap = new HashMap<String, String>() {{}};
        BrainSTEMRobot robot = new BrainSTEMRobot(this.hardwareMap, this.telemetry, this.stateMap);

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

        while (!this.opModeIsActive()) {

        }


        this.waitForStart();
        sampleMecanumDrive.setPoseEstimate(startPosition);
        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
        robot.arm.setState(robot.arm.DEFAULT_VALUE);
        robot.arm.extendHome();
//        robot.grabber.runGrabber(robot.grabber.CLOSED_STATE);
//        robot.grabber.actuallySettingGrabberState(robot.grabber.CLOSED_STATE);
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
        robot.lift.setState();

        totalTime.reset();
        //   1   /////////////////////////////////////////////////////////////////////;
        this.step1 = true;
        telemetry.addData("Trajectory: ", "setting trajectory");
        telemetry.update();
        Trajectory alignCenterOfBlueAutoChannel = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                .lineToLinearHeading(centerofBlueChannel)
                .build();
        sampleMecanumDrive.followTrajectoryAsync(alignCenterOfBlueAutoChannel);

        while (step1) {
            telemetry.addData("StateMap: ", "Driving Test");
            telemetry.update();
            if ((false)) {

            } else {
                telemetry.addData("else:", "else");
                telemetry.update();
                step2 = true;
                step1 = false;
            }
        }
        sampleMecanumDrive.waitForIdle();

        //   2   ////////////////////////////////////////////////////////////////////

        Trajectory depositPreLoadTraj = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                .lineToLinearHeading(depositPreLoad)
                .build();
        sampleMecanumDrive.followTrajectoryAsync(depositPreLoadTraj);


        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW);
        runTime.reset();
        while (step2) {
            if (runTime.milliseconds() < 750) {
                telemetry.addData("Lift: ", "running Lift");
                telemetry.update();
                //robot.lift.raiseHeightTo(robot.lift.LIFT_POSITION_LOWPOLE);
                //robot.lift.setState();
            } else {
                telemetry.addData("else:", "else");
                telemetry.update();
                step3 = true;
                step2 = false;
            }
        }
        sampleMecanumDrive.waitForIdle();

        //   2.5 / Deposit Pre Load /////////////////////////////////////////////////////
//        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND);
//        robot.arm.extendMax();
//        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.RIGHT_POSITION);
//        robot.turret.setState(robot.lift);
//        while (runTime.seconds() < 0.65) ;
//        runTime.reset();
//        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
//        robot.grabber.runGrabber(robot.grabber.OPEN_STATE);
//        while (runTime.milliseconds() < 750) ;
//        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        //robot.turret.setState(robot.lift);
        //stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
        //robot.arm.extendHome();


        //   3   ////////////////////////////////////////////////////////////////////

        telemetry.addData("Trajcectory ::", "Collect COne");
        telemetry.update();

        Trajectory collectConePoseTraj = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                .lineToLinearHeading(collectConesPosition)
                .build();
        sampleMecanumDrive.followTrajectoryAsync(collectConePoseTraj);

        //stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_PICKUP);
        runTime.reset();
        while (step3) {
            if (runTime.seconds() < 1.3) {
                telemetry.addData("While Loop ::", "Step 3");
                telemetry.update();
//                robot.lift.raiseHeightTo(robot.lift.LIFT_POSITION_PICKUP);
//                robot.lift.setState();
            } else {
                step4 = true;
                step3 = false;
            }

        }
        sampleMecanumDrive.waitForIdle();

//        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
//        robot.grabber.runGrabber(robot.grabber.CLOSED_STATE);


        //   4   ////////////////////////////////////////////////////////////////////


        Trajectory depositOnHighGoalTraj = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                .lineToLinearHeading(depositOnHighPole)
                .build();
        sampleMecanumDrive.followTrajectoryAsync(depositOnHighGoalTraj);
        runTime.reset();
        //stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_HIGH);
        while (step4) {
            if (runTime.seconds() < 1) {
//                robot.lift.raiseHeightTo(robot.lift.LIFT_POSITION_HIGHPOLE);
//                robot.lift.setState();
            } else {
                step5 = true;
                step4 = false;
            }

        }
        sampleMecanumDrive.waitForIdle();

//        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND);
//        robot.arm.extendMax();
//        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.RIGHT_POSITION);
//        robot.turret.setState(robot.lift);
        while (runTime.seconds() < 0.65) ;
        runTime.reset();
//        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
//        robot.grabber.runGrabber(robot.grabber.OPEN_STATE);
        while (runTime.milliseconds() < 750) ;
//        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
//        robot.turret.setState(robot.lift);
//        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
//        robot.arm.extendHome();


        while (true) {


            //   5   ////////////////////////////////////////////////////////////////////

            telemetry.addData("traj", "5");
            telemetry.update();

            Trajectory cycleCollectTraj = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                    .lineToLinearHeading(collectConesPosition)
                    .build();
            sampleMecanumDrive.followTrajectoryAsync(cycleCollectTraj);

            runTime.reset();
            stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
            while (step5) {
                if (runTime.seconds() < 0.5) {
//                    robot.lift.raiseHeightTo(robot.lift.LIFT_POSITION_GROUND);
//                    robot.lift.setState();
                    telemetry.addData("while loop", "step 5");
                    telemetry.update();
                } else {
                    step5 = false;
                    step5a = true;
                }
            }


            sampleMecanumDrive.waitForIdle();

//            stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND);
//            robot.arm.extendMax();
//            stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.RIGHT_POSITION);
//            robot.turret.setState(robot.lift);
//            while (runTime.seconds() < 0.65);
//            runTime.reset();
//            stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
//            robot.grabber.runGrabber(robot.grabber.OPEN_STATE);
//            while (runTime.milliseconds() < 750);
//            stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
//            robot.turret.setState(robot.lift);
//            stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
//            robot.arm.extendHome();


            // 6? (technically) ///////////////////////////////////////
            telemetry.addData("traj", "6");
            telemetry.update();

            Trajectory depositConeCollected = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                    .lineToLinearHeading(depositOnHighPole)
                    .build();
            sampleMecanumDrive.followTrajectoryAsync(depositConeCollected);

            //stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_HIGH);
            runTime.reset();
            while (step5a) {
                if (runTime.seconds() < 0.5) {
//                    robot.lift.raiseHeightTo(robot.lift.LIFT_POSITION_HIGHPOLE);
//                    robot.lift.setState();
                } else {
                    step5a = false;
                    step5 = true;
                }
            }

            sampleMecanumDrive.waitForIdle();


            telemetry.addData("Cycle Loop :", "end of loop");
            telemetry.update();

        }
    }
}
