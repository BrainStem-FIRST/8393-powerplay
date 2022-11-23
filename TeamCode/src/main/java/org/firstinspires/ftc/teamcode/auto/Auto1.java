package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;

import java.util.HashMap;
import java.util.Map;

public class Auto1 extends LinearOpMode {
    private final AllianceColor color;
    private BrainSTEMRobot robot;
    private Map stateMap;



    // Locations - For Red /////////////////////////////////////////////////////////////////////
    private Pose2d startPosition = new Pose2d(36, -65, Math.toRadians(90));
    private Pose2d centerofBlueChannel = new Pose2d(36, -23.5, Math.toRadians(90));
    private Pose2d depositPreLoad = new Pose2d(36, -23.5, Math.toRadians(90));
    private Pose2d collectConesPosition = new Pose2d(64, -12.5, Math.toRadians(0));
    private Pose2d depositOnHighPole = new Pose2d(23, -12.5, Math.toRadians(0));
    ///////////////////////////////////////////////////////////////////////////////////////////

    private boolean step1 = false;
    private boolean step2 = false;
    private boolean step3 = false;
    private boolean step4 = false;
    private boolean step5 = false;

    private boolean isRed = true;

    private enum ParkingLocation {
        LEFT, MID, RIGHT
    }

    public enum AllianceColor {
        RED, BLUE
    }

    public Auto1(AllianceColor color) {
        this.color = color;

        switch (color) {
            case RED:
                    isRed = true;
                break;
            case BLUE:
                    isRed = false;

                    startPosition = new Pose2d(startPosition.getX(), -startPosition.getY(), Math.toRadians(-90));
                    centerofBlueChannel = new Pose2d(centerofBlueChannel.getX(), -centerofBlueChannel.getY(), Math.toRadians(0));
                    depositPreLoad = new Pose2d(depositPreLoad.getX(), -depositPreLoad.getY(), Math.toRadians(0));
                    collectConesPosition = new Pose2d(collectConesPosition.getX(), -collectConesPosition.getY(), Math.toRadians(0));
                    depositOnHighPole = new Pose2d(depositOnHighPole.getX(), -depositOnHighPole.getY(), Math.toRadians(0));
                break;
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runTime = new ElapsedTime();
        ElapsedTime totalTime = new ElapsedTime();
        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(this.hardwareMap);

        this.stateMap = new HashMap<String, String>() {{
        }};
        BrainSTEMRobot robot = new BrainSTEMRobot(this.hardwareMap, this.telemetry, this.stateMap);


        this.stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
        this.stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
        this.stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.APPROACH_HEIGHT);
        this.stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        this.stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);


        while (!this.opModeIsActive()) {
            runTime.reset();
            // april tag open cv here
//
            robot.updateSystems();
        }


        this.waitForStart();
        sampleMecanumDrive.setPoseEstimate(startPosition);
        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
        robot.arm.setState(robot.arm.DEFAULT_VALUE);
        robot.arm.extendHome();
        robot.grabber.runGrabber(robot.grabber.CLOSED_STATE);
        robot.grabber.actuallySettingGrabberState(robot.grabber.CLOSED_STATE);
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
        while (step2) {
            if (!robot.lift.hasLiftReachedHeight(robot.lift.LIFT_POLE_LOW)) {
                telemetry.addData("Lift: ", "running Lift");
                telemetry.update();
                robot.lift.raiseHeightTo(robot.lift.LIFT_POSITION_LOWPOLE);
                robot.lift.setState();
            } else {
                telemetry.addData("else:", "else");
                telemetry.update();
                step3 = true;
                step2 = false;
            }
        }
        sampleMecanumDrive.waitForIdle();

        //   2.5 / Deposit Pre Load /////////////////////////////////////////////////////
        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.FULL_EXTEND);
        robot.arm.extendMax();
        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.LEFT_POSITION);
        robot.turret.setState(robot.lift);
        while (runTime.seconds() < 0.65);
        runTime.reset();
        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.OPEN_STATE);
        robot.grabber.runGrabber(robot.grabber.OPEN_STATE);
        while (runTime.milliseconds() < 750);
        stateMap.put(robot.turret.SYSTEM_NAME, robot.turret.CENTER_POSITION);
        robot.turret.setState(robot.lift);
        stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
        robot.arm.extendHome();

        while (this.opModeIsActive());
        //   3   ////////////////////////////////////////////////////////////////////


        Trajectory collectConePoseTraj = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                .lineToLinearHeading(collectConesPosition)
                .build();
        sampleMecanumDrive.followTrajectoryAsync(collectConePoseTraj);

        while (step3) {
            if (!robot.lift.hasLiftReachedHeight(robot.lift.LIFT_PICKUP)){
                robot.lift.raiseHeightTo(robot.lift.LIFT_POSITION_GROUND);
                robot.lift.setState();
            } else {
                step4 = true;
                step3 = false;
            }

        }
        sampleMecanumDrive.waitForIdle();

        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
        robot.grabber.runGrabber(robot.grabber.CLOSED_STATE);


        //   4   ////////////////////////////////////////////////////////////////////


        Trajectory depositOnHighGoalTraj = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                .lineToLinearHeading(depositOnHighPole)
                .build();
        sampleMecanumDrive.followTrajectoryAsync(depositOnHighGoalTraj);

        while (step4) {
            step5 = true;
            step4 = false;
        }
        sampleMecanumDrive.waitForIdle();

        while (runTime.seconds() < 25) {


            //   5   ////////////////////////////////////////////////////////////////////


            Trajectory cycleCollectTraj = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                    .lineToLinearHeading(collectConesPosition)
                    .build();
            sampleMecanumDrive.followTrajectoryAsync(cycleCollectTraj);

            while (step5) {
                robot.lift.raiseHeightTo(robot.lift.LIFT_POSITION_GROUND);
                stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_GROUND);
                robot.lift.setState();
            }
            sampleMecanumDrive.waitForIdle();
        }
    }
}
