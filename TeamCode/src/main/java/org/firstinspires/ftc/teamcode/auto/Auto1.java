package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    private Pose2d alignToCenterOfBlueChannel = new Pose2d(36, -13, Math.toRadians(0));
    private Pose2d depositPreLoad = new Pose2d(48, -12, Math.toRadians(0));
    private Pose2d collectConesPosition = new Pose2d(64, -12.5, Math.toRadians(0));
    private Pose2d depositOnHighPole = new Pose2d(23, -12.5, Math.toRadians(0));

    private boolean step1 = false;
    private boolean step2 = false;
    private boolean step3 = false;
    private boolean step4 = false;
    private boolean step5 = false;


    ///////////////////////////////////////////////////////////////////////////////////////////

    private enum LiftHeight {
        LOW, MID, TOP
    }

    public enum AllianceColor {
        RED, BLUE
    }

    public Auto1(AllianceColor color) {
        this.color = color;

        switch (color) {
            case RED:

                break;
            case BLUE:
                startPosition = new Pose2d(startPosition.getX(), -startPosition.getY(), Math.toRadians(-90));
                alignToCenterOfBlueChannel = new Pose2d(alignToCenterOfBlueChannel.getX(), -alignToCenterOfBlueChannel.getY(), Math.toRadians(0));
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
//            stateMap.put(robot.arm.SYSTEM_NAME, robot.arm.DEFAULT_VALUE);
//            stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
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
//        stateMap.put(robot.grabber.SYSTEM_NAME, robot.grabber.CLOSED_STATE);
//        robot.updateSystems();
        totalTime.reset();
        this.step1 = true;
        telemetry.addData("Trajectory: ", "setting trajectory");
        telemetry.update();
        Trajectory alignCenterOfBlueAutoChannel = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                .lineToLinearHeading(alignToCenterOfBlueChannel)
                .build();
        sampleMecanumDrive.followTrajectoryAsync(alignCenterOfBlueAutoChannel);
        while (step1) {
            //   1   /////////////////////////////////////////////////////////////////////;
            telemetry.addData("StateMap: ", "lift low");
            telemetry.update();
            stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_POLE_LOW);
            if (!robot.lift.hasLiftReachedHeight(robot.lift.LIFT_POLE_LOW)) {
                telemetry.addData("Lift: ", "running Lift");
                telemetry.update();
                robot.lift.raiseHeightTo(robot.lift.LIFT_POSITION_LOWPOLE);
                robot.lift.setState();
            } else {
                telemetry.addData("else:", "else");
                telemetry.update();
                step2 = true;
                step1 = false;
            }
        }
        sampleMecanumDrive.waitForIdle();
        /*
        //   2   ////////////////////////////////////////////////////////////////////

        Trajectory depositPreLoadTraj = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                .lineToLinearHeading(depositPreLoad)
                .build();
        sampleMecanumDrive.followTrajectoryAsync(depositPreLoadTraj);

        while (step2) {
            step3 = true;
            step2 = false;
        }
        sampleMecanumDrive.waitForIdle();

        //   3   ////////////////////////////////////////////////////////////////////


        Trajectory collectConePoseTraj = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                .lineToLinearHeading(collectConesPosition)
                .build();
        sampleMecanumDrive.followTrajectoryAsync(collectConePoseTraj);

        while (step3) {
            step4 = true;
            step3 = false;
        }
        sampleMecanumDrive.waitForIdle();

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
        }*/
    }
}
