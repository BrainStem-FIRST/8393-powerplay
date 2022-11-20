package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import java.util.HashMap;
import java.util.Map;

public class Auto1 {
    private final AllianceColor color;
    private final LinearOpMode opMode;
    private Object robot;


    // Locations - For Red /////////////////////////////////////////////////////////////////////
    private Pose2d startPosition = new Pose2d(36, -65, Math.toRadians(90));
    private Pose2d alignToCenterOfBlueChannel = new Pose2d(36, -13, Math.toRadians(0));
    private Pose2d depositPreLoad = new Pose2d(48, -12, Math.toRadians(0));
    private Pose2d collectConesPosition = new Pose2d(64, -12.5, Math.toRadians(0));
    private Pose2d depositOnHighPole = new Pose2d(23,-12.5, Math.toRadians(0));
    ///////////////////////////////////////////////////////////////////////////////////////////

    private enum LiftHeight {
        LOW, MID, TOP
    }

    public enum AllianceColor {
        RED, BLUE
    }

    public Auto1(AllianceColor color, LinearOpMode opMode) {
        this.color = color;
        this.opMode = opMode;
        Telemetry telemetry = opMode.telemetry;

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

    public void run() throws InterruptedException {
        ElapsedTime runTime = new ElapsedTime();
        ElapsedTime totalTime = new ElapsedTime();
//        FIXME PLZ
//        Map<String, String> stateMap = new HashMap<String, String>() {{ }};
//        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, stateMap);
        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(opMode.hardwareMap);
        


        while (!opMode.opModeIsActive()) {
            runTime.reset();
            // april tag open cv here
        }

        opMode.waitForStart();
        sampleMecanumDrive.setPoseEstimate(startPosition);
        totalTime.reset();


        //   1   /////////////////////////////////////////////////////////////////////


        Trajectory alignCenterOfBlueAutoChannel = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                .lineToLinearHeading(alignToCenterOfBlueChannel)
                .build();
        sampleMecanumDrive.followTrajectory(alignCenterOfBlueAutoChannel);

        sampleMecanumDrive.waitForIdle();

        //   2   ////////////////////////////////////////////////////////////////////

        Trajectory depositPreLoadTraj = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                .lineToLinearHeading(depositPreLoad)
                .build();
        sampleMecanumDrive.followTrajectory(depositPreLoadTraj);

        sampleMecanumDrive.waitForIdle();

        //   3   ////////////////////////////////////////////////////////////////////


        Trajectory collectConePoseTraj = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                .lineToLinearHeading(collectConesPosition)
                .build();
        sampleMecanumDrive.followTrajectory(collectConePoseTraj);

        sampleMecanumDrive.waitForIdle();

        while (runTime.seconds() < 25) {

            //   4   ////////////////////////////////////////////////////////////////////


            Trajectory depositOnHighGoalTraj = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                    .lineToLinearHeading(depositOnHighPole)
                    .build();
            sampleMecanumDrive.followTrajectory(depositOnHighGoalTraj);

            sampleMecanumDrive.waitForIdle();

            //   5   ////////////////////////////////////////////////////////////////////


            Trajectory cycleCollectTraj = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                    .lineToLinearHeading(collectConesPosition)
                    .build();
            sampleMecanumDrive.followTrajectory(cycleCollectTraj);

            sampleMecanumDrive.waitForIdle();
        }


    }
}
