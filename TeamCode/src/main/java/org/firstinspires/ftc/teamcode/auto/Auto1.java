package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;

public class Auto1 {
    private final AllianceColor color;
    private final LinearOpMode opMode;
    private Object BrainSTEMRobot;

    private static double timeNeededToPark = 1;

    // Locations - init for red
    private Pose2d startPosition = new Pose2d(35, -64.5, Math.toRadians(-90));
    private Pose2d alignToCenterOfBlueChannel = new Pose2d(35, -12, Math.toRadians(0));
    private Pose2d depositConePosition = new Pose2d(12, -18, Math.toRadians(0));
    private Pose2d collectConesPosition = new Pose2d(60, -12, Math.toRadians(0));

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
                depositConePosition = new Pose2d(depositConePosition.getX(), -depositConePosition.getY(), Math.toRadians(0));
                collectConesPosition = new Pose2d(collectConesPosition.getX(), -collectConesPosition.getY(), Math.toRadians(0));
                break;
        }
    }

    public void run() throws InterruptedException {
        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(opMode.hardwareMap);
        ElapsedTime runTime = new ElapsedTime();
        ElapsedTime totalTime = new ElapsedTime();
        Telemetry telemetry;


        while (!opMode.opModeIsActive()) {
            runTime.reset();
            // april tag open cv here

        }

    }
}