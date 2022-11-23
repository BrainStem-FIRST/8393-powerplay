package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Auto2 {
    private final Auto2.AllianceColor color;
    private final LinearOpMode opMode;
    private Object robot;
    private HardwareMap hardwareMap;


    // Locations - For Red /////////////////////////////////////////////////////////////////////
    private Pose2d startPosition = new Pose2d(-36, -65, Math.toRadians(90));
    private Pose2d alignToCenterOfBlueChannel = new Pose2d(-36, -13, Math.toRadians(180));
    private Pose2d depositPreLoad = new Pose2d(-48, -12, Math.toRadians(180));
    private Pose2d collectConesPosition = new Pose2d(-64, -12.5, Math.toRadians(180));
    private Pose2d depositOnHighPole = new Pose2d(-23,-12.5, Math.toRadians(180));

    ///////////////////////////////////////////////////////////////////////////////////////////




    private enum LiftHeight {
        LOW, MID, TOP
    }

    public enum AllianceColor {
        RED, BLUE
    }

    public Auto2(Auto2.AllianceColor color, LinearOpMode opMode) {
        this.color = color;
        this.opMode = opMode;
        Telemetry telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        switch (color) {
            case RED:

                break;
            case BLUE:
                startPosition = new Pose2d(startPosition.getX(), -startPosition.getY(), Math.toRadians(-90));
                alignToCenterOfBlueChannel = new Pose2d(alignToCenterOfBlueChannel.getX(), -alignToCenterOfBlueChannel.getY(), Math.toRadians(180));
                depositPreLoad = new Pose2d(depositPreLoad.getX(), -depositPreLoad.getY(), Math.toRadians(180));
                collectConesPosition = new Pose2d(collectConesPosition.getX(), -collectConesPosition.getY(), Math.toRadians(180));
                depositOnHighPole = new Pose2d(depositOnHighPole.getX(), -depositOnHighPole.getY(), Math.toRadians(180));
                break;
        }


    }

    public void run() throws InterruptedException {
        ElapsedTime runTime = new ElapsedTime();
        ElapsedTime totalTime = new ElapsedTime();
    }




}