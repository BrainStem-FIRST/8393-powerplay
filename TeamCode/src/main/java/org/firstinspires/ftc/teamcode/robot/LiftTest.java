package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;



@TeleOp (name = "Lift Encoder Getter")
public class LiftTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {


        HashMap<String, String> stateMap = new HashMap<String, String>() {{
        }};
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, stateMap, false);

        int liftPose;



        while (!isStopRequested()){


            liftPose = robot.lift.getAvgLiftPosition();

            telemetry.addData("Lift Current Pose ::" ,liftPose );
            telemetry.update();
        }
    }



}
