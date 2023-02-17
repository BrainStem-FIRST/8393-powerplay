
package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.autoSubsystems.AutoDistanceSensors;
@Disabled
@Autonomous(name="DISTANCE SENSOR TEST")
public class DistanceSensorAuto extends LinearOpMode {
    private AutoDistanceSensors autoDistanceSensors;

    @Override
    public void runOpMode() throws InterruptedException {
        while(!opModeIsActive() && !isStopRequested()){
            autoDistanceSensors = new AutoDistanceSensors(this.hardwareMap, this.telemetry);
        }

        this.waitForStart();


        while (opModeIsActive()) {
            autoDistanceSensors.outputValues();
        }

    }


}