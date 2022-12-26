
package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.teamcode.robot.subsystems.DistanceSensors;
@Autonomous(name="DISTANCE SENSOR TEST")
public class DistanceSensorAuto extends LinearOpMode {
    private DistanceSensors distanceSensors;

    @Override
    public void runOpMode() throws InterruptedException {
        while(!opModeIsActive() && !isStopRequested()){
            distanceSensors = new DistanceSensors(this.hardwareMap, this.telemetry);
        }

        this.waitForStart();


        while (opModeIsActive()) {
            distanceSensors.outputValues();
        }

    }


}