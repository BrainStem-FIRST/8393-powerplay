package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="1 Blue Auto")
public class BlueAuto1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Auto1 auto = new Auto1(Auto1.AllianceColor.BLUE, this);
        auto.run();

    }

}