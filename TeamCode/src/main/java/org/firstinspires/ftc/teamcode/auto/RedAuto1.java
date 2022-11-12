package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="1 Red Auto")
public class RedAuto1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Auto1 auto = new Auto1(Auto1.AllianceColor.RED, this);
        auto.run();

    }

}
