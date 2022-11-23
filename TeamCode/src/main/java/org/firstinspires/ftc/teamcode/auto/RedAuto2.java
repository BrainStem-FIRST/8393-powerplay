package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="2 Red Auto")
public class RedAuto2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Auto2 auto = new Auto2(Auto2.AllianceColor.RED, this);
        auto.run();

    }

}