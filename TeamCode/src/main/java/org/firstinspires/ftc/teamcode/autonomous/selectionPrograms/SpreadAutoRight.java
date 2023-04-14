package org.firstinspires.ftc.teamcode.autonomous.selectionPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.SpreadAuto;

@Autonomous(name="3 - Right Spread Auto")
public class SpreadAutoRight extends SpreadAuto {
    public SpreadAutoRight() {super(AutoOrientation.RIGHT);}
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }
}

