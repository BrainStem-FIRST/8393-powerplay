package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LEDLights {
    private Telemetry telemetry;

    private DigitalChannel leftRedLED;
    private DigitalChannel leftGreenLED;

    private DigitalChannel rightRedLED;
    private DigitalChannel rightGreenLED;

    public LEDLights (HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftRedLED = hwMap.get(DigitalChannel.class, "Red1");
        leftGreenLED = hwMap.get(DigitalChannel.class, "Green1");

        rightRedLED = hwMap.get(DigitalChannel.class, "Red2");
        rightGreenLED = hwMap.get(DigitalChannel.class, "Green2");

        leftRedLED.setMode(DigitalChannel.Mode.OUTPUT);
        leftGreenLED.setMode(DigitalChannel.Mode.OUTPUT);
        rightRedLED.setMode(DigitalChannel.Mode.OUTPUT);
        rightGreenLED.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void setBothLEDGreen(){
        leftGreenLED.setState(true);
        leftRedLED.setState(false);

        rightGreenLED.setState(true);
        rightRedLED.setState(false);
    }

    public void setBothLEDRed(){
        leftGreenLED.setState(false);
        leftRedLED.setState(true);

        rightGreenLED.setState(false);
        rightRedLED.setState(true);
    }

    public void setBothLEDAmber(){
        leftGreenLED.setState(true);
        leftRedLED.setState(true);

        rightGreenLED.setState(true);
        rightRedLED.setState(true);
    }

    public void setLefLEDGreen(){
        leftGreenLED.setState(true);
        leftRedLED.setState(false);
    }

    public void setLeftLEDRed(){
        leftGreenLED.setState(false);
        leftRedLED.setState(true);
    }

    public void setLeftLEDAmber(){
        leftGreenLED.setState(true);
        leftRedLED.setState(true);
    }

    public void setRightLEDGreen(){
        rightGreenLED.setState(true);
        rightRedLED.setState(false);
    }

    public void setRightLEDRed(){
        rightGreenLED.setState(false);
        rightRedLED.setState(true);
    }

    public void setRightLEDAmber(){
        rightGreenLED.setState(true);
        rightRedLED.setState(true);
    }


}
