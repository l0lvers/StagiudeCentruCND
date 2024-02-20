package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Logging {
    private static Logging instance =null;

    private MultipleTelemetry multTelemetry;
    private Logging(){

    }
    public void init(Telemetry telemetry){
        multTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    public static Logging getInstance(){
        if(instance ==null){
            instance = new Logging();
        }
        return instance;
    }
    public MultipleTelemetry getTelemetry(){
        return multTelemetry;
    }
}
