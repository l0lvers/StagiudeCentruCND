package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RobotUtils {
    private SampleMecanumDrive drive;
    private DcMotor uppies1;
    private DcMotor uppies2;
    private DcMotor intake1;
    private DcMotor intake2;
    private Servo cuva1;
    private Servo cuva2;
    private Servo opritoare;

    public RobotUtils(HardwareMap hardwareMap)
    {
        drive = new SampleMecanumDrive(hardwareMap);

        uppies1 = hardwareMap.get(DcMotor.class, "uppies1");
        uppies2 = hardwareMap.get(DcMotor.class, "uppies2");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        cuva1 = hardwareMap.get(Servo.class, "cuva1");
        cuva2 = hardwareMap.get(Servo.class, "cuva2");
        opritoare = hardwareMap.get(Servo.class, "opritoare");
    }
    public void intake(){
        intake1.setPower(0.7);
        intake2.setPower(0.7);
    }
    public void setPosCuva0()
    {
        cuva1.setPosition(0);
        cuva2.setPosition(0);
    }
    public void setPosOpritoare()
    {
        opritoare.setPosition(0);
    }
}
