package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RobotUtils {
    private SampleMecanumDrive drive;
    private DcMotor uppies1; //=slider1
    private DcMotor uppies2;//=slider2
    private DcMotor intake1;
    private DcMotor intake2;
    private Servo cuva1;
    private Servo cuva2;
    private Servo bratCuva1;
    private Servo bratCuva2;
    private Servo opritoare;

    int uppiepos = 2000;

    public RobotUtils(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);

        uppies1 = hardwareMap.get(DcMotor.class, "uppies1");
        uppies2 = hardwareMap.get(DcMotor.class, "uppies2");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        cuva1 = hardwareMap.get(Servo.class, "cuva1");
        cuva2 = hardwareMap.get(Servo.class, "cuva2");
        opritoare = hardwareMap.get(Servo.class, "opritoare");
        bratCuva1 = hardwareMap.get(Servo.class, "bratCuva1");
        bratCuva2 = hardwareMap.get(Servo.class, "bratCuva2");
        uppies1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        uppies2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        uppies1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uppies2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void intake() {
        intake1.setPower(0.5);
        intake2.setPower(0.5);
    }
    public void setPosCuva0() {
        cuva1.setPosition(0);
        cuva2.setPosition(0);
    }
    public void setPosOpritoare() {
        opritoare.setPosition(0);
    }
    public void downies(){
        uppies1.setTargetPosition(10);
        uppies2.setTargetPosition(10);
        uppies1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        uppies2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void uppies(){
        uppies1.setTargetPosition(uppiepos);
        uppies2.setTargetPosition(uppiepos);
        uppies1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        uppies2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setBratPos0(){
        bratCuva1.setPosition(0);
        bratCuva2.setPosition(0);
    }

}
