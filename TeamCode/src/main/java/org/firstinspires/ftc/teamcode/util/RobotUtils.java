package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class RobotUtils {

    //------------------------------OBIECTE-------------------------------------
    public SampleMecanumDrive drive;
    public DcMotorEx sliderLeft;
    public DcMotorEx sliderRight;
    public DcMotorEx intakeLeft;
    public DcMotorEx intakeRight;
    //public Servo cuvaLeft;
    //public Servo cuvaRight;
    public ServoImplEx bratCuvaLeft;
    public ServoImplEx bratCuvaRight;
    public Servo opritoare;
    public Servo droneLauncher;
    //-----------------------------VARIABILE------------------------

    public static int sliderInitPos=-3;
    public static int sliderLowPos=-1200;
    public static int sliderMidPos=-1750;
    public static int sliderHighPos=-2250;
    public static double sliderPow=0.7;
    public static double intakePow=1;
    public static double cuvaScorePos = 0;
    public static double bCuvaScorePos = 0.7;
    public static double cuvaInitPos = 0;
    public static double bCuvaInitPos=0.868;

    public static double opritoareOpenPos=0.3;
    public static double opritoateOutPos = -0.8;
    public static double opritoareClosePos=0.5
            ;
    public static double dronaLaunchPos=0.02;
    public static double dronaInitPos=0.17;


    public RobotUtils(HardwareMap hardwareMap) {
        //gasire elemente din cod in viata reala
        drive = new SampleMecanumDrive(hardwareMap);

        sliderLeft = hardwareMap.get(DcMotorEx.class, "sliderLeft");
        sliderRight = hardwareMap.get(DcMotorEx.class, "sliderRight");
        intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
        //cuvaLeft = hardwareMap.get(Servo.class, "cuvaLeft");
        //cuvaRight = hardwareMap.get(Servo.class, "cuvaRight");
        opritoare = hardwareMap.get(Servo.class, "opritoare");
        bratCuvaLeft = hardwareMap.get(ServoImplEx.class, "bratCuvaLeft");
        bratCuvaRight = hardwareMap.get(ServoImplEx.class, "bratCuvaRight");
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");

        bratCuvaLeft.setPwmEnable();
        bratCuvaRight.setPwmEnable();

        bratCuvaRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        bratCuvaLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));

        sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    //----------------------------SLIDERE---------------------------
    public void SetSliderPos(int pos) //am pus o sa fie mai clean codu
    {
        sliderLeft.setTargetPosition(pos);
        sliderRight.setTargetPosition(-pos);
    }
    public void PutSliderToPosition(int position, double power) //generalizare pentru pozitia sliderelor
    {
        double absPower = Math.abs(power);

        sliderLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sliderRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        sliderLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sliderRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        int curentPosition = sliderRight.getCurrentPosition();

        SetSliderPos(position);

        if (curentPosition > position){
            sliderLeft.setPower(-absPower);
            sliderRight.setPower(absPower);
        }
        else if(curentPosition < position){
            sliderLeft.setPower(absPower);
            sliderRight.setPower(-absPower);
        }//explicatie pt prostii ca mine la ifuri si elseuri:
         //daca positia curenta e mai mare decat cea dorita mere mai jos altfel mere mai sus
         //am vzt asta la adi si mi s-a parut genial asa ca dc nu
    }
    public void PutSlidersInit()//pune sliderele la pozitia initiala, de intake
    {
        PutSliderToPosition(sliderInitPos,sliderPow);
    }
    public void PutSliderLow() //pune sliderele la pozitie joasa
    {
        PutSliderToPosition(sliderLowPos, sliderPow);
    }

    public void PutSliderMid() //pune sliderele la pozitie medie
    {
        PutSliderToPosition(sliderMidPos, sliderPow);
    }
    public void PutSliderHigh() //pune sliderele la pozitie mare
    {
        PutSliderToPosition(sliderHighPos, sliderPow);
    }

    //---------------------------------CUVA SI SCORING-----------------------------

    public void PosCuvaInit() //pune cuva in pos initiala, aproap paralela cu terenu
    {
        //cuvaLeft.setPosition(cuvaInitPos);
        //cuvaRight.setPosition(cuvaInitPos);
        bratCuvaLeft.setPosition(bCuvaInitPos);
        bratCuvaRight.setPosition(bCuvaInitPos);
    }

    public void PosCuvaScore() //pune cuva in pozitia de scoring, aproape paralela cu tabla
    {
        //cuvaLeft.setPosition(cuvaScorePos);
        //cuvaRight.setPosition(cuvaScorePos);
        bratCuvaLeft.setPosition(bCuvaScorePos);
        bratCuvaRight.setPosition(bCuvaScorePos);
    }
    public void OpritoareClose() //inchide opritoarea cuvei sa nu pice pixeli
    {
        opritoare.setPosition(opritoareClosePos);
    }
    public void OpritoareOpen() //deschide opritoarea cuvei ca sa ia pixeli si sa scoreze
    {
        opritoare.setPosition(opritoareOpenPos);
    }
    public void opritoateOut()
    {
        opritoare.setPosition(opritoateOutPos);
    }

    //-----------------------------------INTAKE------------------------------------
    public void IntakeOn() //porneste intakeu
    {
        intakeLeft.setPower(-intakePow);
        intakeRight.setPower(intakePow);
    }
    public void IntakeReverse() //scuipa in caz de orice
    {
        intakeLeft.setPower(intakePow);
        intakeRight.setPower(-intakePow);
    }
    public void IntakeStop() // opreste intakeu
    {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }

    //--------------------------------DRONA------------------------------------
    public void DroneLaunch() //Lanseaza avionu de hartie
    {
        droneLauncher.setPosition(dronaLaunchPos);
    }
    public void DroneInit() ////aduce servou la pozitia initiala in caz de orice
    {
        droneLauncher.setPosition(dronaInitPos);
    }
}
