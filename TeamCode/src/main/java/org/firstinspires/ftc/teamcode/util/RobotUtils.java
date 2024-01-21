package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RobotUtils {

    //------------------------------OBIECTE-------------------------------------
    private SampleMecanumDrive drive;
    private DcMotor sliderLeft; //=slider1
    private DcMotor sliderRight;//=slider2
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private Servo cuvaLeft;
    private Servo cuvaRight;
    private Servo bratCuvaLeft;
    private Servo bratCuvaRight;
    private Servo opritoare;
    private Servo droneLauncher;
    //-----------------------------VARIABILE------------------------

    int sliderInitPos=0;
    int sliderLowPos=0;
    int sliderMidPos=0;
    int sliderHighPos=0;
    double sliderPow=0;
    double intakePow=0;
    double cuvaScorePos = 0;
    double bCuvaScorePos = 0;
    double cuvaInitPos = 0;
    double bCuvaInitPos=0;
    double opritoareOpenPos=0;
    double opritoareClosePos=0;

    double dronaLaunchPos=0;
    double dronaInitPos=0;


    public RobotUtils(HardwareMap hardwareMap) {
        //gasire elemente din cod in viata reala
        drive = new SampleMecanumDrive(hardwareMap);

        sliderLeft = hardwareMap.get(DcMotor.class, "sliderLeft");
        sliderRight = hardwareMap.get(DcMotor.class, "sliderRight");
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        cuvaLeft = hardwareMap.get(Servo.class, "cuvaLeft");
        cuvaRight = hardwareMap.get(Servo.class, "cuvaRight");
        opritoare = hardwareMap.get(Servo.class, "opritoare");
        bratCuvaLeft = hardwareMap.get(Servo.class, "bratCuvaLeft");
        bratCuvaRight = hardwareMap.get(Servo.class, "bratCuvaRight");
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
    }

    //----------------------------SLIDERE---------------------------
    public void SetSliderPos(int pos) //am pus o sa fie mai clean codu
    {
        sliderLeft.setTargetPosition(pos);
        sliderRight.setTargetPosition(-pos);
    }
    public void PutSliderToPosition(int position, double power) //generalizare pentru pozitia sliderelor
    {
        sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int curentPosition = sliderRight.getCurrentPosition();

        if (curentPosition > position){
            sliderLeft.setPower(power);
            sliderRight.setPower(-power);
        }
        else if(curentPosition < position){
            sliderLeft.setPower(-power);
            sliderRight.setPower(power);
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
        cuvaLeft.setPosition(cuvaInitPos);
        cuvaRight.setPosition(cuvaInitPos);
        bratCuvaLeft.setPosition(bCuvaInitPos);
        bratCuvaRight.setPosition(bCuvaInitPos);
    }

    public void PosCuvaScore() //pune cuva in pozitia de scoring, aproape paralela cu tabla
    {
        cuvaLeft.setPosition(cuvaScorePos);
        cuvaRight.setPosition(cuvaScorePos);
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

    //-----------------------------------INTAKE------------------------------------
    public void IntakeOn() //porneste intakeu
    {
        intakeLeft.setPower(intakePow);
        intakeRight.setPower(-intakePow);
    }
    public void IntakeReverse() //scuipa in caz de orice
    {
        intakeLeft.setPower(intakePow);
        intakeRight.setPower(intakePow);
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
