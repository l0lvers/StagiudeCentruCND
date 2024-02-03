package org.firstinspires.ftc.teamcode.drive.DecebalTech.teleop;

import android.transition.Slide;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.IncludedFirmwareFileInfo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RobotUtils;


@TeleOp(name="DriveMatei", group="Linear Opmode")
@Config


public class DriveMatei extends LinearOpMode {


    enum ChasisState{
        DRIVE,
        TURBO,
        PRECISION
    }
    enum SliderState{
        AUTO,
        MANUAL
    }

    private RobotUtils robot;
    private ChasisState chasisState = ChasisState.DRIVE;
    private SliderState sliderState = SliderState.MANUAL;
    private SampleMecanumDrive drive;
    private double loopTime=0;
    private boolean sliderLow = false;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        robot = new RobotUtils(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        robot.posCuvaInit();
        robot.putSliderInit();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            switch (chasisState){
                case DRIVE:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y/1.5,
                                    -gamepad1.left_stick_x/1.5,
                                    -gamepad1.right_stick_x/1.5
                            )
                    );
                    if (gamepad1.right_trigger>0.3) {
                        chasisState = ChasisState.TURBO;
                    }
                    if (gamepad1.left_trigger>0.3) {
                        chasisState = ChasisState.PRECISION;
                    }
                    break;
                case TURBO:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );
                    if (gamepad1.right_trigger==0) {
                        chasisState = ChasisState.DRIVE;
                    }
                    break;
                case PRECISION:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y/3,
                                    -gamepad1.left_stick_x/3,
                                    -gamepad1.right_stick_x/3
                            )
                    );
                    if (gamepad1.left_trigger==0) {
                        chasisState = ChasisState.DRIVE;
                    }
            }

            if(gamepad1.dpad_up) robot.DroneLaunch();
            if(gamepad1.dpad_down) robot.DroneInit();
            if(gamepad1.square) robot.IntakeReverse();
            if(gamepad1.circle) {robot.IntakeStop(); robot.outtake_cuva_off(); robot.posCuvaInit();}
            if(gamepad1.cross) {robot.IntakeOn(); robot.outtake_cuva_in(); robot.IntakeCuva();}

            switch (sliderState){
                case AUTO:
                    if(gamepad2.dpad_down)
                    {
                        robot.putSliderInit();
                        sliderLow = false;
                    }
                    if(gamepad2.dpad_up)
                    {
                        robot.putSliderHigh();
                        sliderLow = false;
                    }
                    if(gamepad2.dpad_left)
                    {
                        robot.putSliderMid();
                        sliderLow = false;
                    }
                    if(gamepad2.dpad_right)
                    {
                        robot.putSliderLow();
                        sliderLow = true;
                    }
                    if(gamepad2.right_bumper){
                        robot.sliderRight.setPower(0);
                        robot.sliderLeft.setPower(0);
                        robot.sliderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.sliderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        sliderState = SliderState.MANUAL;
                    }
                    if (gamepad2.circle){
                        robot.posCuvaInit();
                    }
                    if(gamepad2.cross){
                        robot.posCuvaScore();
                    }
                    if(gamepad2.square)
                    {
                        robot.outtake_cuva_out();
                    }
                    if(gamepad2.triangle) robot.outtake_cuva_in();
                    if(gamepad2.right_stick_button) robot.outtake_cuva_off();
                   if(robot.sliderRight.getCurrentPosition()>0 &&robot.sliderRight.getCurrentPosition()<1350) robot.posCuvaInit();
                   if(robot.sliderRight.getCurrentPosition()>1350 &&robot.sliderRight.getCurrentPosition()<2400) robot.posCuvaScore();

                    break;
                case MANUAL:
                    if(gamepad2.left_stick_y>0.15){
                        robot.sliderLeft.setPower(gamepad2.left_stick_y);
                        robot.sliderRight.setPower(-gamepad2.left_stick_y);
                    }
                    else if(gamepad2.left_stick_y<-0.15){
                        robot.sliderLeft.setPower(gamepad2.left_stick_y);
                        robot.sliderRight.setPower(-gamepad2.left_stick_y);
                    }
                    else{
                        robot.sliderLeft.setPower(-0);
                        robot.sliderRight.setPower(0);
                    }
                    if(gamepad2.left_bumper){
                        robot.sliderRight.setPower(0);
                        robot.sliderLeft.setPower(0);
                        //teoretic cele 2 lini de dedesubt sunt redundante petnru ca sunt
                        //prezente si in RobotUtils in toate functiile de go to position
                        robot.sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        sliderState = SliderState.AUTO;
                    }
                    if(gamepad2.left_stick_button){
                        robot.sliderRight.setPower(0);
                        robot.sliderLeft.setPower(0);
                        robot.sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        //teoretic cele 2 lini de dedesubt sunt redundante petnru ca sunt
                        //prezente si in RobotUtils in toate functiile de go to position
                        robot.sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        sliderState = SliderState.AUTO;
                    }
                    if(gamepad2.square) robot.outtake_cuva_out();
                    if(gamepad2.triangle) robot.outtake_cuva_in();
                    if(gamepad2.cross) robot.posCuvaScore();
                    if(gamepad2.circle) robot.posCuvaInit();


                    break;
            }

            telemetry.addData("mod sasiu: ", chasisState.toString());
            telemetry.addData("mod slidere: ", sliderState.toString());
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();

            drive.update();

        }
    }




}