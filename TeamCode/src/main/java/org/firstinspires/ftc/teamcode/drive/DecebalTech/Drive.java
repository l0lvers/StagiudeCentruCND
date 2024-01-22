package org.firstinspires.ftc.teamcode.drive.DecebalTech;

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


@TeleOp(name="Drive", group="Linear Opmode")
@Config


public class Drive extends LinearOpMode {


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



    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            switch (chasisState) {
                case DRIVE:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y / 1.5,
                                    -gamepad1.left_stick_x / 1.5,
                                    -gamepad1.right_stick_x / 1.5
                            )
                    );

                    if (gamepad1.right_trigger > 0.3) {
                        chasisState = ChasisState.TURBO;
                    }
                    if (gamepad1.left_trigger > 0.3) {

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

                    if (gamepad1.right_trigger == 0) {

                        chasisState = ChasisState.DRIVE;
                    }
                    break;
                case PRECISION:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y / 3,
                                    -gamepad1.left_stick_x / 3,
                                    -gamepad1.right_stick_x / 3
                            )
                    );

            }
            if (gamepad1.left_trigger == 0) {

                chasisState = ChasisState.DRIVE;
            }
            if(gamepad1.triangle) robot.DroneLaunch();
            if(gamepad1.dpad_up) robot.DroneInit();
            if(gamepad1.square) robot.IntakeOn();
            if(gamepad1.circle) robot.IntakeStop();
            if(gamepad1.cross) robot.IntakeReverse();

            switch (sliderState){
                case AUTO:
                    if(gamepad2.dpad_down) robot.PutSliderLow();
                    if(gamepad2.dpad_right) robot.PutSlidersInit();
                    if(gamepad2.dpad_left) robot.PutSliderMid();
                    if(gamepad2.dpad_up) robot.PutSliderHigh();
                    if(gamepad2.left_bumper){
                        robot.sliderRight.setPower(0);
                        robot.sliderLeft.setPower(0);
                        robot.sliderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.sliderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        sliderState = SliderState.MANUAL;
                    }
                    if(robot.sliderLeft.getCurrentPosition()>0 && robot.sliderLeft.getCurrentPosition()<1550) {
                        robot.PosCuvaInit();
                    }
                    if(robot.sliderLeft.getCurrentPosition()>1550 && robot.sliderLeft.getCurrentPosition()<3000) {
                        robot.PosCuvaScore();
                    }
                    break;

                case MANUAL:
                    if(gamepad2.right_trigger > 0.5)
                    {
                        robot.sliderLeft.setPower(0.3);
                        robot.sliderRight.setPower(-0.3);
                    }
                    else if(gamepad2.left_trigger >0.5)
                    {
                        robot.sliderLeft.setPower(-0.3);
                        robot.sliderRight.setPower(0.3);
                    }
                    else {robot.sliderLeft.setPower(0); robot.sliderRight.setPower(0);}
                    if(gamepad2.right_bumper)
                    {
                        robot.sliderRight.setPower(0);
                        robot.sliderLeft.setPower(0);
                        robot.sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        sliderState = SliderState.AUTO;
                    }

            }
            if(gamepad2.triangle) robot.OpritoareClose();
            if(gamepad2.square) robot.OpritoareOpen();
            if(gamepad2.cross) robot.PosCuvaScore();
            if (gamepad2.circle) robot.PosCuvaInit();


            drive.update();

        }
    }




}