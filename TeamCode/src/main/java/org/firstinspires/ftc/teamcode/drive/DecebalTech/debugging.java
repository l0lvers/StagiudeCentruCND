package org.firstinspires.ftc.teamcode.drive.DecebalTech;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RobotUtils;

@TeleOp
@Config
public class debugging extends LinearOpMode {
    private RobotUtils robot;
    private SampleMecanumDrive drive;

    public static int positie = 100;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        robot = new RobotUtils(hardwareMap);
        robot.sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


//        robot.sliderRight.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y/2,
                            -gamepad1.left_stick_x/2,
                            -gamepad1.right_stick_x/2
                    )
            );

//            if(gamepad1.right_bumper) {
//                robot.sliderRight.setPower(0.5);
//                robot.sliderLeft.setPower(-0.5);
//            }
//            else if(gamepad1.left_bumper) {
//                robot.sliderRight.setPower(-0.5);
//                robot.sliderLeft.setPower(0.5);
//            }
//            else {
//                robot.sliderRight.setPower(0);
//                robot.sliderLeft.setPower(0);
//            }

            if(gamepad1.square) robot.PutSliderToPosition(positie,0.5);
            if(gamepad1.dpad_up) robot.PosCuvaScore();
            if(gamepad1.dpad_down) robot.PosCuvaInit();

            telemetry.addData("Slider Stanga: ", robot.sliderLeft.getCurrentPosition());
            telemetry.addData("Slider Dreapta: ", robot.sliderRight.getCurrentPosition());
            //telemetry.addData("Cuva Stanga: ", robot.cuvaLeft.getPosition());
            //telemetry.addData("Cuva Right: ", robot.cuvaRight.getPosition());
            telemetry.addData("Opritoare: ", robot.opritoare.getPosition());
            telemetry.addData("Drone", robot.droneLauncher.getPosition());
            telemetry.addData("Brat Cuva Stanga: ", robot.bratCuvaLeft.getPosition());
            telemetry.addData("Brat Cuva Dreapta: ",robot.bratCuvaRight.getPosition());
            telemetry.addData("power_left",robot.sliderLeft.getPower());
            telemetry.addData("power_right",robot.sliderRight.getPower());

            telemetry.update();

            drive.update();
        }
    }

}
