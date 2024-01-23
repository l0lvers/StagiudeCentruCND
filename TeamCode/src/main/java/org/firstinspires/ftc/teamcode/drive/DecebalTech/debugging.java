package org.firstinspires.ftc.teamcode.drive.DecebalTech;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RobotUtils;

@TeleOp
@Config
public class debugging extends LinearOpMode {
    private RobotUtils robot;
    private SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            telemetry.addData("Slider Stanga: ", robot.sliderLeft.getCurrentPosition());
            telemetry.addData("Slider Dreapta: ", robot.sliderRight.getCurrentPosition());
            telemetry.addData("Cuva Stanga: ", robot.cuvaLeft.getPosition());
            telemetry.addData("Cuva Right: ", robot.cuvaRight.getPosition());
            telemetry.addData("Opritoare: ", robot.opritoare.getPosition());
            telemetry.addData("Drone", robot.droneLauncher.getPosition());
            telemetry.addData("Brat Cuva Stanga: ", robot.bratCuvaLeft.getPosition());
            telemetry.addData("Brat Cuva Dreapta: ",robot.bratCuvaRight.getPosition());

            telemetry.update();

            drive.update();
        }
    }

}
