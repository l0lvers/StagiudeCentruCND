package org.firstinspires.ftc.teamcode.drive.auto;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.detection.DetectionPipelineMatei;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.RobotUtils;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RedFar", group="Autonom")
@Config

public class RedFar extends LinearOpMode {
    private RobotUtils robot;
    OpenCvCamera webcam;
    DetectionPipelineMatei detectionPipeline;
    boolean bCameraOpened = false;
    private SampleMecanumDrive drive;

    enum Zone {
        RIGHT,
        LEFT,
        CENTER
    }

    Zone zone = Zone.CENTER;
    Zone zoneFinal = Zone.CENTER;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        detectionPipeline = new DetectionPipelineMatei();
        webcam.setPipeline(detectionPipeline);
        detectionPipeline.setGridSize(10);
        drive = new SampleMecanumDrive(hardwareMap);
        robot = new RobotUtils(hardwareMap);
        robot.sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(2000);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                bCameraOpened = true;
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        Pose2d startPose = new Pose2d(-36, 61, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        // de aici incepi sa scrii trajectory sequences
//---------------------------middle case------------------------------------------------------------
        TrajectorySequence preload = drive.trajectorySequenceBuilder(startPose)

                .build();
        TrajectorySequence pedrum = drive.trajectorySequenceBuilder(preload.end())
                .lineToLinearHeading(new Pose2d(-53,-61, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-53,-12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(42,-12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(42,-35, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(50, -35, Math.toRadians(0)))
                .build();
        TrajectorySequence parcare = drive.trajectorySequenceBuilder(pedrum.end())
                .lineToLinearHeading(new Pose2d(42,-35,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(42,-10,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60,-10, Math.toRadians(0)))
                .build();
//--------------------------------left case-------------------------------------------------------
        TrajectorySequence pixelmov = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-46,-61,Math.toRadians(-90)))
                .build();
        TrajectorySequence pedrum2 = drive.trajectorySequenceBuilder(pixelmov.end())
                .lineToLinearHeading(new Pose2d(-34,-46, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-34,-10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(42,-10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(42,-30,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(51,-30,Math.toRadians(0)))
                .build();
        TrajectorySequence parcare2 = drive.trajectorySequenceBuilder(pedrum2.end())
                .lineToLinearHeading(new Pose2d(42,-30, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(42,-13, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60,-13, Math.toRadians(0)))
                .build();

//------------------------------------right case---------------------------------------------------

            double zoneleft = detectionPipeline.getZoneLuminosity(4);
            double zonemid = Math.min(Math.min(Math.min(detectionPipeline.getZoneLuminosity(64)
                                    , detectionPipeline.getZoneLuminosity(54))
                            , detectionPipeline.getZoneLuminosity(74))
                    , detectionPipeline.getZoneLuminosity(44));


            if (zoneleft < zonemid && zoneleft < 80) zone = Zone.LEFT;
            else if (zonemid < zoneleft && zonemid < 80) zone = Zone.CENTER;
            else zone = Zone.RIGHT;


            telemetry.addData("zone = ", zone.toString());
            telemetry.addData("luminosity zone left", zoneleft);
            telemetry.addData("luminosity zone mid", zonemid);
            telemetry.update();
        }
    }
}


