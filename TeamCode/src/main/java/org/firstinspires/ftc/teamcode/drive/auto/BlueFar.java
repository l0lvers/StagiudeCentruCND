package org.firstinspires.ftc.teamcode.drive.auto;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous(name = "BlueFar", group="Autonom")
@Config

public class BlueFar extends LinearOpMode {
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
        
        sleep(1000);
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
        sleep(2000);
        Pose2d startPos = new Pose2d(-36, 61, Math.toRadians(90));
        drive.setPoseEstimate(startPos);
        // de aici incepi sa scrii trajectory sequences

        TrajectorySequence pune_preload_dreaptaR = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-45, 61, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-45,35,Math.toRadians(90)))
                .build();

        TrajectorySequence stack1 = drive.trajectorySequenceBuilder(pune_preload_dreaptaR.end())
                .lineToLinearHeading(new Pose2d(-45,47,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-33,47,Math.toRadians(0)))
                .build();

        TrajectorySequence drive_score = drive.trajectorySequenceBuilder(stack1.end())
                .lineToLinearHeading(new Pose2d(-33,10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(42,10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(42,40, Math.toRadians(0)))
                .build();

        TrajectorySequence score_pos =drive.trajectorySequenceBuilder(drive_score.end())

                .lineToLinearHeading(new Pose2d(51, 40, Math.toRadians(0)))
                .addTemporalMarker(0.2,()->{
                    robot.putSliderLow();
                })
                .addTemporalMarker(0.4,()->{
                    robot.posCuvaScore();
                })
                .addTemporalMarker(1,()->{
                    robot.outtake_cuva_in();
                })
                
                .addTemporalMarker(2,()->{
                    robot.posCuvaInit();
                })

                .build();

        TrajectorySequence park =drive.trajectorySequenceBuilder(score_pos.end())
                .lineToLinearHeading(new Pose2d(42,40,Math.toRadians(0)))
                .addTemporalMarker(0.2,()->{
                    robot.putSliderInit();
                })



                .lineToLinearHeading(new Pose2d(42,10,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60,10, Math.toRadians(0)))
                .build();
        //stanga

        TrajectorySequence pune_preload_stangaL = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-47, 37, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-47,46,Math.toRadians(90)))
                .build();

        TrajectorySequence stack2 = drive.trajectorySequenceBuilder(pune_preload_stangaL.end())
                .lineToLinearHeading(new Pose2d(-33,46, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-33,9, Math.toRadians(0)))
                .build();


        TrajectorySequence drive_score1 = drive.trajectorySequenceBuilder(stack2.end())
                .lineToLinearHeading(new Pose2d(42,11, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(42,35, Math.toRadians(0)))
                .build();

        TrajectorySequence score_pos1 =drive.trajectorySequenceBuilder(drive_score1.end())
                .lineToLinearHeading(new Pose2d(52, 35, Math.toRadians(0)))
                .addTemporalMarker(0.2,()->{
                    robot.putSliderLow();
                })
                .addTemporalMarker(0.4,()->{
                    robot.posCuvaScore();
                })
                 
                .addTemporalMarker(1,()->{
                    robot.outtake_cuva_in();
                })
                 
                .addTemporalMarker(1.5,()->{
                    robot.posCuvaInit();
                })
                 
                .build();

        TrajectorySequence park1 =drive.trajectorySequenceBuilder(score_pos1.end())

                .lineToLinearHeading(new Pose2d(42,35,Math.toRadians(0)))
                .addTemporalMarker(0.2,()->{
                    robot.putSliderInit();
                })
                .lineToLinearHeading(new Pose2d(42,12,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60,12, Math.toRadians(0)))
                .build();

        //centru

        TrajectorySequence pune_preload_center = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-35,31,Math.toRadians(90)))
                 
                .lineToLinearHeading(new Pose2d(-36,50,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-45,50,Math.toRadians(90)))
                .build();

        TrajectorySequence stack3 = drive.trajectorySequenceBuilder(pune_preload_center.end())
                .lineToLinearHeading(new Pose2d(-57,0,Math.toRadians(0)))
                 
                .build();

        TrajectorySequence drive_score2 = drive.trajectorySequenceBuilder(stack2.end())
                .lineToLinearHeading(new Pose2d(42,15, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(42,35, Math.toRadians(0)))
                .build();

        TrajectorySequence score_pos2 =drive.trajectorySequenceBuilder(drive_score2.end())

                .lineToLinearHeading(new Pose2d(52, 40, Math.toRadians(0)))
                .addTemporalMarker(0.2,()->{
                    robot.putSliderLow();
                })
                .addTemporalMarker(0.4,()->{
                    robot.posCuvaScore();
                })
                 
                .addTemporalMarker(1,()->{
                    robot.outtake_cuva_out();
                })
                 
                .addTemporalMarker(1.5,()->{
                    robot.posCuvaInit();
                })
                .addTemporalMarker(2,()->{
                    robot.outtake_cuva_off();
                })

                .build();

        TrajectorySequence park2 =drive.trajectorySequenceBuilder(score_pos2.end())

                .lineToLinearHeading(new Pose2d(42,35,Math.toRadians(0)))
                .addTemporalMarker(0.2,()->{
                    robot.putSliderInit();
                })
                .lineToLinearHeading(new Pose2d(42,15,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60,15, Math.toRadians(0)))
                .build();


        while (!isStarted() && !isStopRequested()) {


            robot.posCuvaInit();

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
        switch (zoneFinal) {
            case RIGHT:
                drive.followTrajectorySequence(pune_preload_dreaptaR);
                sleep(300);
                drive.followTrajectorySequence(stack1);
                sleep(300);
                drive.followTrajectorySequence(drive_score);
                sleep(300);
                drive.followTrajectorySequence(score_pos);
                sleep(1000);
                drive.followTrajectorySequence(park);

                break;

            case LEFT:
                drive.followTrajectorySequence(pune_preload_stangaL);
                sleep(300);
                drive.followTrajectorySequence(stack2);
                sleep(300);
                drive.followTrajectorySequence(drive_score1);
                sleep(300);
                drive.followTrajectorySequence(score_pos1);
                sleep(1000);
                drive.followTrajectorySequence(park1);

                break;

            case CENTER:
                drive.followTrajectorySequence(pune_preload_center);
                sleep(300);
                drive.followTrajectorySequence(stack3);
                sleep(300);
                drive.followTrajectorySequence(drive_score2);
                sleep(300);
                drive.followTrajectorySequence(score_pos2);
                sleep(1000);
                drive.followTrajectorySequence(park2);
                break;
        }
    }

}

