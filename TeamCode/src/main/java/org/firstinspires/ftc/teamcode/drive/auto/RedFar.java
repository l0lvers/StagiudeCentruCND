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
        Pose2d startPos = new Pose2d(-36, -61, Math.toRadians(-90));
        drive.setPoseEstimate(startPos);
        // de aici incepi sa scrii trajectory sequences
        //pune pixeli la centru
        TrajectorySequence pune_preload_dreaptaR = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-24, -38, Math.toRadians(-90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-24,-45,Math.toRadians(-90)))
                .build();

        TrajectorySequence pune_preload_stangaL = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-47, -37, Math.toRadians(-90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-47,-46,Math.toRadians(-90)))
                .build();

        TrajectorySequence pune_preload_center = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-35,-31,Math.toRadians(-90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-36,-50,Math.toRadians(-90)))
                .build();
        ///se aseaza in fata stack-ului
        TrajectorySequence stack1 = drive.trajectorySequenceBuilder(pune_preload_dreaptaR.end())
                .lineToLinearHeading(new Pose2d(-53,-45,Math.toRadians(0)))
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(-53,-11,Math.toRadians(0)))
                .build();

        TrajectorySequence stack2 = drive.trajectorySequenceBuilder(pune_preload_stangaL.end())
                .lineToLinearHeading(new Pose2d(-33,-46, Math.toRadians(-90)))
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(-33,-11, Math.toRadians(0)))
                .build();


        TrajectorySequence stack3 = drive.trajectorySequenceBuilder(pune_preload_center.end())
                .lineToLinearHeading(new Pose2d(-57,-11,Math.toRadians(0)))
                .build();
//merge aprope de pozitia de scoring
        TrajectorySequence drive_score = drive.trajectorySequenceBuilder(stack1.end())
                .lineToLinearHeading(new Pose2d(42,-11, Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(42,-35, Math.toRadians(0)))
                .build();

        TrajectorySequence drive_score1 = drive.trajectorySequenceBuilder(stack2.end())
                .lineToLinearHeading(new Pose2d(42,-11, Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(42,-35, Math.toRadians(0)))
                .build();

        TrajectorySequence drive_score2 = drive.trajectorySequenceBuilder(stack2.end())
                .lineToLinearHeading(new Pose2d(42,-11, Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(42,-35, Math.toRadians(0)))
                .build();
       //scoreaza

        TrajectorySequence score_pos =drive.trajectorySequenceBuilder(drive_score.end())

                .lineToLinearHeading(new Pose2d(52, -35, Math.toRadians(0)))
                .addTemporalMarker(0.2,()->{
                   robot.PutSliderLow();
                })
                .addTemporalMarker(0.4,()->{
                    robot.PosCuvaScore();
                })
                .waitSeconds(1)
                .addTemporalMarker(1,()->{
                    robot.OpritoareOpen();
                })
                .waitSeconds(1)
                .addTemporalMarker(2,()->{
                    robot.PosCuvaInit();
                })
                .waitSeconds(2)

                .build();

        TrajectorySequence score_pos1 =drive.trajectorySequenceBuilder(drive_score1.end())
                .lineToLinearHeading(new Pose2d(52, -35, Math.toRadians(0)))
                .addTemporalMarker(0.2,()->{
                    robot.PutSliderLow();
                })
                .addTemporalMarker(0.4,()->{
                    robot.PosCuvaScore();
                })
                .waitSeconds(1)
                .addTemporalMarker(1,()->{
                    robot.OpritoareOpen();
                })
                .waitSeconds(1)
                .addTemporalMarker(1.5,()->{
                    robot.PosCuvaInit();
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence score_pos2 =drive.trajectorySequenceBuilder(drive_score2.end())

                .lineToLinearHeading(new Pose2d(52, -35, Math.toRadians(0)))
                .addTemporalMarker(0.2,()->{
                    robot.PutSliderLow();
                })
                .addTemporalMarker(0.4,()->{
                    robot.PosCuvaScore();
                })
                .waitSeconds(1)
                .addTemporalMarker(1,()->{
                    robot.opritoateOut();
                })
                .waitSeconds(2)
                .addTemporalMarker(1.5,()->{
                    robot.PosCuvaInit();
                })
                .waitSeconds(1)
                .waitSeconds(0.5)
                .build();

//se parcheaza
        TrajectorySequence park =drive.trajectorySequenceBuilder(score_pos.end())
                .lineToLinearHeading(new Pose2d(42,-35,Math.toRadians(0)))
                .addTemporalMarker(0.2,()->{
                    robot.PutSlidersInit();
                })

                .waitSeconds(0.5)


                .lineToLinearHeading(new Pose2d(42,-12,Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(60,-12, Math.toRadians(0)))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence park1 =drive.trajectorySequenceBuilder(score_pos1.end())

                .lineToLinearHeading(new Pose2d(42,-35,Math.toRadians(0)))
                .addTemporalMarker(0.2,()->{
                    robot.PutSlidersInit();
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(42,-12,Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(60,-12, Math.toRadians(0)))
                .build();

        TrajectorySequence park2 =drive.trajectorySequenceBuilder(score_pos2.end())

                .lineToLinearHeading(new Pose2d(42,-35,Math.toRadians(0)))
                .addTemporalMarker(0.2,()->{
                    robot.PutSlidersInit();
                })
                .lineToLinearHeading(new Pose2d(42,-12,Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(60,-12, Math.toRadians(0)))
                .waitSeconds(0.5)
                .build();



        while (!isStarted() && !isStopRequested()) {

            robot.PosCuvaInit();
            robot.PutSlidersInit();
            double zoneleft = detectionPipeline.getZoneLuminosity(4);
            double zonemid = Math.min(Math.min(Math.min( detectionPipeline.getZoneLuminosity(64)
                                    ,detectionPipeline.getZoneLuminosity(54))
                            ,detectionPipeline.getZoneLuminosity(74))
                    ,detectionPipeline.getZoneLuminosity(44));


            if (zoneleft < zonemid && zoneleft < 80) zone = Zone.LEFT;
            else if (zonemid < zoneleft && zonemid < 80) zone = Zone.CENTER;
            else zone = Zone.RIGHT;


            telemetry.addData("zone = ",zone.toString());
            telemetry.addData("luminosity zone left",zoneleft);
            telemetry.addData("luminosity zone mid",zonemid);


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
