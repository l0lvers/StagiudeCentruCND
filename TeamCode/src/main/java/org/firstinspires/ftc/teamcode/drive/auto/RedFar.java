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
    private  boolean nu_stiu_sa_codez2 = true;

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
        TrajectorySequence pune_preload_dreaptaR = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-36, -30, Math.toRadians(-90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-36,-50,Math.toRadians(-90)))
                .build();

        TrajectorySequence pune_preload_stangaL = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-36, -30, Math.toRadians(-90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-36,-50,Math.toRadians(-90)))
                .build();

        TrajectorySequence pune_preload_center = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-36,-30,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-36,-50,Math.toRadians(-90)))
                .build();
        ///dasd/a/d/d/asd/asdasfa
        TrajectorySequence stack1 = drive.trajectorySequenceBuilder(pune_preload_dreaptaR.end())
                .lineToLinearHeading(new Pose2d(-57,-10,Math.toRadians(0)))
                .build();

        TrajectorySequence stack2 = drive.trajectorySequenceBuilder(pune_preload_stangaL.end())
                .lineToLinearHeading(new Pose2d(-57,-10,Math.toRadians(0)))
                .build();


        TrajectorySequence stack3 = drive.trajectorySequenceBuilder(pune_preload_center.end())
                .lineToLinearHeading(new Pose2d(-57,-10,Math.toRadians(0)))
                .build();
//as/fa/fa/fa/sfa/sf/af/
        TrajectorySequence drive_score = drive.trajectorySequenceBuilder(stack1.end())
                .lineToLinearHeading(new Pose2d(42,-10, Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(42,-35, Math.toRadians(0)))
                .build();

        TrajectorySequence drive_score1 = drive.trajectorySequenceBuilder(stack2.end())
                .lineToLinearHeading(new Pose2d(42,-10, Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(42,-35, Math.toRadians(0)))
                .build();

        TrajectorySequence drive_score2 = drive.trajectorySequenceBuilder(stack2.end())
                .lineToLinearHeading(new Pose2d(42,-10, Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(42,-35, Math.toRadians(0)))
                .build();
       //fafggshsshbs bhf bvahy bvas va h hjbshbg

        TrajectorySequence score_pos =drive.trajectorySequenceBuilder(drive_score.end())

                .lineToLinearHeading(new Pose2d(52, -35, Math.toRadians(0)))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence score_pos1 =drive.trajectorySequenceBuilder(drive_score1.end())
                .lineToLinearHeading(new Pose2d(52, -35, Math.toRadians(0)))
                

                .build();

        TrajectorySequence score_pos2 =drive.trajectorySequenceBuilder(drive_score2.end())

                .lineToLinearHeading(new Pose2d(52, -35, Math.toRadians(0)))
                .waitSeconds(0.5)
                .build();

//dasfagagashah
        TrajectorySequence park =drive.trajectorySequenceBuilder(score_pos.end())
                .lineToLinearHeading(new Pose2d(42,-35,Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(42,-10,Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(60,-10, Math.toRadians(0)))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence park1 =drive.trajectorySequenceBuilder(score_pos1.end())

                .lineToLinearHeading(new Pose2d(42,-35,Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(42,-10,Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(60,-10, Math.toRadians(0)))
                .build();

        TrajectorySequence park2 =drive.trajectorySequenceBuilder(score_pos2.end())

                .lineToLinearHeading(new Pose2d(42,-35,Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(42,-10,Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(60,-10, Math.toRadians(0)))
                .waitSeconds(0.5)
                .build();


        while (!isStarted() && !isStopRequested()) {

            double zoneleft = detectionPipeline.getZoneLuminosity(4);
            double zonemid = Math.min(Math.min(Math.min(detectionPipeline.getZoneLuminosity(64)
                                    , detectionPipeline.getZoneLuminosity(54))
                            , detectionPipeline.getZoneLuminosity(74))
                    , detectionPipeline.getZoneLuminosity(44));


            if (zoneleft < zonemid && zoneleft < 80) zone = Zone.RIGHT;
            else if (zonemid < zoneleft && zonemid < 80) zone = Zone.CENTER;
            else zone = Zone.LEFT;


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
                      sleep(500);
                      drive.followTrajectorySequence(park);

                      break;

                  case LEFT:
                      drive.followTrajectorySequence(pune_preload_dreaptaR);
                      sleep(300);
                      drive.followTrajectorySequence(stack2);
                      sleep(300);
                      drive.followTrajectorySequence(drive_score1);
                      sleep(300);
                      drive.followTrajectorySequence(score_pos1);
                      sleep(500);
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
                      sleep(500);
                      drive.followTrajectorySequence(park2);
                      break;
              }


    }
}
