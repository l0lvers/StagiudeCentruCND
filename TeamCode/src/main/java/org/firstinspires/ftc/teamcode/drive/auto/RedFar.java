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
        Pose2d startPos = new Pose2d(-36, -61, Math.toRadians(90));
        drive.setPoseEstimate(startPos);
        // de aici incepi sa scrii trajectory sequences
        TrajectorySequence pune_preload_dreaptaR = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-46, -39, Math.toRadians(-90)))


                .build();

        TrajectorySequence score_pos1R = drive.trajectorySequenceBuilder(pune_preload_dreaptaR.end())
                .lineToLinearHeading(new Pose2d(-46, -45,Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(-23,-61, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(47,-28, Math.toRadians(0)),Math.toRadians(0))
              //  .addTemporalMarker(0.2,()-> {
                //    robot.go_sliders_mid_auto();

               // })
               // .addTemporalMarker(0.2,()-> {
               //     robot.PosCuvaScore();
               // })
                .build();

        TrajectorySequence stackR = drive.trajectorySequenceBuilder(score_pos1R.end())
                .lineToLinearHeading(new Pose2d(43,-32,Math.toRadians(0)))
               // .addTemporalMarker(0.2,()-> {
               //     robot.PosCuvaInit();

                //})
                //.addTemporalMarker(0.2,()-> {
                 //   robot.go_slider_low_auto();

                //})
                .lineToLinearHeading(new Pose2d(-32,-37,Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-50,-35.5, Math.toRadians(0)), Math.toRadians(-10))
                .lineToLinearHeading(new Pose2d(-60, -35.5, Math.toRadians(0)))

                .build();

        TrajectorySequence score_pos2R =drive.trajectorySequenceBuilder(stackR.end())
                .lineToLinearHeading(new Pose2d(-50,-35.5, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-23,-60, Math.toRadians(0)), Math.toRadians(-15))
                .lineToLinearHeading(new Pose2d(0,-60,Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(47,-33, Math.toRadians(0)),Math.toRadians(15))
                .build();

        TrajectorySequence parkR =drive.trajectorySequenceBuilder(score_pos2R.end())
                .lineToLinearHeading(new Pose2d(40,-12,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60, -12,Math.toRadians(0)))
                .build();





        while (!isStarted() && !isStopRequested()) {

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
           /*  bCameraOpened = false;
            if(nu_stiu_sa_codez2) {
              zoneFinal = zone;
              nu_stiu_sa_codez2 = false;
          }
*/

                drive.followTrajectorySequence(pune_preload_dreaptaR);
                sleep(300);
                drive.followTrajectorySequence(score_pos1R);
                sleep(300);
                drive.followTrajectorySequence(stackR);
                sleep(500);
                drive.followTrajectorySequence(score_pos2R);
                sleep(1000);
                drive.followTrajectorySequence(parkR);



    }
}
