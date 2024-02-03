package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.detection.DetectionPipelineMatei;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.RobotUtils;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="RedClose_Matei", group="AUTINOMOUSGOOD")
@Config


public class RedClose_Matei extends LinearOpMode {
    private RobotUtils robot;
    OpenCvCamera webcam;
    DetectionPipelineMatei detectionPipeline;

    boolean bCameraOpened = false;
    private SampleMecanumDrive drive;
    private double loopTime=0,loop;

    private  boolean nu_stiu_sa_codez2 = true;

    enum ZoneType{
        RIGHT,
        LEFT,
        CENTER
    }
    ZoneType zone =  ZoneType.CENTER;
    ZoneType zoneFinal =  ZoneType.CENTER;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        detectionPipeline = new DetectionPipelineMatei();
        webcam.setPipeline(detectionPipeline);
        detectionPipeline.setGridSize(10);
        drive =new SampleMecanumDrive(hardwareMap);
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
        sleep(2000);
        Pose2d startPose = new Pose2d(11,-60,Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence pune_preload_stanga = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(6,-38,Math.toRadians(-55)),Math.toRadians(-50))
                .build();
        TrajectorySequence pune_preload_dreapta = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(15,-38,Math.toRadians(-130)),Math.toRadians(110))
                .lineToLinearHeading(new Pose2d(8,-40,Math.toRadians(-130)))
                .build();
        TrajectorySequence pune_preload_mijloc = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(11,-29,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(11,-37,Math.toRadians(-90)))
                .build();

        TrajectorySequence align3 = drive.trajectorySequenceBuilder(pune_preload_stanga.end())
                .lineToLinearHeading(new Pose2d(20,-55,Math.toRadians(15)))
                .lineToLinearHeading(new Pose2d(30,-55,Math.toRadians(0)))
                .addTemporalMarker(0.3,()->{
                    robot.putSliderLowAuto();
                })

                .build();
        TrajectorySequence alignst = drive.trajectorySequenceBuilder(pune_preload_dreapta.end())
                .lineToLinearHeading(new Pose2d(11,-55,Math.toRadians(135)))
                .lineToLinearHeading(new Pose2d(30,-55,Math.toRadians(0)))
                .addTemporalMarker(0.3,()->{
                    robot.putSliderLowAuto();
                })

                .build();
        TrajectorySequence align2 = drive.trajectorySequenceBuilder(pune_preload_mijloc.end())
                .lineToLinearHeading(new Pose2d(20,-55,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(30,-55,Math.toRadians(0)))
                .addTemporalMarker(0.3,()->{
                    robot.putSliderLowAuto();
                })


                .build();
        TrajectorySequence score_preload_zone_left_u = drive.trajectorySequenceBuilder(align3.end())
                .lineToLinearHeading(new Pose2d(49,-23.5,Math.toRadians(-1)),
                        SampleMecanumDrive.getVelocityConstraint(30,Math.toRadians(180),DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(30)
                )
                .addTemporalMarker(0.1,()->{
                    robot.posCuvaScore();
                })

                .build();
        TrajectorySequence score_preload_zone_mid_m = drive.trajectorySequenceBuilder(align2.end())
                .lineToLinearHeading(new Pose2d(49,-30,Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(30,Math.toRadians(180),DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(30)
                )
                .addTemporalMarker(0.1,()->{
                    robot.posCuvaScore();
                })


                .build();
        TrajectorySequence score_preload_zone_right_d = drive.trajectorySequenceBuilder(alignst.end())
                .lineToLinearHeading(new Pose2d(49,-37.5,Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(30,Math.toRadians(180),DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(30)
                )
                .addTemporalMarker(0.1,()->{
                    robot.posCuvaScore();
                })

                .build();
        TrajectorySequence prepark = drive.trajectorySequenceBuilder(score_preload_zone_mid_m.end())
                .lineToLinearHeading(new Pose2d(36,-50,Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(30,30,DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(30)
                )
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(prepark.end())
                .lineToLinearHeading(new Pose2d(52,-59,Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(30,30,DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(30)
                )
                .build();



        while (!isStarted() && !isStopRequested()) {

            double zoneright = Math.min(Math.min(Math.min( detectionPipeline.getZoneLuminosity(85)
                                    ,detectionPipeline.getZoneLuminosity(95))
                            ,detectionPipeline.getZoneLuminosity(96))
                    ,detectionPipeline.getZoneLuminosity(86));
            double zonemid = Math.min(Math.min(Math.min(Math.min(Math.min(detectionPipeline.getZoneLuminosity(65)
                                                    ,detectionPipeline.getZoneLuminosity(55))
                                            ,detectionPipeline.getZoneLuminosity(45))
                                    ,detectionPipeline.getZoneLuminosity(35))
                            ,detectionPipeline.getZoneLuminosity(25))
                    ,detectionPipeline.getZoneLuminosity(15));


            if ( zoneright<zonemid &&  zoneright<92) zone = ZoneType.RIGHT;
            else if (zonemid <  zoneright && zonemid<92)zone = ZoneType.CENTER;
            else zone = ZoneType.LEFT;

            FtcDashboard.getInstance().getTelemetry().addData("luminosity zone mid",zonemid);
            FtcDashboard.getInstance().getTelemetry().addData("luminosity zone right",zoneright);
            FtcDashboard.getInstance().getTelemetry();


            telemetry.addData("zone = ",zone.toString());
            telemetry.addData("luminosity zone right", zoneright);
            telemetry.addData("luminosity zone mid",zonemid);
            loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.addData("luminosity zone 6",detectionPipeline.getZoneLuminosity(8));

            telemetry.update();

            robot.posCuvaInit();


        }
        bCameraOpened = false;
        if(nu_stiu_sa_codez2) {
            zoneFinal = zone;
            nu_stiu_sa_codez2 = false;
            robot.posCuvaInit();
            sleep(100);
            robot.putSliderInitAuto();
        }
        sleep(100);
        switch(zoneFinal){
            case RIGHT:
                drive.followTrajectorySequence(pune_preload_dreapta);
                sleep(300);
                drive.followTrajectorySequence(alignst);
                sleep(700);
                drive.followTrajectorySequence(score_preload_zone_right_d);
                sleep(300);
                robot.outtake_cuva_out();
                sleep(1000);
                drive.followTrajectorySequence(prepark);
                sleep(300);
                robot.posCuvaInit();
                sleep(1000);
                robot. putSliderInitAuto();
                sleep(300);
                robot.outtake_cuva_off();
                drive.followTrajectorySequence(park);
                break;
            case CENTER:
                drive.followTrajectorySequence(pune_preload_mijloc);
                sleep(300);
                drive.followTrajectorySequence(align2);
                sleep(700);
                drive.followTrajectorySequence(score_preload_zone_mid_m);
                sleep(300);
                robot.outtake_cuva_out();
                sleep(1000);
                drive.followTrajectorySequence(prepark);
                sleep(300);
                robot.posCuvaInit();
                sleep(1000);
                robot. putSliderInitAuto();
                sleep(300);
                robot.outtake_cuva_off();
                drive.followTrajectorySequence(park);

                break;
            case LEFT:

                drive.followTrajectorySequence(pune_preload_stanga);
                sleep(300);
                drive.followTrajectorySequence(align3);
                sleep(700);
                drive.followTrajectorySequence(score_preload_zone_left_u);
                sleep(300);
                robot.outtake_cuva_out();
                sleep(1000);
                drive.followTrajectorySequence(prepark);
                sleep(300);
                robot.posCuvaInit();
                sleep(1000);
                robot. putSliderInitAuto();
                sleep(300);
                robot.outtake_cuva_off();
                drive.followTrajectorySequence(park);

                break;
        }
        if (!opModeIsActive()) return;




    }
}