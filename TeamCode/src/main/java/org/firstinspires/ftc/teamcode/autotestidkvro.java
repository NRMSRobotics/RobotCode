package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Autonomous(name = "autotestidkvro (Blocks to Java)")
public class autotestidkvro extends LinearOpMode {

  private Limelight3A LimeLightLemonade;
  private DcMotor front_left;
  private DcMotor back_left;
  private DcMotor front_right;
  private DcMotor back_right;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    int pos_x;
    double botX;
    double botY;
    double botZ;
    double strafeThat;
    LLResult llight_result;
    Pose3D botpose;


    LimeLightLemonade = hardwareMap.get(Limelight3A.class, "LimeLight Lemonade");
    front_left = hardwareMap.get(DcMotor.class, "front_left");
    back_left = hardwareMap.get(DcMotor.class, "back_left");
    front_right = hardwareMap.get(DcMotor.class, "front_right");
    back_right = hardwareMap.get(DcMotor.class, "back_right");

    pos_x = 0;
    LimeLightLemonade.pipelineSwitch(0);
    LimeLightLemonade.start();
    LimeLightLemonade.setPollRateHz(100);
    waitForStart();
    if (opModeIsActive()) {
      front_left.setDirection(DcMotor.Direction.REVERSE);
      back_left.setDirection(DcMotor.Direction.REVERSE);
      moveForwardFor(0.1, 10);
      sleep(300);
      while (opModeIsActive()) {
        llight_result = LimeLightLemonade.getLatestResult();
        if (llight_result.isValid()) {
          telemetry.addData("LLResult is valid", 1);

          botpose = llight_result.getBotpose();

//          List<LLResultTypes.FiducialResult> fiducal = llight_result.getFiducialResults();
//
//          for (LLResultTypes.FiducialResult fiducal : fiducals)

          telemetry.addData("tx", llight_result.getTx());
          telemetry.addData("ty", llight_result.getTy());
          telemetry.addData("botpose", botpose.toString());

          if (botpose != null){
            botX = botpose.getPosition().x;
            botY = botpose.getPosition().y;
            botZ = botpose.getPosition().z;

            telemetry.addData("BotPosePositionX", botX);
            telemetry.addData("BotPosePositionY", botY);
            telemetry.addData("BotPosePositionZ", botZ);

            strafeThat = llight_result.getTx() * 0.03;

            back_left.setPower(strafeThat);
            front_left.setPower(strafeThat);
           back_right.setPower(-strafeThat);
           front_right.setPower(-strafeThat);
          }

          // Note to self: Set to ~= 0, <= -5 works sometimes based on where you place the robot, but values will never be consistent.
//          if (llight_result.getTx() != 0) {
//            front_left.setPower(0);
//            back_left.setPower(0);
//            front_right.setPower(0);
//            back_right.setPower(0);
//          } else {
//            back_left.setPower(0.1);
//            front_left.setPower(0.1);
//            back_right.setPower(-0.1);
//            front_right.setPower(-0.1);
//          }
        } else {
          telemetry.addData("Error: LLResult not IsValid", 1);
        }
        telemetry.update();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void moveForwardFor(double moveSpeed, int moveTime) {
    back_left.setPower(moveSpeed);
    front_left.setPower(moveSpeed);
    front_right.setPower(moveSpeed);
    back_right.setPower(moveSpeed);
    sleep(moveTime);
    telemetry.addData("uhhhh", 123);
    back_left.setPower(0);
    front_left.setPower(0);
    front_right.setPower(0);
    back_right.setPower(0);
  }
}
