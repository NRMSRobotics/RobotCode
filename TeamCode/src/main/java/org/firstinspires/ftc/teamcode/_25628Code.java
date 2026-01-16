package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
@TeleOp(name = "_2025Code1")
public class _25628Code extends OpMode {
  private IMU imu;
  private DcMotor back_left;
  private DcMotor front_left;
  private DcMotor back_right;
  private DcMotor front_right;
  private DcMotor flywheel1;
  private DcMotor flywheel2;

  private DcMotor contrivance1;

  private DcMotor contrivance2;

  private DcMotor contrivance3;

  boolean imuInit;
  double wheelSpeedDivisor;
  int mode;
  float vertical;
  float horizontal;
  float pivot;

  @Override
  public void init() {
    imu = hardwareMap.get(IMU.class, "imu");
    front_left = hardwareMap.get(DcMotor.class, "front_left");
    front_right = hardwareMap.get(DcMotor.class, "front_right");
    back_left = hardwareMap.get(DcMotor.class, "back_left");
    back_right = hardwareMap.get(DcMotor.class, "back_right");

    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
    imu.initialize(parameters);

    flywheel1 = hardwareMap.get(DcMotor.class, "flywheel1");
    flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");

    contrivance1 = hardwareMap.get(DcMotor.class, "motor1");
    contrivance2 = hardwareMap.get(DcMotor.class, "motor2");
    contrivance3 = hardwareMap.get(DcMotor.class, "motor3");

    back_left = hardwareMap.get(DcMotor.class, "back_left");
    front_left = hardwareMap.get(DcMotor.class, "front_left");
    back_right = hardwareMap.get(DcMotor.class, "back_right");
    front_right = hardwareMap.get(DcMotor.class, "front_right");

    imuInit = false;
    wheelSpeedDivisor = 1.15;
    mode = 0;
    back_left.setDirection(DcMotor.Direction.REVERSE);
    front_left.setDirection(DcMotor.Direction.REVERSE);
  }

  public void moveRobot() {
    // gamepad2 works the same as gamepad 1, [may throw errors if not connected to robot?]

    double forward = -gamepad1.right_stick_y;
    double strafe = gamepad1.right_stick_x;
    double rotate = gamepad1.left_stick_x;

    // Dear program seeker,
    // This program was created by Franklin wade; and for one purpose.
    // To move the mysterious "Three Motors".
    // The objectives and morals of these "Motors" (or contrivances), are currently unknown,
    // However, you may change the way you control them under these few lines, starting at
    // 'double contrivance1power'
    // I am using the second gamepad controller for this task.
    // Thank you, program seeker. Good luck with your perilous journey.
    // This message was brought to you by FremLank 5G Wireless, with Gold Plated connectors (AI Powered)
    // pls don't kill me henry im sory
    // I am so damn tired - Franklin (8:45-8:46 AM at Friday, January 16, 2026)
    // hey robot inspectors, inspect this comment, will you? i bet you can't looool

    double contrivance1power = gamepad2.right_trigger;
    double contrivance2power = gamepad2.left_trigger;
    double contrivance3power = gamepad2.left_stick_y;

    telemetry.addData("Forward", forward);
    telemetry.addData("Strafe", strafe);
    telemetry.addData("Rotate", rotate);

    double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    double adjustedStrafe = -forward * Math.sin(heading) + strafe * Math.cos(heading);
    double adjustedForward = forward * Math.cos(heading) + strafe * Math.sin(heading);

    if (imuInit) {
      front_left.setPower((adjustedForward + adjustedStrafe + rotate) / wheelSpeedDivisor);
      front_right.setPower((adjustedForward - adjustedStrafe - rotate) / wheelSpeedDivisor);
      back_left.setPower((adjustedForward - adjustedStrafe + rotate) / wheelSpeedDivisor);
      back_right.setPower((adjustedForward + adjustedStrafe - rotate) / wheelSpeedDivisor);
    }
    else {
      front_left.setPower((forward + strafe + rotate) / wheelSpeedDivisor);
      front_right.setPower((forward - strafe - rotate) / wheelSpeedDivisor);
      back_left.setPower((forward - strafe + rotate) / wheelSpeedDivisor);
      back_right.setPower((forward + strafe - rotate) / wheelSpeedDivisor);
    }

    // franklin's code

    contrivance1.setPower(contrivance1power);
    contrivance2.setPower(contrivance2power);
    contrivance3.setPower(contrivance3power);

    // is that seriously ALL i need to do

    // that was literally 9 lines of code bro

    // are we serious rn 😔


  }

  public void loop() {
    if (gamepad1.touchpad){
      imu.resetYaw();
      imuInit = true;
      gamepad1.rumble(1, 0, 676);
    }
    moveRobot();

    //Extra Features, note that these are still in testing and probably include a bunch of bugs.
    if (gamepad1.x && gamepad1.y) {
      //Killswitch
      terminateOpModeNow();
    }
    if (gamepad1.dpad_up && wheelSpeedDivisor != 1) {
      gamepad1.setLedColor(0, 1, 0, 676);
      gamepad1.rumble(1, 0, 676);
      wheelSpeedDivisor = 1;
    }
    if (gamepad1.dpad_down && wheelSpeedDivisor != 2) {
      gamepad1.setLedColor(1, 0, 0, 676);
      gamepad1.rumble(1, 0, 676);
      wheelSpeedDivisor = 2;

      //flywheel1.setPower(gamepad1.right_trigger);
      //flywheel2.setPower(gamepad1.right_trigger * -1);
    }
  }
}

