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
public class _2025Code1 extends OpMode {
  private IMU imu;
  private DcMotor back_left;
  private DcMotor front_left;
  private DcMotor back_right;
  private DcMotor front_right;
  private DcMotor flywheel1;
  private DcMotor flywheel2;

  boolean speedtoggle;
  double wheelSpeedDivisor;
  int mode;

  @Override
  public void init() {
    imu = hardwareMap.get(IMU.class, "imu");
    front_left = hardwareMap.get(DcMotor.class, "front_left");
    front_right = hardwareMap.get(DcMotor.class, "front_right");
    back_left = hardwareMap.get(DcMotor.class, "back_left");
    back_right = hardwareMap.get(DcMotor.class, "back_right");

    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
    imu.initialize(parameters);
  }

  public void runOpMode() {
    flywheel1 = hardwareMap.get(DcMotor.class, "flywheel1");
    flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");

    back_left = hardwareMap.get(DcMotor.class, "back_left");
    front_left = hardwareMap.get(DcMotor.class, "front_left");
    back_right = hardwareMap.get(DcMotor.class, "back_right");
    front_right = hardwareMap.get(DcMotor.class, "front_right");

    speedtoggle = true;
    wheelSpeedDivisor = 1.15;
    mode = 0;
    back_left.setDirection(DcMotor.Direction.REVERSE);
    front_left.setDirection(DcMotor.Direction.REVERSE);
  }

  public void moveRobot() {
    double forward = -gamepad1.right_stick_y;
    double strafe = -gamepad1.right_stick_x;
    double rotate = gamepad1.left_stick_x;

    double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    double adjustedForward = -forward * Math.sin(heading) + strafe * Math.cos(heading);
    double adjustedStrafe = forward * Math.cos(heading) + strafe * Math.sin(heading);

    front_left.setPower((adjustedForward + adjustedStrafe + rotate) / wheelSpeedDivisor);
    front_right.setPower((adjustedForward - adjustedStrafe - rotate) / wheelSpeedDivisor);
    back_left.setPower((adjustedForward - adjustedStrafe + rotate) / wheelSpeedDivisor);
    back_right.setPower((adjustedForward + adjustedStrafe - rotate) / wheelSpeedDivisor);
  }

  public void loop() {
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
    if (gamepad1.dpad_down && wheelSpeedDivisor != 1) {
      gamepad1.setLedColor(1, 0, 0, 676);
      gamepad1.rumble(1, 0, 676);
      wheelSpeedDivisor = 2;

      flywheel1.setPower(gamepad1.right_trigger);
      flywheel2.setPower(gamepad1.right_trigger * -1);

    }
  }
}

