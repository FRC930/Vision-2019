/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  
private static final CANSparkMax left1 = new CANSparkMax(1, MotorType.kBrushless);
private static final CANSparkMax left2 = new CANSparkMax(2, MotorType.kBrushless);
private static final CANSparkMax left3 = new CANSparkMax(3, MotorType.kBrushless);
private static final CANSparkMax right1 = new CANSparkMax(4, MotorType.kBrushless);
private static final CANSparkMax right2 = new CANSparkMax(5, MotorType.kBrushless);
private static final CANSparkMax right3 = new CANSparkMax(6, MotorType.kBrushless);

private static final CANEncoder leftEnc = new CANEncoder(left1);
private static final CANEncoder rightEnc = new CANEncoder(right1);

private static final AHRS gyro = new AHRS(SerialPort.Port.kUSB);

Waypoint[] points = new Waypoint[] {
  new Waypoint(0, 0, 0),
  new Waypoint(2, 1, 0)
};

Trajectory.Config config;
Trajectory trajectory;
TankModifier modifier;
EncoderFollower left;
EncoderFollower right;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    left2.follow(left1);
    left3.follow(left1);
    right2.follow(right1);
    right3.follow(right1);

    config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 50.0);
    trajectory = Pathfinder.generate(points, config);

    modifier = new TankModifier(trajectory).modify(0.7);

    left = new EncoderFollower(modifier.getLeftTrajectory());
    right = new EncoderFollower(modifier.getRightTrajectory());

    left.configureEncoder((int)leftEnc.getPosition(), 1, 0.1143);
    left.configurePIDVA(0.7, 0.0, 0.0, 0.15, 0.06);
    right.configureEncoder((int)rightEnc.getPosition(), 1, 0.1143);
    right.configurePIDVA(0.7, 0.0, 0.0, 0.15, 0.06);
    
    gyro.reset();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    double l = left.calculate((int)leftEnc.getPosition());
    double r = right.calculate((int)rightEnc.getPosition());

    double gyro_heading = gyro.getYaw();    // Assuming the gyro is giving a value in degrees
    double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees

    double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    double turn = 0.8 * (-1.0/80.0) * angleDifference;

    left1.set(l + turn);
    right1.set(r - turn);
    //left1.set(0);
    //right1.set(0);
    System.out.println("Left: " + leftEnc.getPosition() + " Right: " + rightEnc.getPosition());

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
