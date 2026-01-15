package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax shooterMotor;
  private final SparkMax angleMotor;

  public ShooterSubsystem() {
    shooterMotor = new SparkMax(
        Shooter.shooterFlywheel,
        MotorType.kBrushless
    );

    angleMotor = new SparkMax(
        Shooter.shooterAngle,
        MotorType.kBrushless
    );

    configureShooterMotor();
    configureAngleMotor();
  }

  private void configureShooterMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kCoast)
        .inverted(Shooter.flywheelInverted)
        .smartCurrentLimit(40);

    shooterMotor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );
  }

  private void configureAngleMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .inverted(Shooter.angleInverted)
        .smartCurrentLimit(20);

    angleMotor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );
  }

  /* =======================
     SHOOTER (FLYWHEEL)
     ======================= */

  public void shoot(double speed) {
    shooterMotor.set(speed);
  }

  public void stopShooter() {
    shooterMotor.set(0);
  }

  /* =======================
     ANGLE
     ======================= */

  public void angleUp() {
    angleMotor.set(0.4);
  }

  public void angleDown() {
    angleMotor.set(-0.3);
  }

  public void stopAngle() {
    angleMotor.set(0);
  }
}
