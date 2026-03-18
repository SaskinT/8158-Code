package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimbConstants.*;

public class ClimbSubsystem extends SubsystemBase {
  private final SparkMax m_climberMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushless);

  public ClimbSubsystem() {
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig.smartCurrentLimit(CLIMBER_MOTOR_CURRENT_LIMIT);
    
    // Uyarıları önlemek için yeni API kullanımı
    m_climberMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setClimber(double speed) {
    m_climberMotor.set(speed);
  }

  public void stop() {
    m_climberMotor.stopMotor();
  }
}