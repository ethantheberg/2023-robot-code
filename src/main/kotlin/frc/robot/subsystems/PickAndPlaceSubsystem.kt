package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.commands.SetPickAndPlacePosition
import frc.robot.constants.ArmConstants
import frc.robot.constants.CommandValues
import frc.robot.constants.IntakeConstants

class PickAndPlaceSubsystem : SubsystemBase(){

    val elevatorMotor = CANSparkMax(ArmConstants.elevatorMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
    val elbowMotor = CANSparkMax(ArmConstants.elbowMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
    val wristMotor = CANSparkMax(ArmConstants.wristMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
    val intakeOneMotor = CANSparkMax(ArmConstants.intakeMotorOneId, CANSparkMaxLowLevel.MotorType.kBrushless)
    val intakeTwoMotor = CANSparkMax(ArmConstants.intakeMotorTwoId, CANSparkMaxLowLevel.MotorType.kBrushless)

    val elevatorEncoder = elevatorMotor.encoder
    val elbowEncoder = elbowMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    val wristEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)

    val forwardLimit = DigitalInput(ArmConstants.topBreakerId)
    val reverseLimit = DigitalInput(ArmConstants.bottomBreakerId)


    var elevatorZeroed = false

    //Is the carriage at the top of the elevator?

    val topHit get() = !forwardLimit.get()

    // Is the carriage at the bottom of the elevator?
    val bottomHit get() = !reverseLimit.get()

    object Telemetry {
        val armTable = NetworkTableInstance.getDefault().getTable("Arm")
        var elbowPosition = armTable.getDoubleTopic("ElbowPosition").publish()
        var elbowVelocity = armTable.getDoubleTopic("ElbowVelocity").publish()
        var elevatorPosition = armTable.getDoubleTopic("ElevatorPosition").publish()
        var elevatorVelocity = armTable.getDoubleTopic("ElevatorVelocity").publish()

        val elevatorVoltage = armTable.getDoubleTopic("ElevatorVoltage").publish()
        val bottomHit = armTable.getBooleanTopic("BottomHit").publish()
        val topHit = armTable.getBooleanTopic("TopHit").publish()
        val wristPosition = armTable.getDoubleTopic("WristPosition").publish()
        val wristVoltage = armTable.getDoubleTopic("WristVoltage").publish()
        val elbowVoltage = armTable.getDoubleTopic("ElbowVolts").publish()
        val intakeVoltage = armTable.getDoubleTopic("IntakeVolts").publish()

        val elevatorZeroed =  armTable.getBooleanTopic("ElevatorZeroed").publish()
        
        val pidTable = armTable.getSubTable("ArmPIDs")

        val elevatorP = pidTable.getDoubleTopic("elevatorP").getEntry(ArmConstants.elevatorP)
        val elevatorI = pidTable.getDoubleTopic("elevatorI").getEntry(ArmConstants.elevatorI)
        val elevatorD = pidTable.getDoubleTopic("elevatorD").getEntry(ArmConstants.elevatorD)
        val elbowP = pidTable.getDoubleTopic("elbowP").getEntry(ArmConstants.elbowP)
        val elbowI = pidTable.getDoubleTopic("elbowI").getEntry(ArmConstants.elbowI)
        val elbowD = pidTable.getDoubleTopic("elbowD").getEntry(ArmConstants.elbowD)
        val wristP = pidTable.getDoubleTopic("wristP").getEntry(ArmConstants.wristP)
        val wristI = pidTable.getDoubleTopic("wristI").getEntry(ArmConstants.wristI)
        val wristD = pidTable.getDoubleTopic("wristD").getEntry(ArmConstants.wristD)

        val nt = NetworkTableInstance.getDefault().getTable("DriverControl")

        var cubeNT = nt.getBooleanTopic("Cube").publish() // Both
        var middlePlaceNT = nt.getBooleanTopic("Middle Place").publish() // Place
        var floorNT = nt.getBooleanTopic("Floor").publish() // Place
        var chuteNT = nt.getBooleanTopic("Chute Pickup").publish() // Pickup
        var pickupNT = nt.getBooleanTopic("Pickup").publish()

        // THESE ARE ONLY FOR DRIVERS(Network Tables), NOT USED IN CODE
        var groundNT = nt.getBooleanTopic("Ground Pickup").publish() // Pickup
        var coneNT = nt.getBooleanTopic("Cone").publish() // Both
        var highPlaceNT = nt.getBooleanTopic("High Place").publish() // Place

        var visionNT = nt.getBooleanTopic("Vision").publish()



    }


    init {
        //intake
        elbowMotor.setSmartCurrentLimit(38)
        wristMotor.setSmartCurrentLimit(38)
        intakeOneMotor.setSmartCurrentLimit(28)
        intakeTwoMotor.setSmartCurrentLimit(28)
        elevatorMotor.setSmartCurrentLimit(38)
        elbowMotor.inverted = ArmConstants.elbowMotorInverted
        wristMotor.idleMode = CANSparkMax.IdleMode.kBrake
        wristMotor.inverted = true
        //intakeOneMotor.idleMode = CANSparkMax.IdleMode.kBrake
        //intakeTwoMotor.idleMode = CANSparkMax.IdleMode.kBrake
        intakeTwoMotor.inverted = false
        elbowMotor.idleMode = CANSparkMax.IdleMode.kBrake
        elevatorMotor.idleMode = CANSparkMax.IdleMode.kBrake

        intakeOneMotor.inverted = false
        intakeTwoMotor.inverted = true

        //intakeOneMotor.setSmartCurrentLimit(IntakeConstants.intakeMotorsCurrentLimit)
        //intakeTwoMotor.setSmartCurrentLimit(IntakeConstants.intakeMotorsCurrentLimit)
        wristMotor.setSmartCurrentLimit(IntakeConstants.pitchMotorCurrentLimit)

        intakeOneMotor.burnFlash()
        intakeTwoMotor.burnFlash()
        wristMotor.burnFlash()
        elbowMotor.burnFlash()
        elevatorMotor.burnFlash()
        //everything else:
        // TODO: Elevator conversion factors have been tuned
        elbowEncoder.positionConversionFactor = 2.0 * Math.PI
        elbowEncoder.velocityConversionFactor = (2.0 *Math.PI)/60
        elbowEncoder.inverted = true

        elevatorEncoder.positionConversionFactor =  (0.003010870139 * 2.4) / 60.0
        elevatorEncoder.velocityConversionFactor = (0.003010870139 * 2.4) / 60.0

        wristEncoder.positionConversionFactor = 2.0 * Math.PI
        wristEncoder.velocityConversionFactor = (2.0 * Math.PI)/ 60.0
        wristEncoder.inverted = false

        Telemetry.elevatorP.set(ArmConstants.elevatorP)
        Telemetry.elevatorI.set(ArmConstants.elevatorI)
        Telemetry.elevatorD.set(ArmConstants.elevatorD)
        Telemetry.elbowP.set(ArmConstants.elbowP)
        Telemetry.elbowI.set(ArmConstants.elbowI)
        Telemetry.elbowD.set(ArmConstants.elbowD)
        Telemetry.wristP.set(ArmConstants.wristP)
        Telemetry.wristI.set(ArmConstants.wristI)
        Telemetry.wristD.set(ArmConstants.wristD)


        // TODO: wristEncoder position and velocity conversion factor
    }


    val elbowPositionFirst get() = Rotation2d(elbowEncoder.position).plus(Rotation2d(absoluteWristPosition))

    val elbowPositionRadians get() = elbowPositionFirst.minus(Rotation2d(ArmConstants.elbowEncoderPosOffset)).radians
    val elevatorPositionMeters get() = elevatorEncoder.position

    //intake
    val absoluteWristPosition get() = Rotation2d(-wristEncoder.position).minus(Rotation2d(ArmConstants.wristEncoderPosOffset)).radians


    /** This is the pitch with taking consideration to the position of the elbow.
     * 0 degrees means the intake is parallel to the ground.
     */
    //val relativeWristPosition get() = Rotation2d(wristEncoder.position).minus(Rotation2d(ArmConstants.wristEncoderPosOffset)).radians

    /** This value sets the voltage for the intake motors. Positive value will spin the wheels inward
     * and pick objects up.
     */
    var intakesVoltage = 0.0
        set(x: Double) {
            field = x
            intakeOneMotor.setVoltage(x)
            intakeTwoMotor.setVoltage(x)
        }

    var wristVoltage = 0.0
        set(x: Double) {
            field = x
            wristMotor.setVoltage(x)
        }

    var elbowVoltage = 0.0
        set(x: Double) {
            field = x
            elbowMotor.setVoltage(x)
        }

    var elevatorVoltage = 0.0
        set(x: Double) {
            field = x
            Telemetry.elevatorVoltage.set(elevatorVoltage)
            if (!elevatorZeroed) {
                elevatorMotor.setVoltage(ArmConstants.elevatorZeroingVoltage)
            } else if (bottomHit && x < 0.0) {
                elevatorMotor.setVoltage(0.0)
            } else if (topHit && x > 0.0) {
                elevatorMotor.setVoltage(0.0)
            } else {
                elevatorMotor.setVoltage(x)
            }
        }

    override fun periodic() {
        super.periodic()
        if (!elevatorZeroed) {
            elevatorMotor.setVoltage(ArmConstants.elevatorZeroingVoltage)

        }
        // These are the voltages we set and will be sent to the elevator and elbow.
        if(bottomHit)
        {
            if (!elevatorZeroed) {
                elevatorEncoder.position = 0.0
            }
            elevatorZeroed = true

        }
        else if(topHit)
        {
            //elevatorEncoder.position = ArmConstants.elevatorMaxHeight
        }

        // Telemetry setting.
        Telemetry.elevatorZeroed.set(elevatorZeroed)
        Telemetry.elbowPosition.set(elbowPositionRadians)
        Telemetry.elevatorPosition.set(elevatorEncoder.position)
        Telemetry.elbowVelocity.set(elbowEncoder.velocity)
        Telemetry.topHit.set(topHit)
        Telemetry.bottomHit.set(bottomHit)
        Telemetry.wristVoltage.set(wristVoltage)
        Telemetry.elbowVoltage.set(elbowVoltage)

        Telemetry.intakeVoltage.set(intakesVoltage)
        Telemetry.elevatorPosition.set(elevatorPositionMeters)
        Telemetry.elevatorVelocity.set(elevatorEncoder.velocity)
        Telemetry.wristPosition.set(absoluteWristPosition)

        Telemetry.cubeNT.set(CommandValues.cube)
        Telemetry.coneNT.set(CommandValues.cone)
        Telemetry.floorNT.set(CommandValues.floor)
        Telemetry.chuteNT.set(CommandValues.chute)
        Telemetry.pickupNT.set(CommandValues.pickup)
        Telemetry.middlePlaceNT.set(CommandValues.middlePlace)
        Telemetry.highPlaceNT.set(CommandValues.highPlace)
        Telemetry.groundNT.set(CommandValues.ground)

        Telemetry.visionNT.set(CommandValues.vision)

        ArmConstants.elevatorP = Telemetry.elevatorP.get()
        ArmConstants.elevatorI = Telemetry.elevatorI.get()
        ArmConstants.elevatorD = Telemetry.elevatorD.get()
        ArmConstants.elbowP = Telemetry.elbowP.get()
        ArmConstants.elbowI = Telemetry.elbowI.get()
        ArmConstants.elbowD = Telemetry.elbowD.get()
        ArmConstants.wristP = Telemetry.wristP.get()
        ArmConstants.wristI = Telemetry.wristI.get()
        ArmConstants.wristD = Telemetry.wristD.get()



    }
//    fun NTPnP(pnp: PickAndPlaceSubsystem, controller: XboxController): Command {
//
//        return SetPickAndPlacePosition(
//            true,
//            pnp,
//            {
//                Telemetry.elevatorValue.get()
//            }, // elevator
//            { Telemetry.elbowValue.get() }, // elbow
//            { Telemetry.wristValue.get() }, // wrist
//            { controller.leftTriggerAxis * 12.0 } // intake
//        )
//    }

}