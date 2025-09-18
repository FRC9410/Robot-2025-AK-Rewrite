
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BiConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MapConstants;
import frc.robot.Constants.ScoringConstants;

public class Dashboard extends SubsystemBase {
  private final NetworkTableInstance inst;
  private final NetworkTable table;
  private final NetworkTable drivingTable;
  private Auto auto;
  private boolean isClimbing;
  private final BiConsumer<String, Object> updateData;
  
  /** Creates a new Dashboard. */
  public Dashboard(BiConsumer<String, Object> updateData) {
    this.updateData = updateData;
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("Scoring");
    drivingTable = inst.getTable("Driving PIDs");

    table.getEntry("leftL1").setBoolean(false);
    table.getEntry("leftL2").setBoolean(false);
    table.getEntry("leftL3").setBoolean(false);
    table.getEntry("leftL4").setBoolean(false);
    table.getEntry("rightL1").setBoolean(false);
    table.getEntry("rightL2").setBoolean(false);
    table.getEntry("rightL3").setBoolean(false);
    table.getEntry("rightL4").setBoolean(false);
    table.getEntry("isClimbing").setBoolean(false);
    table.getEntry("blueLeft").setBoolean(false);
    table.getEntry("blueRight").setBoolean(false);
    table.getEntry("redLeft").setBoolean(false);
    table.getEntry("redRight").setBoolean(false);
    table.getEntry("autoElevator").setBoolean(false);
    drivingTable.getEntry("useIncreasedFfValue").setBoolean(false);

    selectedCoralPosition = null;
  }

  private enum CoralPositions {
    NO_POSITION,
    LEFT_L1,
    LEFT_L2,
    LEFT_L3,
    LEFT_L4,
    RIGHT_L1,
    RIGHT_L2,
    RIGHT_L3,
    RIGHT_L4
  }

  private CoralPositions selectedCoralPosition = CoralPositions.NO_POSITION;

  @Override
  public void periodic() {
    final CoralPositions newCoralSelection = getCoralSelection();
    if (newCoralSelection != selectedCoralPosition) {
      setCoralSelection(newCoralSelection);
    }

    final Auto newAuto = getAuto();
    if (newAuto!= auto) {
      setAuto(newAuto);
    }

    isClimbing = table.getEntry("isClimbing").getBoolean(false);

    updateData.accept(MapConstants.TARGET_POSE, getScoringPose());
    updateData.accept(MapConstants.ELEVATOR_POSITION, getSelectedCoralPosition());
    updateData.accept(MapConstants.AUTO, getAutoFromDash());
  }

  // public ReefSide getReefSide() {
  //   if (table.getEntry("front").getBoolean(false) && reefSide != ReefSide.FRONT) {
  //     return ReefSide.FRONT;
  //   } else if (table.getEntry("front_left").getBoolean(false) && reefSide != ReefSide.FRONT_LEFT) {
  //     return ReefSide.FRONT_LEFT;
  //   } else if (table.getEntry("front_right").getBoolean(false) && reefSide != ReefSide.FRONT_RIGHT) {
  //     return ReefSide.FRONT_RIGHT;
  //   } else if (table.getEntry("back").getBoolean(false) && reefSide != ReefSide.BACK) {
  //     return ReefSide.BACK;
  //   } else if (table.getEntry("back_left").getBoolean(false) && reefSide != ReefSide.BACK_LEFT) {
  //     return ReefSide.BACK_LEFT;
  //   } else if (table.getEntry("back_right").getBoolean(false) && reefSide != ReefSide.BACK_RIGHT) {
  //     return ReefSide.BACK_RIGHT;
  //   }
  //   return reefSide;
  // }

  // public CoralSide getCoralSide() {
  //   if (table.getEntry("left").getBoolean(false) && coralSide != CoralSide.LEFT) {
  //     return CoralSide.LEFT;
  //   } else if (table.getEntry("right").getBoolean(false) && coralSide != CoralSide.RIGHT) {
  //     return CoralSide.RIGHT;
  //   }
  //   return coralSide;
  // }

  public CoralPositions getCoralSelection() {
    if (table.getEntry("l1").getBoolean(false) && selectedCoralPosition != CoralPositions.LEFT_L1) {
      return CoralPositions.LEFT_L1;
    } else if (table.getEntry("l2").getBoolean(false) && selectedCoralPosition != CoralPositions.LEFT_L2) {
      return CoralPositions.LEFT_L2;
    } else if (table.getEntry("l3").getBoolean(false) && selectedCoralPosition != CoralPositions.LEFT_L3) {
      return CoralPositions.LEFT_L3;
    } else if (table.getEntry("l4").getBoolean(false) && selectedCoralPosition != CoralPositions.LEFT_L4) {
      return CoralPositions.LEFT_L4;
    } else if (table.getEntry("l1").getBoolean(false) && selectedCoralPosition != CoralPositions.RIGHT_L1) {
      return CoralPositions.RIGHT_L1;
    } else if (table.getEntry("l2").getBoolean(false) && selectedCoralPosition != CoralPositions.RIGHT_L2) {
      return CoralPositions.RIGHT_L2;
    } else if (table.getEntry("l3").getBoolean(false) && selectedCoralPosition != CoralPositions.RIGHT_L3) {
      return CoralPositions.RIGHT_L3;
    } else if (table.getEntry("l4").getBoolean(false) && selectedCoralPosition != CoralPositions.RIGHT_L4) {
      return CoralPositions.RIGHT_L4;
    }
    return selectedCoralPosition;
  }

  public Auto getAuto() {
    if (table.getEntry("blueLeft").getBoolean(false) && auto != Auto.BLUE_LEFT) {
      return Auto.BLUE_LEFT;
    } else if (table.getEntry("blueRight").getBoolean(false) && auto != Auto.BLUE_RIGHT) {
      return Auto.BLUE_RIGHT;
    } else if (table.getEntry("redLeft").getBoolean(false) && auto != Auto.RED_LEFT) {
      return Auto.RED_LEFT;
    } else if (table.getEntry("redRight").getBoolean(false) && auto != Auto.RED_RIGHT) {
      return Auto.RED_RIGHT;
    }
    return auto;
  }

  public Auto getAutoFromDash() {
    if (table.getEntry("blueLeft").getBoolean(false)) {
      return Auto.BLUE_LEFT;
    } else if (table.getEntry("blueRight").getBoolean(false)) {
      return Auto.BLUE_RIGHT;
    } else if (table.getEntry("redLeft").getBoolean(false)) {
      return Auto.RED_LEFT;
    } else if (table.getEntry("redRight").getBoolean(false)) {
      return Auto.RED_RIGHT;
    }
    return auto;
  }

  public boolean getIsClimbing () {
    return isClimbing;
  }

  public void clearSelections() {
    table.getEntry("front").setBoolean(false);
    table.getEntry("front_left").setBoolean(false);
    table.getEntry("front_right").setBoolean(false);
    table.getEntry("back").setBoolean(false);
    table.getEntry("back_left").setBoolean(false);
    table.getEntry("back_right").setBoolean(false);
    table.getEntry("left").setBoolean(false);
    table.getEntry("right").setBoolean(false);
    // table.getEntry("l1").setBoolean(false);
    // table.getEntry("l2").setBoolean(false);
    // table.getEntry("l3").setBoolean(false);
    // table.getEntry("l4").setBoolean(false);
    table.getEntry("redLeft").setBoolean(false);
    table.getEntry("redRight").setBoolean(false);
    table.getEntry("blueLeft").setBoolean(false);
    table.getEntry("blueRight").setBoolean(false);
    table.getEntry("isRed").setBoolean(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);

    selectedCoralPosition = null;
    auto = null;
  }

  public void clearCoralSelections() {
    table.getEntry("leftL1").setBoolean(false);
    table.getEntry("leftL2").setBoolean(false);
    table.getEntry("leftL3").setBoolean(false);
    table.getEntry("leftL4").setBoolean(false);
    table.getEntry("rightL1").setBoolean(false);
    table.getEntry("rightL2").setBoolean(false);
    table.getEntry("rightL3").setBoolean(false);
    table.getEntry("rightL4").setBoolean(false);
  }

  public void clearAutoSelections() {
    table.getEntry("blueLeft").setBoolean(false);
    table.getEntry("blueRight").setBoolean(false);
    table.getEntry("redLeft").setBoolean(false);
    table.getEntry("redRight").setBoolean(false);
  }

  // public void setReefSide(ReefSide side) {
  //   if (side == null) {
  //     return;
  //   }

  //   clearReefSelections();
  //   reefSide = side;
  //   switch (side) {
  //     case FRONT:
  //       table.getEntry("front").setBoolean(true);
  //       break;
  //     case FRONT_LEFT:
  //       table.getEntry("front_left").setBoolean(true);
  //       break;
  //     case FRONT_RIGHT:
  //       table.getEntry("front_right").setBoolean(true);
  //       break;
  //     case BACK:
  //       table.getEntry("back").setBoolean(true);
  //       break;
  //     case BACK_LEFT:
  //       table.getEntry("back_left").setBoolean(true);
  //       break;
  //     case BACK_RIGHT:
  //       table.getEntry("back_right").setBoolean(true);
  //       break;
  //   }
  // }


  public void setCoralSelection(CoralPositions position) {
    if (position == null) {
      return;
    }

    clearCoralSelections();
    selectedCoralPosition = position;
    switch (position) {
      case RIGHT_L1:
        table.getEntry("l1").setBoolean(true);
        break;
      case RIGHT_L2:
        table.getEntry("l2").setBoolean(true);
        break;
      case RIGHT_L3:
        table.getEntry("l3").setBoolean(true);
        break;
      case RIGHT_L4:
        table.getEntry("l4").setBoolean(true);
        break;
      case LEFT_L1:
        table.getEntry("l1").setBoolean(true);
        break;
      case LEFT_L2:
        table.getEntry("l2").setBoolean(true);
        break;
      case LEFT_L3:
        table.getEntry("l3").setBoolean(true);
        break;
      case LEFT_L4:
        table.getEntry("l4").setBoolean(true);
        break;
    }
  }

  public void setAuto(Auto auto) {
    if (auto == null) {
      return;
    }

    clearAutoSelections();
    this.auto = auto;
    switch (auto) {
      case RED_LEFT:
        table.getEntry("redLeft").setBoolean(true);
        break;
      case RED_RIGHT:
        table.getEntry("redRight").setBoolean(true);
        break;
      case BLUE_LEFT:
        table.getEntry("blueLeft").setBoolean(true);
        break;
      case BLUE_RIGHT:
        table.getEntry("blueRight").setBoolean(true);
        break;
    }
  }

  // public Pose2d getScoringPose () {
  //   final boolean isRed = drivingTable.getEntry("invert paths").getBoolean(false);


  //   if (reefSide == null || coralSide == null) {
  //     return null;
  //   }

  //   if (isRed) {
  //     if (coralSide == CoralSide.LEFT) {
  //       switch (reefSide) {
  //         case FRONT:
  //           return Constants.ScoringConstants.RED_FRONT_LEFT;
  //         case FRONT_LEFT:
  //           return Constants.ScoringConstants.RED_FRONT_LEFT_LEFT;
  //         case FRONT_RIGHT:
  //           return Constants.ScoringConstants.RED_FRONT_RIGHT_LEFT;
  //         case BACK:
  //           return Constants.ScoringConstants.RED_BACK_LEFT;
  //         case BACK_LEFT:
  //           return Constants.ScoringConstants.RED_BACK_LEFT_LEFT;
  //         case BACK_RIGHT:
  //           return Constants.ScoringConstants.RED_BACK_RIGHT_LEFT;
  //         default:
  //           return null;
  //       }
  //     } else {
  //       switch (reefSide) {
  //         case FRONT:
  //           return Constants.ScoringConstants.RED_FRONT_RIGHT;
  //         case FRONT_LEFT:
  //           return Constants.ScoringConstants.RED_FRONT_LEFT_RIGHT;
  //         case FRONT_RIGHT:
  //           return Constants.ScoringConstants.RED_FRONT_RIGHT_RIGHT;
  //         case BACK:
  //           return Constants.ScoringConstants.RED_BACK_RIGHT;
  //         case BACK_LEFT:
  //           return Constants.ScoringConstants.RED_BACK_LEFT_RIGHT;
  //         case BACK_RIGHT:
  //           return Constants.ScoringConstants.RED_BACK_RIGHT_RIGHT;
  //         default:
  //           return null;
  //       }
  //     }
  //   } else {
  //     if (coralSide == CoralSide.LEFT) {
  //       switch (reefSide) {
  //         case FRONT:
  //           return Constants.ScoringConstants.BLUE_FRONT_LEFT;
  //         case FRONT_LEFT:
  //           return Constants.ScoringConstants.BLUE_FRONT_LEFT_LEFT;
  //         case FRONT_RIGHT:
  //           return Constants.ScoringConstants.BLUE_FRONT_RIGHT_LEFT;
  //         case BACK:
  //           return Constants.ScoringConstants.BLUE_BACK_LEFT;
  //         case BACK_LEFT:
  //           return Constants.ScoringConstants.BLUE_BACK_LEFT_LEFT;
  //         case BACK_RIGHT:
  //           return Constants.ScoringConstants.BLUE_BACK_RIGHT_LEFT;
  //         default:
  //           return null;
  //       }
  //     } else {
  //       switch (reefSide) {
  //         case FRONT:
  //           return Constants.ScoringConstants.BLUE_FRONT_RIGHT;
  //         case FRONT_LEFT:
  //           return Constants.ScoringConstants.BLUE_FRONT_LEFT_RIGHT;
  //         case FRONT_RIGHT:
  //           return Constants.ScoringConstants.BLUE_FRONT_RIGHT_RIGHT;
  //         case BACK:
  //           return Constants.ScoringConstants.BLUE_BACK_RIGHT;
  //         case BACK_LEFT:
  //           return Constants.ScoringConstants.BLUE_BACK_LEFT_RIGHT;
  //         case BACK_RIGHT:
  //           return Constants.ScoringConstants.BLUE_BACK_RIGHT_RIGHT;
  //         default:
  //           return null;
  //       }
  //     }
  //   }
  // }

  public CoralPositions getSelectedCoralPosition() {
    switch (selectedCoralPosition) {
      case LEFT_L1:
        return CoralPositions.LEFT_L1;
      case LEFT_L2:
        return CoralPositions.LEFT_L2;
      case LEFT_L3:
        return CoralPositions.LEFT_L3;
      case LEFT_L4:
        return CoralPositions.LEFT_L4;
      case RIGHT_L1:
        return CoralPositions.RIGHT_L1;
      case RIGHT_L2:
        return CoralPositions.RIGHT_L2;
      case RIGHT_L3:
        return CoralPositions.RIGHT_L3;
      case RIGHT_L4:
        return CoralPositions.RIGHT_L4;
      default: 
        return CoralPositions.NO_POSITION;
    }
  }

  // public Auto getSelectedAuto() {
  //   return auto;
  // }

  // public double getSelectedCoralLevel() {
  //   if (coralLevel == null) {
  //     return Constants.ElevatorConstants.HOME_POSITION;
  //   }

  //   switch (coralLevel) {
  //     case L1:
  //       return Constants.ElevatorConstants.L1_SCORE_POSITION;
  //     case L2:
  //       return Constants.ElevatorConstants.L2_SCORE_POSITION;
  //     case L3:
  //       return Constants.ElevatorConstants.L3_SCORE_POSITION;
  //     case L4:
  //       return Constants.ElevatorConstants.L4_SCORE_POSITION;
  //     default:
  //       return Constants.ElevatorConstants.HOME_POSITION;
  //   }
  // }

  // public enum ReefSide {
  //   FRONT,
  //   FRONT_LEFT,
  //   FRONT_RIGHT,
  //   BACK,
  //   BACK_LEFT,
  //   BACK_RIGHT
  // }

  // public enum CoralSide {
  //   LEFT,
  //   RIGHT
  // }

  // public enum CoralLevel {
  //   L1,
  //   L2,
  //   L3,
  //   L4
  // }

  public enum Auto {
    RED_LEFT,
    RED_RIGHT,
    BLUE_LEFT,
    BLUE_RIGHT
  }
}
