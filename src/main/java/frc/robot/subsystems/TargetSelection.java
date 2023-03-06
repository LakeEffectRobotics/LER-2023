package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.TargetSelection.Type.*;
import static frc.robot.subsystems.TargetSelection.Height.*;

public class TargetSelection extends SubsystemBase {
    
    /**
     * Enum representing the type of game piece scored in the Node
     */
    public enum Type {
        CONE, CUBE
    }

    /**
     * Enum representing the height of the node, with LOW being on the floor
     */
    public enum Height {
        LOW, MID, HIGH
    }

    /**
     * Software representation of the Grid
     * Item at 0,0 represents the top-left scoring node from the robot's perspective
     * Currently this is rigged up to just have cone/cube at each height until we have time to develop vision-based scoring
     */
    private static final Node[][] GRID = new Node[][] {
        { new Node(CONE, HIGH), new Node(CUBE, HIGH) },
        { new Node(CONE, MID), new Node(CUBE, MID) },
        { new Node(CONE, LOW), new Node(CUBE, LOW) },
    };

    /**
     * Class representing a node which can be targeted
     */
    public static class Node {

        private Type type;
        private Height height;

        /**
         * Create a new Node
         * This is private to prevent creation outside of TargetSelection
         * @param type Type of game piece scored on the node
         * @param height Height of the node
         */
        private Node(Type type, Height height) {
            this.type = type;
            this.height = height;
        } 

        /**
         * Get the type of game piece scored on the node
         * @return Type of game piece
         */
        public Type getType() {
            return type;
        }

        /**
         * Get the height of the node off the floor
         * @return Height of the node
         */
        public Height getHeight() {
            return height;
        }
    }
}
