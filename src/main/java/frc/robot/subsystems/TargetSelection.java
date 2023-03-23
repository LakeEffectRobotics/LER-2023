package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.TargetSelection.Type.*;

import java.util.Map;

import static frc.robot.subsystems.TargetSelection.Height.*;

public class TargetSelection extends SubsystemBase {
    private ShuffleboardLayout layout = Shuffleboard.getTab("target").getLayout("target", BuiltInLayouts.kGrid).withSize(3, 3);

    public TargetSelection() {

    }

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
        { new Node(CONE, HIGH, 0, 0), new Node(CUBE, HIGH, 1, 0), new Node(CONE, HIGH, 2, 0) },
        { new Node(CONE, MID, 0, 1), new Node(CUBE, MID, 1, 1), new Node(CONE, MID, 2, 1) },
        { new Node(CONE, LOW, 0, 2), new Node(CUBE, LOW, 1, 2), new Node(CONE, LOW, 2, 2) },
    };

    // Actual selection location is stored as x, y locally
    private int selectedRow = 0;
    private int selectedCol = 0;
    /**
     * Currently selected Node, defaults to 0,0
     * TODO: Pick a better starting node?
     */
    private Node selectedNode = GRID[selectedRow][selectedCol];

    /**
     * Get the currently selected Node
     */
    public Node getSelectedNode() {
        return selectedNode;
    }

    /**
     * Select a specific node
     * @param height Height of node
     * @param col Grid column of Node. 0 is left of field from the robot's perspective
     */
    public void selectedNode(Height height, int col) {
        // Bound column input to grid size
        col = bound(col, 0, GRID[0].length);

        // Determine the row index based on provided height
        switch(height){
            case HIGH:
                selectedRow = 0;
                selectedCol = col;
                break;
            case MID:
                selectedRow = 1;
                selectedCol = col;
                break;
            case LOW:
                selectedRow = 2;
                selectedCol = col;
                break;
        }

        // Select node from provided values
        selectedNode = GRID[selectedRow][selectedCol];
    }

    /**
     * Raise the selected Node up one height
     */
    public void selectionUp(){
        updateEntry(selectedNode.getEntry(), false);

        selectedRow = bound(selectedRow + 1, 0, GRID.length-1);
        selectedNode = GRID[selectedRow][selectedCol];

        updateEntry(selectedNode.getEntry(), true);
        SmartDashboard.putString("HEIGHT", selectedNode.height.name());
    }
    
    /**
     * Lower the selected Node down one height
     */
    public void selectionDown(){
        updateEntry(selectedNode.getEntry(), false);

        selectedRow = bound(selectedRow - 1, 0, GRID.length-1);
        selectedNode = GRID[selectedRow][selectedCol];

        updateEntry(selectedNode.getEntry(), true);
        SmartDashboard.putString("HEIGHT", selectedNode.height.name());
    }
    
    /**
     * Move the selected Node left one column
     */
    public void selectionLeft(){
        updateEntry(selectedNode.getEntry(), false);

        selectedCol = bound(selectedCol - 1, 0, GRID[selectedRow].length-1);
        selectedNode = GRID[selectedRow][selectedCol];

        updateEntry(selectedNode.getEntry(), true);
        SmartDashboard.putString("Object Type", selectedNode.type.name());
    }
    
    /**
     * Move the selected Node right one column
     */
    public void selectionRight(){
        updateEntry(selectedNode.getEntry(), false);

        selectedCol = bound(selectedCol + 1, 0, GRID[selectedRow].length-1);
        selectedNode = GRID[selectedRow][selectedCol];

        updateEntry(selectedNode.getEntry(), true);
        SmartDashboard.putString("Object Type", selectedNode.type.name());
    }

    private static int bound(int val, int min, int max){
        if(val < min)
            return min;
        if (val > max)
            return max;
        else
            return val;
    }

    private void updateEntry(GenericEntry entry, boolean value) {
        entry.setBoolean(value);
    }

    /**
     * Class representing a node which can be targeted
     */
    public static class Node {

        private Type type;
        private Height height;
        private GenericEntry entry;

        /**
         * Create a new Node
         * This is public to allow for usage in autonomous
         * @param type Type of game piece scored on the node
         * @param height Height of the node
         */
        public Node(Type type, Height height, int column, int row) {
            this.type = type;
            this.height = height;
            entry = Shuffleboard
                .getTab("target")
                .getLayout("target", BuiltInLayouts.kGrid)
                .add(height.toString() + " " + type.toString() + " " + column, false)
                .withPosition(column, row)
                .getEntry();
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

        public GenericEntry getEntry() {
            return entry;
        }
    }
}
