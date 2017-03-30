// For all movement state, if the error between the current position and desired position is within a set threshold, move to next state

// Message type of publishing the trajectory to moveit would be a vector of structs containing:
// vector of 6 doubles (representing a list of joint angle position on the path)
// A double for time

// For pickup and drop actions, apply actions for a constant time then move to next state

// Also publishing the current state for other nodes at 50ms

// State 1:
// Initialize and Move to PCB

// State 2:
// At PCB, detect PCB position using tf with the computer vision node, cv publish transform broadcaster, takes input as a raw image stream
// Have the transformation from PCB available using can_tranform(). If yes, move to state 3

// State 3:
// Move to SOIC, move to state 4

// State 4:
// At SOIC, detect SOIC using tf with the computer vision node
// Have the transformation from SOIC available using can_transform(). If yes, move to state 5

// State 5:
// Move to syringe position
// Location of syringe will be hardcoded

// State 6:
// pick up the syringe 
// Move to state 6

// State 6:
// Move to PCB
// Move to state 7

// State 7:
// Apply solder paste
// Move to state 8

// State 8:
// Move to solder paste area
// Move to state 9

// State 9:
// Release solder paste tool
// Move to state 10

// State 10:
// Move to suction cup
// Suction cup location hardcoded
// Move to state 11

// State 11:
// Pick up suction cup
// Move to state 12

// State 12:
// Move to SOIC
// Move to state 13

// State 13
// Pick up SOIC
// Move to state 14

// State 14
// Move to PCB
// Move to state 15

// State 15
// Place SOIC
// Move to state 16

// State 16
// Move to suction cup holder
// Move to state 17

// State 17
// Drop suction cup holder
// Move to state 18

// state 18
// Move to hot air pencil
// Move to state 19

// state 19
// Pick up hot air pencil
// Move to state 20

// state 20
// Move to PCB
// move to state 21

// state 21
// Apply hot air
// move to state 22

// state 22
// move to hot air pencil holder
// move to state 23

// state 23
// drop hot air pencil
// move to 24

// state 24
// return home

// state 25
// terminate
















int main() {

// Computer vision to detect all positions

// Pick up syringe

// Move to PCB and apply solder paste

// Switch tool too suction cup

// Move to and Pick up SOIC

// Move to PCB and place SOIC

// Switch tool to hot air pencil

// Move to PCB

// Flow solder

// Put tool away and return home

return 0;
}
