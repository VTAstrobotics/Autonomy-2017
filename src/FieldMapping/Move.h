#ifndef MOVE_H
#define MOVE_H

// right means to the right if the robot is facing mining area
enum Direction {RIGHT, LEFT, TOWARDS_MINE, TOWARDS_BIN};

/*
This class will define the next move of the robot. We will
gain access to the map by inheriting from the Map class
*/
class Move : Map{
private:
  Direction direction; // direction of the move
  double distance;
  void distance(int robotRow, int robotCol, int mineRow,int mineCol);
public:
  Move(); // constructor

  // accessor methods
  void setDirection(Direction datWay);
  void setDistance(double dist);

  // mutator methods
  Direction getDirection();
  double getDistance();
  void choseMove(); // decides on which direction to move in

};


#endif
