#ifndef MOVE_H
#define MOVE_H

// right means to the right if the robot is facing mining area
enum Direction {RIGHT, LEFT, TOWARDS_MINE, TOWARDS_BIN, };

class Move{
private:
  Direction direction; // direction of the move
  double distance;

public:
  Move(); // constructor

  // accessor methods
  void setDirection(Direction datWay);
  void setDistance(double dist);

  // mutator methods
  Direction getDirection();
  double getDistance();

};


#endif
