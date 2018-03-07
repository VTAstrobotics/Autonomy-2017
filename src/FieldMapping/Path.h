#include "AutonomyMap.h"

#ifndef PATH_H
#define PATH_H

/**
this class will contain a series of moves which will constitue
a path for the robot to move in
*/
class Path{
private:
  double pathDistance;
  vector<Move> thisPath; // creates a vector full of moves

public:
  Path(); // constructor for class
  void addMove(Move aMove);

  // accessor methods
  vector<Move> getPath();
  double getDistance();


};

#endif
