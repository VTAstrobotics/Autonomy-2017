#include "AutonomyMap.h"
#include "Map.h"
#include "Robot.h"
#include "Move.h"

#ifndef PATH_H
#define PATH_H

/**
this class will contain a series of moves which will constitue
a path for the robot to move in
*/
class Path{
private:
  vector<Move> thisPath; // creates a vector of moves

  // helper methods
  void calcDistance(int robotRow, int robotCol, int mineCol);
  void addMove(Move aMove); // adds a move to the path

public:
  Path(); // constructor for class
  void createPath(int miningCol); // create the best path to the position

  // accessor methods
  vector<Move> getPath();
  int getPathDistance(); // returns the distance of path

};
#endif
