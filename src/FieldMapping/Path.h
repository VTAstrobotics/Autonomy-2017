#ifndef PATH_H
#define PATH_H

#include "AutonomyMap.h"
#include "Map.h"
#include "Robot.h"
#include "Move.h"

/**
this class will contain a series of moves which will constitue
a path for the robot to move in
*/
class Path{
private:
  vector<Move> thisPath; // creates a vector of moves

  // helper methods
  double calcDistance(int robotRow, int robotCol, int mineCol,  Map map);

public:
  Path(); // constructor for class
  void createPath(int miningCol, Map map, Robot robot); // create the best path to the position
  Direction Path::nextMove(int &robotRow, int &robotCol, int miningCol, Map map);
  void addMove(Move aMove); // adds a move to the path

  // accessor methods
  vector<Move> getPath();
  int getPathDistance(); // returns the distance of path

};
#endif
