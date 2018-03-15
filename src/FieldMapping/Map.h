#include"AutonomyMap.h"
#ifndef MAP_H // check to make sure the header isn't included twice
#define  MAP_H
/*
this class represents the map to be used in the AutonomyMap
algorithm. Each index of the map represents 0.91 sq ft

*/
class Map{
  // private members to be used for the class
private:
  const int COLS = 13; // number of columns the field is divided into
  const int ROWS = 25; // number of rows the field is divided into
  const int MINIG_ROW = 15; // row where mining field starts
  const int startRow; // robots starting row
  const int startCol; // robots strating col
  const char obstacle = 'X'; // obstacle to be placed on the map
  const char openSpace = ' '; // space to fill the map with initially
  char[ROWS][COLS] map; // map of the field
  Robot thisRobot; // robot to be used

public:
  Map(Robot& robot); // constructor, initiallizes the map
  char charAt(int row, int col); // returns char at given index
  void setSquare(int row, int col, char piece); // sets index to a char
};

#endif
