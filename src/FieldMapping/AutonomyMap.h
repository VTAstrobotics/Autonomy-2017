#include<iostream> // allows input and output
#include<String> // includes the String class to be used
#include<fstream> // allows filestream to be used
#include<vector> // this will allow us to use a vector
#include<math.h>

using std::endl;
using std::cout;
using std::cin;

// right means to the right if the robot is facing mining area
enum Direction {RIGHT, LEFT, TOWARDS_MINE, TOWARDS_BIN, NOT_DETERMINED};
