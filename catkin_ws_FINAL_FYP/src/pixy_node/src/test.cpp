#include "Undistort.cpp"

using namespace std;
int main() {


Undistort calibrate;

std::cout << "Origin X " << calibrate.findNewXVal(0,0) << std::endl;
std::cout << "Origin Y " << calibrate.findNewXVal(0,0) << std::endl;
std::cout << "Top L X " << calibrate.findNewXVal(0,198) << std::endl;
std::cout << "Top L Y " << calibrate.findNewXVal(0,198) << std::endl;
std::cout << "B L X " << calibrate.findNewXVal(318,0) << std::endl;
std::cout << "B R Y " << calibrate.findNewXVal(318,0) << std::endl;
std::cout << "Top R X " << calibrate.findNewXVal(318,198) << std::endl;
std::cout << "Top R Y " << calibrate.findNewXVal(320,300) << std::endl;

return 0;
}
