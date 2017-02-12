//Function to give avg of reading

#include <deque>
#include <stdio.h>
#include <iostream>
using namespace std;

class Filter{
public:
  void addElement(float reading) {
    //always add new element
    history.push_front(reading);

    if (history.size()<filter_size) {
      //Add Automatically for first few rounds, if too small return without removing
      return;
    }
    history.pop_back();
    return;
  }

  float get_average(void){
    float total =0;

    for (unsigned i = 0; i < history.size(); i++) {
      total = total +history[i];
    }

    return total / history.size();

  };

  Filter(short size){
    filter_size=size;
  };

private:
  short filter_size;
  deque<float> history;

};
