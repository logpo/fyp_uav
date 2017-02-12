#include "Derivative.h"
#include <stdio.h>
#include <iostream>

using namespace std;

float Derivative::computeDerivative(float current) {
  /* Function takes a diference value and finds rate of change for previous few elements */
    float total = 0;

    if (history.size() == 0) {
      past_value = current;
      history.push_front(0); //Add newest element
      return 0;
    }

    float d = (current - past_value) / T;
    history.push_front(d); //Add newest element

    if (history.size()>buffer_size) {
      /*If filter is required size, remove an element, otherwise add until right size*/
      history.pop_back();
    }

    //Iterate and add to the total of the past derivative.

     int j=0;
     while (j<history.size()) {
       total = total + history[j];
       j++;
     }


     past_value = current;
     return total / history.size();

}

//Only Constructor
Derivative::Derivative(unsigned size){
  buffer_size = size;
  T=0.05;
}

//Setter functions
void Derivative::setT(float timestep){
  T=timestep;
}
void Derivative::setSize(unsigned size){
  buffer_size = size;
}
