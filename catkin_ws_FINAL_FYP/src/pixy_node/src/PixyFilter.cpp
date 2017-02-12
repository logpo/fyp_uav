#include "PixyFilter.h"
#include <algorithm>
#include <stdio.h>
#include <iostream>
using namespace std;

//COnstructor, only type
/*
PixyFilter::PixyFilter(unsigned size){
  filter_size = size;
  tol = 0.1;
  count_error = 0;
}*/

//Setter function
void PixyFilter::setFilterSize(unsigned size) {
  x_filter.setFilterSize(size);
  y_filter.setFilterSize(size);
  h_filter.setFilterSize(size);
  w_filter.setFilterSize(size);
}
void PixyFilter::setTolerance(float tolerance) {
  x_filter.setTolerance(tolerance);
  y_filter.setTolerance(tolerance);
  h_filter.setTolerance(tolerance);
  w_filter.setTolerance(tolerance);

}

void PixyFilter::clearFilter(void) {
  /* Function clears filter. Use after not having good data for some period*/
  x_filter.clearFilter();
  y_filter.clearFilter();
  h_filter.clearFilter();
  w_filter.clearFilter();

}

bool PixyFilter::addElement(uint32_t x_val, uint32_t y_val, uint32_t h_val, uint32_t w_val) {
  x_filter.addElement(static_cast<float>(x_val));
  y_filter.addElement(static_cast<float>(y_val));
  h_filter.addElement(static_cast<float>(h_val));
  w_filter.addElement(static_cast<float>(w_val));

  return true;
}

float PixyFilter::getLastValue(void) {
  //Returns the latest value added
  //return buffer[0];
  return 0;
}

float PixyFilter::getLastX_Value(void){
  return x_filter.getLastValue();
}
float PixyFilter::getLastY_Value(void){
  return y_filter.getLastValue();

}
float PixyFilter::getLastH_Value(void){
  return h_filter.getLastValue();
}
float PixyFilter::getLastW_Value(void){
  return w_filter.getLastValue();
}
