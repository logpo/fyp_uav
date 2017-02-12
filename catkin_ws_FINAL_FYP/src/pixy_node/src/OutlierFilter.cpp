#include "OutlierFilter.h"
#include <algorithm>
#include <stdio.h>
#include <iostream>
using namespace std;

//COnstructor, only type
OutlierFilter::OutlierFilter(unsigned size){
  filter_size = size;
  tol = 0.3;
  count_error = 0;
}

//Setter function
void OutlierFilter::setFilterSize(unsigned size) {
  filter_size = size;
}
void OutlierFilter::setTolerance(float tolerance) {
  tol = tolerance;
}

void OutlierFilter::clearFilter(void) {
  /* Function clears filter. Use after not having good data for some period*/
  buffer.clear();
}

bool OutlierFilter::addElement(const float reading) {
  /* function to check if element is valid and then add it if it is */
  float mean = 0;
  float total = 0;
  float IQR;
  deque<float> copy_filter;

  if (count_error >= filter_size -1) {
    buffer.clear();
    count_error = 0;
    //std::cout << "Error" << std::endl;
  }

  if (buffer.size()<filter_size) {
    /* Automatically add to the filter if it is too small, should only happen in
    first few cycles */
    count_error = 0;
    buffer.push_front(reading);
    return true;
  }
  buffer.pop_back();//Remove an element from the back

  copy_filter = buffer;

  //To find the mean of the measurements:
  //Iterate and add to the total of the past values

  for (unsigned i = 0; i < copy_filter.size(); i++) {
    total =  total + copy_filter[i];
  }

  mean = total / copy_filter.size();

  //Iterate through filter to find residual (value - mean)
  for (unsigned i = 0; i < copy_filter.size(); i++) {
    copy_filter[i] =  copy_filter[i] - mean;
  }

  //Sorts residuals in ascending order
  sort(copy_filter.begin(),copy_filter.end());

  float LQ = copy_filter[(copy_filter.size()/4)];// Find the lower quartile of range
  float UQ = *(copy_filter.begin() + (3 * copy_filter.size()/4));// Find the upper quartile of range

  IQR = UQ - LQ; //Find interquartile range

  float difference ;
  if (reading>mean) {
    difference = reading - mean;
  } else {
    //difference = mean - difference;
  }

  //If the reading is outside the range defined, just add the mean to the reading
  if (difference < (2 * IQR) + 2) {
    buffer.push_front(reading);
    count_error = 0;
    return true;

     //A check to ensure a good value always makes it in
  } else if ( difference < (tol * mean)) {
        buffer.push_front(reading);
        count_error = 0;
        return true;
  } else {
      buffer.push_front(mean);
     //std::cout << "Mean is: "<< mean << std::endl;
      //::cout << "IQR is:" << IQR << std::endl;
      /*
      std::cout << "size is: " << buffer.size()<<std::endl;
      std::cout << "reading is:"<<reading << std::endl;
      std::cout << "Not add" << std::endl;
      */
      count_error++;
      return false;
  }
}

float OutlierFilter::getLastValue(void) {
  //Returns the latest value added
  return buffer[0];
}
