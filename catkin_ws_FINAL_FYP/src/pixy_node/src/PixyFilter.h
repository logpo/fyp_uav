//Class for putting together the four filters allocated to each block

//Created by Theo Drissner-Devine 14/07/16

#ifndef PixyFilter_PixyFilter_h
#define PixyFilter_PixyFilter_h
#include <stdint.h>

#include <deque>
#include "OutlierFilter.h"

using namespace std;

class PixyFilter{
public:
  void setFilterSize(unsigned size);
  bool addElement(uint32_t x_val, uint32_t y_val, uint32_t h_val, uint32_t w_val);
  void setTolerance(float tolerance);
  void clearFilter(void);
  float getLastX_Value(void);
  float getLastY_Value(void);
  float getLastH_Value(void);
  float getLastW_Value(void);
  float getLastValue(void);


  PixyFilter(unsigned size) : x_filter(size), y_filter(size), h_filter(size), w_filter(size) { tol = 0.1; count_error = 0;}

protected:

  OutlierFilter x_filter;//X offset in pix
  OutlierFilter y_filter;
  OutlierFilter h_filter;//Height
  OutlierFilter w_filter;


private:
  short count_error;//Counter of how many times the added value has been rejected in a row. If too great, discard
  unsigned filter_size;
  float tol;// A hardcoded tolerance to always ensure the good values can get in. 1 =100%


};

#endif
