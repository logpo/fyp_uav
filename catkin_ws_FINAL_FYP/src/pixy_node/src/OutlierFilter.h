//Class for excluding outliers in filter measurements

//Created by Theo Drissner-Devine 14/07/16

#ifndef OutlierFilter_OutlierFilter_h
#define OutlierFilter_OutlierFilter_h

#include <deque>

using namespace std;

class OutlierFilter{
  friend class PixyFilter;
public:
  void setFilterSize(unsigned size);
  bool addElement(float reading);
  void setTolerance(float tolerance);
  void clearFilter(void);
  float getLastValue(void);

  OutlierFilter(unsigned size);

protected:
  deque<float> buffer;
  deque<float>::iterator p;

private:
  short count_error;//Counter of how many times the added value has been rejected in a row. If too great, discard
  unsigned filter_size;
  float tol;// A hardcoded tolerance to always ensure the good values can get in. 1 =100%


};

#endif
