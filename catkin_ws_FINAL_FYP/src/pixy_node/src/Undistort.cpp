#include <stdio.h>
#include <fstream>
#include <istream>
#include <iostream>
#include <stdlib.h>

using namespace std;

class Undistort{
public:
  short findNewXVal(short old_x_val, short old_y_val){
    short new_x_val;

    if (old_y_val>198) {
      old_y_val = 198;
    }
    if (old_x_val>318) {
      old_x_val = 318;
    }
    //Work out coordinate postion in File
    int  offset = (old_x_val * 198) + old_y_val;

    //Read out new coordinate
    new_x_val = x_table[offset];

    //return new coordinate
    return new_x_val;

  }

  short findNewYVal(short old_x_val, short old_y_val){
    short new_y_val;

    if (old_y_val>198) {
      old_y_val = 198;
    }
    if (old_x_val>318) {
      old_x_val = 318;
    }

    //Work out coordinate postion in File
    int  offset = (old_x_val * 198) + old_y_val;
    //Read out new coordinate
    new_y_val = *(y_table+offset);

    //return new coordinate
    return  new_y_val;

  }

private:

  //Declare array that has all values
  short  x_table [62964], y_table [62964];

public:
  Undistort()


  //COnstructor of Undistort class, open file, read to array
  {
    int i = 0;

    //Read from x_file
    ifstream afile ("LUTx.txt",ios::in);
    if(afile.is_open()) {
      string buffer;
      while( getline(afile,buffer) ) {
        x_table[i] = atoi(buffer.c_str());
        i++;
      }
        afile.close();
      }
    else {
      cout << "Unable to open file" << endl;
    }

    i=0;
    //Read from y_file
    ifstream bfile ("LUTy.txt",ios::in);
    if(bfile.is_open()) {
      string buffer;
      while( getline(bfile,buffer) ) {
        y_table[i] = atoi(buffer.c_str());
        i++;
      }
        bfile.close();
      }
    else {
      cout << "Unable to open file" << endl;
    }
  };

};
