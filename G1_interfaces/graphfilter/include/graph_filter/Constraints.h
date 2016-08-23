#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include <stdio.h>
#include <map>
#include <iostream>
#include <vector>

class Constraints {
  
  protected:
    // template vertex number to requred constraints success number
    std::map<int, int> req_nums;
    //checked scene graph number to success numbers
    // std::map<int, int> success_nums;
  public:

    Constraints(std::map<int, int> & req_nums_);
    ~Constraints();

    // void reset();
    // virtual int check(int a);
    // virtual bool check(int a, int b);
    virtual bool passLabel(int sid, int l);
    virtual std::string getClassName();
    std::vector<int> getVerticeList();
};

#endif