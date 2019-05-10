
#ifndef lewansoul_h
#define lewansoul_h

#include "arduino.h"

class LewanSoul
{
  public:
    LewanSoul(int discard);
    
    void setup();
    
    void moveTo(int id, int destination);
};

#endif // lewansoul_h
