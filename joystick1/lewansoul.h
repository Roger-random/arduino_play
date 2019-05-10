
#ifndef lewansoul_h
#define lewansoul_h

#include "arduino.h"

class LewanSoul
{
  public:
    LewanSoul(int discard);
    
    void setup();
    
    void moveTo(int id, int destination);
    void spinAt(int id, int velocity);
};

#endif // lewansoul_h
