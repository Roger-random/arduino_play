
#ifndef lewansoul_h
#define lewansoul_h

#include "arduino.h"

class LewanSoul
{
  public:
    LewanSoul(int discard);
    
    void setup();

    // ID of desired servo to move, and destination from -100 to 100
    void moveTo(int id, int destination);

    // ID of desired servo to spin, and velocity from -100 to 100
    void spinAt(int id, int velocity);
};

#endif // lewansoul_h
