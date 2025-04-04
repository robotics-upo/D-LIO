#include <time.h>

#ifndef __ELAPSEDTIME_HPP__
#define __ELAPSEDTIME_HPP__

class ElapsedTime
{
  private:

    // Image with bicubic params for each pixel
    struct timespec t1, t2;

  public:

    ElapsedTime(void)
    {
      clock_gettime(CLOCK_REALTIME, &t1);
    }

    void tick(void)
    {
      clock_gettime(CLOCK_REALTIME, &t1);
    }

    double tock(void)
    {
      clock_gettime(CLOCK_REALTIME, &t2);
      if ((t2.tv_nsec - t1.tv_nsec) < 0) 
        return t2.tv_sec - t1.tv_sec - 1 + (t2.tv_nsec - t1.tv_nsec + 1000000000.0)/1000000000.0;
      else 
        return t2.tv_sec - t1.tv_sec + (t2.tv_nsec - t1.tv_nsec)/1000000000.0;
    }

    double elapsed(void)
    {
      if ((t2.tv_nsec - t1.tv_nsec) < 0) 
        return t2.tv_sec - t1.tv_sec - 1 + (t2.tv_nsec - t1.tv_nsec + 1000000000.0)/1000000000.0;
      else 
        return t2.tv_sec - t1.tv_sec + (t2.tv_nsec - t1.tv_nsec)/1000000000.0;
    }
};

#endif

