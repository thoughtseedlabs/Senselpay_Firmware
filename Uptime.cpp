
#include <Arduino.h>

struct Uptime
{
  long Days = 0;
  int Hours = 0;
  int Minutes = 0;
  int Seconds = 0;
  int HighMillis = 0;
  int Rollover = 0;

  const long DAY = 86400000; // 86400000 milliseconds in a day
  const long HOUR = 3600000; // 3600000 milliseconds in an hour
  const long MINNUTE = 60000; // 60000 milliseconds in a minute
  const long SECOND = 1000;  // 1000 milliseconds in a second

  ulong lastUpdateTime = millis();
  const ulong updateDelay = 1000;

  // Runs the uptime script located below the main loop and reenters the main loop
  void update()
  {
    ulong milliesNow = millis();

    if ((milliesNow - lastUpdateTime) > updateDelay)
    {
      lastUpdateTime = milliesNow;

      //** Making Note of an expected rollover *****//
      if (milliesNow >= 3000000000)
      {
        HighMillis = 1;
      }

      //** Making note of actual rollover **//
      if (milliesNow <= 100000 && HighMillis == 1)
      {
        Rollover++;
        HighMillis = 0;
      }

      // First portion takes care of a rollover [around 50 days]
      Days = (milliesNow / DAY) + (Rollover * 50);
      Hours = (milliesNow % DAY) / HOUR;
      Minutes = ((milliesNow % DAY) % HOUR) / MINNUTE;
      Seconds = (((milliesNow % DAY) % HOUR) % MINNUTE) / SECOND;

      // debug
      // Days = (((milliesNow % DAY) % HOUR) % MINNUTE) % SECOND;
    }
  };

  // Get system uptime as string
  String getUptime()
  {
    String secs_o = ":";
    String mins_o = ":";
    String hours_o = ":";

    if (Seconds < 10)
      secs_o = ":0";

    if (Minutes < 10)
      mins_o = ":0";

    if (Hours < 10)
      hours_o = ":0";

    return Days + hours_o + Hours + mins_o + Minutes + secs_o + Seconds;
  }
};
