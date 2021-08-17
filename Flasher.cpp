#include <Arduino.h>

struct Flasher
{
  const byte pin;
  const uint32_t *sequence;
  const boolean flashForever = false;

  int sequenceIndex;
  boolean flashOn = false;
  boolean startFlag = false;
  uint32_t startTime;

  Flasher(byte attachPin, const uint32_t *sequence, const boolean flashForever)
      : pin(attachPin), sequence(sequence), flashForever(flashForever)
  {
    flashOn = false;
    startFlag = false;
    sequenceIndex = 0;
  }

  void setup()
  {
    pinMode(pin, OUTPUT);
    //start with the LED off
    digitalWrite(pin, LOW);
  }

  void start()
  {
    startTime = millis();
    startFlag = true;
  }

  void stop()
  {
    startFlag = false;
  }

  void loop()
  {
    if (startFlag)
    {
      if (millis() - startTime >= sequence[sequenceIndex])
      {
        sequenceIndex++;
        if (sequence[sequenceIndex] == 0)
        {
          sequenceIndex = 0;
          if (!flashForever)
            startFlag = false;
        }

        startTime = millis();

        flashOn = !flashOn;
        digitalWrite(pin, flashOn ? HIGH : LOW);
      }
    }
  }
};

struct Fader
{
  const byte pin;
  const uint32_t fadeTime;

  uint32_t startTime;
  boolean gettingBrighter;

  Fader(byte attachPin, uint32_t fadeTime) : pin(attachPin), fadeTime(fadeTime)
  {
    startTime = millis();
    gettingBrighter = true;
  }

  void setup()
  {
    pinMode(pin, OUTPUT);
  }

  void loop()
  {
    if (millis() - startTime >= fadeTime)
    {
      startTime = millis();
      gettingBrighter = !gettingBrighter;
    }

    if (gettingBrighter)
    {
      analogWrite(pin, (millis() - startTime) / ((double)fadeTime) * 255);
    }
    else
    {
      analogWrite(pin, 255 - (millis() - startTime) / ((double)fadeTime) * 255);
    }
  }
};
