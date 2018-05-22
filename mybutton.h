#define BUT_DELAY      500
#define BUT_REPEAT     50

class Button {
  public:
    Button(int buttonPin);
    int state() {return s;}
    bool updateButton();
    bool reset = true;
    unsigned long pressedTime = 0;
  protected:
    int s, pin, lastState = HIGH;
    unsigned long debounceTimer = 0, debounceDelay = 50;
};


