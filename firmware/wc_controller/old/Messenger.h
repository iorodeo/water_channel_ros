#ifndef Messenger_h
#define Messenger_h
#include <inttypes.h>

#define MESSENGERBUFFERSIZE 64

class Messenger
{

public:
  Messenger();
  Messenger(char separator);
  int readInt();
  long readLong(); // Added based on a suggestion by G. Paolo Sani
  char readChar();
  double readDouble(); // Added based on a suggestion by Lorenzo C.
  void copyString(char *string, uint8_t size);
  uint8_t checkString(char *string);
  uint8_t process(int serialByte);
  uint8_t available();
  virtual void switchYard();

protected:
  uint8_t next();
  
private:
  void init(char separator);
  void reset();
  
  uint8_t messageState;
  
  char* current; // Pointer to current data
  char* last;
  
  char token[2];
  uint8_t dumped;
  
  uint8_t bufferIndex; // Index where to write the data
  char buffer[MESSENGERBUFFERSIZE]; // Buffer that holds the data
  uint8_t bufferLength; // The length of the buffer (defaults to 64)
  uint8_t bufferLastIndex; // The last index of the buffer
};

#endif

