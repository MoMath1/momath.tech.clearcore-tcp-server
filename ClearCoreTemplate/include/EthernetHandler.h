// The maximum number of characters to receive from an incoming packet
#define MAX_PACKET_LENGTH 256

void ethernetSetup();
bool getEthernetMessage(char* msgBuffer);
bool sendEthernetMessage(const char* msgBuffer);