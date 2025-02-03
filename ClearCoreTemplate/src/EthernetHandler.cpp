
/*
 * Title: EthernetTCPServer_autoClientManagement
 *
 * Objective:
 *    This example demonstrates how to configure a ClearCore as a TCP server to 
 *    send and receive TCP datagrams (packets). 
 *    
 * Description:
 *    This example configures a ClearCore device to act as a TCP server. 
 *    This server can receive connections from another device acting as a TCP 
 *    client to exchange data over ethernet TCP. 
 *    This simple example accepts connection requests from clients, receives and
 *    prints incoming data from connected devices, and sends a simple "Hello 
 *    client" response.
 *    A partner project, EthernetTcpClientHelloWorld, is available to configure 
 *    another ClearCore as a client.
 *
 * Setup:
 * 1. Set the usingDhcp boolean as appropriate. If not using DHCP, specify static 
 *    IP and network information.
 * 2. Ensure the server and client are connected to communicate on the same network.
 *    If both devices are directly connected (as opposed to going through a switch)
 *    a ethernet crossover cable may be required. 
 * 3. It may be helpful to use another application to view serial output from 
 *    each device. PuTTY is one such application: https://www.putty.org/
 *
 * Links:
 * ** ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
 * ** ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf
 * 
 * Copyright (c) 2022 Teknic Inc. This work is free to use, copy and distribute under the terms of
 * the standard MIT permissive software license which can be found at https://opensource.org/licenses/MIT
 */
#include "EthernetHandler.h"
#include <SPI.h>
#include <Ethernet.h>

// LED Pin to use for Ethernet status
#define ETH_STATUS_LED IO0

#define BAUD_RATE 115200

// MAC address of the ClearCore
byte mac[] = {};

// The port number on the server over which packets will be sent/received
#define PORT_NUM 8888

// Buffer for holding received packets
unsigned char packetReceived[MAX_PACKET_LENGTH];

// Set usingDhcp to false to use user defined network settings
bool usingDhcp = false;

// Initialize the ClearCore as a server listening for incoming
// client connections on specified port (8888 by default)
EthernetServer server = EthernetServer(PORT_NUM);

// Initialize a client object
// This object will hold a connected clients information
// allowing the server to interact with the client
EthernetClient client;

void ethernetSetup()
{
  // Set up serial communication between ClearCore and PC serial terminal
  Serial.flush();
  Serial.begin(BAUD_RATE);
  uint32_t timeout = 5000;
  uint32_t startTime = millis();
  while (!Serial && millis() - startTime < timeout)
  {
    continue;
  }

  pinMode(ETH_STATUS_LED, OUTPUT);

  // Make sure the physical link is active before continuing
  while (Ethernet.linkStatus() == LinkOFF)
  {
    Serial.println("The Ethernet cable is unplugged...");
    delay(1000);
  }

  // To configure with an IP address assigned via DHCP
  if (usingDhcp)
  {
    // Use DHCP to configure the local IP address
    bool dhcpSuccess = Ethernet.begin(mac);
    if (dhcpSuccess)
    {
      Serial.print("DHCP successfully assigned an IP address: ");
      Serial.println(Ethernet.localIP());
    }
    else
    {
      Serial.println("DHCP configuration was unsuccessful!");
      while (true)
      {
        // TCP will not work without a configured IP address
        continue;
      }
    }
  }
  else
  {
    // Configure with a manually assigned IP address

    // ClearCore MAC address
    byte mac[] = {0x24, 0x15, 0x10, 0xB0, 0x42, 0x1B}; // (placeholder)

    // Set ClearCore's IP address
    IPAddress ip = IPAddress(192, 168, 0, 100);
    Ethernet.begin(mac, ip);
    Serial.print("Assigned manual IP address: ");
    Serial.println(Ethernet.localIP());

    // Optionally set additional network addresses if needed

    // IpAddress myDns = myDns(192, 168, 1, 1);
    // IpAddress gateway = gateway(192, 168, 1, 1);
    // IpAddress netmask = subnetress(255, 255, 255, 0);

    // Ehternet.begin(mac, ip, myDns, gateway, subnet);
  }

  // Start listening for TCP connections
  server.begin();

  Serial.println("Server now listening for client connections...");
}

uint32_t startTime = millis();
// Connect to clients, and send/receive packets
bool getEthernetMessage(char* msgBuffer)
{
  // Obtain a reference to a connected client with incoming data available
  // This function will only return a valid reference if the connected device has
  // data available to read
  client = server.available();

  // Check if the server has returned a connected client with incoming data available
  if (client.connected())
  {
    // Turn on LED if a client has sent a message
    digitalWrite(ETH_STATUS_LED, true);

    // Read packet from the client
    while (client.available() > 0)
    {
      int lengthOfMessage = client.available();
      if (lengthOfMessage > MAX_PACKET_LENGTH)
      {
        Serial.println("WARNING: received packet with length greater than MAX_PACKET_LENGTH. Message will be truncated");
        Serial.print("Received Packet Length: ");
        Serial.print(lengthOfMessage);
        Serial.print(", MAX_PACKET_LENGTH: ");
        Serial.println(MAX_PACKET_LENGTH);
      }
      for (int i = 0; i < lengthOfMessage && i <= MAX_PACKET_LENGTH; i++)
      {
        char currentChar = client.read();
        packetReceived[i] = currentChar;
      }

      // Print the data received from the client for debugging
      Serial.print("Read the following from the client: ");
      Serial.print((char *)packetReceived);

      // Copy the data into the param buffer, for sending back to caller
      strcpy(msgBuffer, (const char*)packetReceived);

      // Clear the message buffer for the next iteration of the loop
      for (int i = 0; i < MAX_PACKET_LENGTH; i++)
      {
        packetReceived[i] = '\0';
      }

      while (client.read() != -1); // Flush out the rest of the message if longer than MAX_PACKET_LENGTH
    }
    Serial.println();

    // Send response message to client
    if (client.write("ACK\r") > 0)
    {
      Serial.println("Sent ACK response to client");
    }
    else
    {
      Serial.println("ERROR: Unable to send ACK to client");
    }

    return true;
  }
  else
  {
    // Turn off LED if a message has not been received
    digitalWrite(ETH_STATUS_LED, false);

    // Not sure how to impliment in arduino
    // if(client.ConnectionState()->state == CLOSING){
    // client.stop();
    //}
    client.stop();

    const int logDelay = 3000;
    if (Ethernet.linkStatus() == LinkON && (millis() - startTime > logDelay))
    {
      Serial.println("Waiting for message...");
      startTime = millis();
    }
    // Make sure the physical link is active before continuing
    while (Ethernet.linkStatus() == LinkOFF)
    {
      Serial.println("The Ethernet cable is unplugged...");
      delay(1000);
    }
    return false;
  }
}

// Send message back to most recently connected client
bool sendEthernetMessage(const char* msgBuffer)
{
  if (client.connected())
  {
    if (client.write(msgBuffer, strlen(msgBuffer)) > 0)
    {
      Serial.println("Sent the following message to client:");
      Serial.println(msgBuffer);
      return true;
    }
    else
    {
      Serial.println("ERROR: Unable to send message to client");
      return false;
    }
  }
  else
  {
    Serial.println("ERROR: Client no longer connected");
    return false;
  }
}