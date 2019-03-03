/*
 Desc: Driver for the SICK LMS100 unit
 Author: Nico Blodow and Radu Bogdan Rusu
 Modified by: Kasper Vinther
 Date: 29/3 - 2009
*/
#include <sys/types.h>
#include <vector>
#include <netinet/in.h>
#include <libplayercore/player.h>
#include <iostream>

#define BUF_SIZE 1024

////////////////////////////////////////////////////////////////////////////////
typedef struct
{
  unsigned char* string;
  int length;
} MeasurementQueueElement_t;

////////////////////////////////////////////////////////////////////////////////
typedef struct
{
  uint8_t DeviceStatus;
  uint16_t MessageCounter;
  uint16_t ScanCounter;
  uint32_t PowerUpDuration;
  uint32_t TransmissionDuration;
  uint16_t InputStatus;
  uint16_t OutputStatus;
  uint8_t ReservedByteA;
  uint32_t ScanningFrequency;
  uint32_t MeasurementFrequency;
  uint8_t NumberEncoders;
  uint8_t NumberChannels16bit;
  uint32_t ScalingFactor;
  uint32_t ScalingOffset; 
  int32_t StartingAngle;
  uint16_t AngularStepWidth;
  uint16_t NumberData;
} MeasurementHeader_t;


////////////////////////////////////////////////////////////////////////////////
class lms100_cola
{
  public:
    lms100_cola (const char* host, int port, int debug_mode);

    // Creates socket, connects
    int Connect ();
    int Disconnect ();

    // Configuration parameters
    int SetResolutionAndFrequency (float freq, float ang_res, float angle_start, float angle_range);
    int ConfigureScanDataOutput ();

    int StartMeasurement (bool intensity = true);
    player_laser_data ReadMeasurement  ();
    int StopMeasurement  ();

    int SetUserLevel  (int8_t userlevel, const char* password);

    int TerminateConfiguration ();

    int SendCommand   (const char* cmd);
    int ReadResult    ();
    // for "Variables", Commands that only reply with one Answer message
    int ReadAnswer    ();
    // for "Procedures", Commands that reply with a Confirmation message and an Answer message
    int ReadConfirmationAndAnswer ();

    player_laser_config GetConfiguration ();

  private:
    // assembles frame to be sent
    int assemblecommand (unsigned char* command, int len);
    uint64_t message_cutter ();

    const char* hostname;
    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    // Internal Parameters:
    int verbose;
    player_laser_config Configuration;

    // for reading:
    unsigned char buffer[4096];
    uint16_t index_k;
    unsigned int bufferlength;

    // for sending:
    unsigned char command[BUF_SIZE];
    int commandlength;
    std::vector<MeasurementQueueElement_t>* MeasurementQueue;
};
