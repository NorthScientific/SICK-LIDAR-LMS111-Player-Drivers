/*
 Desc: Driver for the SICK LMS100 unit
 Author: Nico Blodow and Radu Bogdan Rusu
 Modified by: Kasper Vinther
 Date: 29/3 - 2009
*/
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <libplayercore/playercore.h>

#include "lms100_cola.h"
#include <unistd.h>

const int CMD_BUFFER_SIZE=255;

////////////////////////////////////////////////////////////////////////////////
// Constructor.
lms100_cola::lms100_cola (const char* host, int port, int debug_mode)
{
  portno   = port;
  hostname = host;
  verbose  = debug_mode;
  bzero (command, BUF_SIZE);
  MeasurementQueue = new std::vector<MeasurementQueueElement_t >;
}

////////////////////////////////////////////////////////////////////////////////
// Connect to the LMS100 unit using hostname:portno
// Returns 0 if connection was successful, -1 otherwise
int
  lms100_cola::Connect ()
{
  // Create a socket
  sockfd = socket (AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    return (-1);

  // Get the network host entry
  server = gethostbyname ((const char *)hostname);
  if (server == NULL)
    return (-1);

  // Fill in the sockaddr_in structure values
  bzero ((char *) &serv_addr, sizeof (serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port   = htons (portno);
  bcopy ((char *)server->h_addr,
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);

  // Attempt to connect
  if (connect (sockfd, (const sockaddr*)&serv_addr, sizeof (serv_addr)) < 0)
    return (-1);

  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Disconnect from the LMS100 unit
// Returns 0 if connection was successful, -1 otherwise
int
  lms100_cola::Disconnect ()
{
  return (close (sockfd));
}

////////////////////////////////////////////////////////////////////////////////
// Get the current laser unit configuration and return it into Player format
player_laser_config
  lms100_cola::GetConfiguration ()
{
  player_laser_config_t cfg;
  cfg = Configuration;
  return cfg;
}

////////////////////////////////////////////////////////////////////////////////
// Configure scan data output (this function does not work yet! The LMS100 
// answers with error code "sFA 8")
int
  lms100_cola::ConfigureScanDataOutput ()
{
  char cmd[CMD_BUFFER_SIZE];
  uint16_t OutputChannel = 1;
  bool Remission = 0;
  int Resolution = 1;  
  int Unit = 0;
  uint16_t Encoder = 0;
  bool Position = 0;
  bool DeviceName = 0;
  bool Comment = 0;
  bool Time = 0;
  uint16_t Outputinterval = 1;
  snprintf (cmd, CMD_BUFFER_SIZE, "sWN LMDscandatacfg %d %d %d %d %d %d %d %d %d %d", 
    OutputChannel, Remission, Resolution, Unit, Encoder, Position, DeviceName, 
    Comment, Time, Outputinterval);

  SendCommand (cmd);

  if (ReadAnswer () != 0)
    return (-1);

  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Set the desired userlevel by logging in with the appropriate password
int
  lms100_cola::SetUserLevel (int8_t userlevel, const char* password)
{
  char cmd[CMD_BUFFER_SIZE];
  snprintf (cmd, CMD_BUFFER_SIZE, "sMN SetAccessMode %d %s", userlevel, password);
  SendCommand (cmd);

  return (ReadAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Terminate configuration and change back to userlevel 0
int
  lms100_cola::TerminateConfiguration ()
{
  const char* cmd = "sMN Run";
  SendCommand (cmd);

  return (ReadAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Set both resolution and frequency without going to a higher user level (?)
int
  lms100_cola::SetResolutionAndFrequency (float freq, float ang_res,
                                          float angle_start, float angle_range)
{
  char cmd[CMD_BUFFER_SIZE];
  uint32_t frequ = (uint32_t) (freq*100);
  uint8_t NumberSegments=1;
  uint32_t ang_resu = (uint32_t) (ang_res*10000);
  int32_t angle_star = (int32_t) ((angle_start+90)*10000);
  int32_t angle_rang = (int32_t) ((angle_range+90)*10000);
  snprintf (cmd, CMD_BUFFER_SIZE, "sMN mLMPsetscancfg +%d +%d +%d %d +%d",
    frequ, NumberSegments, ang_resu, angle_star, angle_rang);
  SendCommand (cmd);

  int error = ReadAnswer ();

  // If no error, parse the results
  if (error == 0)
  {
    strtok ((char*)buffer, " "); strtok (NULL, " ");
    int ErrorCode = strtol (strtok (NULL, " "), NULL, 16);
    long int sf = strtol (strtok (NULL, " "), NULL, 16);
    long int re = strtol (strtok (NULL, " "), NULL, 16);

    if ((ErrorCode != 0) && (verbose))
      printf (">> Warning: got an error code %d\n", ErrorCode);

    memcpy (&Configuration.scanning_frequency, &sf, sizeof (uint32_t));
    memcpy (&Configuration.resolution, &re, sizeof (uint32_t));

    if (verbose)
      printf (">> Measured value quality is: %ld [2500-5000]\n",
        strtol (strtok (NULL, " "), NULL, 16));
  }

  return (error);
}

////////////////////////////////////////////////////////////////////////////////
// Start a measurement for both distance and intensity or just distance.
int
  lms100_cola::StartMeasurement (bool intensity)
{
  char cmd[CMD_BUFFER_SIZE];
  snprintf (cmd, CMD_BUFFER_SIZE, "sMN LMCstartmeas");
  SendCommand (cmd);
  ReadAnswer ();
  
  //Status of the LMS must be ASCII 7 which means it is ready for measurement
  printf (" >>> Waiting for LMS to be ready...\n");
  snprintf (cmd, CMD_BUFFER_SIZE, "sRN STlms");
  do{
    SendCommand (cmd);
    buffer[10]=0;
    n = read (sockfd, buffer, 1);

    if (n < 0) return (-1);

    if (buffer[0] != 0x02){
      if (verbose) printf ("> E: expected STX!\n");
      n = read (sockfd, buffer, 3000);
      return (-1);
    }
    n = read (sockfd, buffer, 3000);
  }while (buffer[10] != 55 );
  printf (" >>> LMS ready\n");

  // Start continous measurement value output
  int startmeasure = 1;
  snprintf (cmd, CMD_BUFFER_SIZE, "sEN LMDscandata %d",startmeasure);
  SendCommand (cmd);
  ReadAnswer ();

  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Read a measurement
player_laser_data_t
  lms100_cola::ReadMeasurement ()
{
  player_laser_data_t player_data;
  player_data.ranges_count = -1; 
  bzero (buffer, 3000);
  if (!MeasurementQueue->empty ())
  {
    if (verbose) printf (">>> Reading from queue...\n");
    memcpy (buffer, (char*) MeasurementQueue->front ().string, MeasurementQueue->front ().length + 1);
    free (MeasurementQueue->front ().string);
    MeasurementQueue->erase (MeasurementQueue->begin ());
  }
  else
  {
    if (verbose == 2) printf (">>> Queue empty. Reading from socket...\n");
    
    n = read (sockfd, buffer, 30);
    if (n < 0)
    {
      if (verbose) printf (">>> E: error reading from socket!\n");
      return (player_data);
    }
    if (buffer[0] != 0x02)
    {
      if (verbose) printf (">>> E: error expected 1 STX byte!\n");
      n = read (sockfd, buffer, 3000);
      return (player_data);
    }

    n = read (sockfd, buffer, 3000);
  }

  // parse measurements header and fill in the configuration parameters
  MeasurementHeader_t meas_header;

  index_k=0;
  meas_header.DeviceStatus = (uint8_t) message_cutter ();
  meas_header.MessageCounter = (uint16_t) message_cutter ();
  meas_header.ScanCounter = (uint16_t) message_cutter ();
  meas_header.PowerUpDuration = (uint32_t) message_cutter ();
  meas_header.TransmissionDuration = (uint32_t) message_cutter ();
  index_k += 2;
  meas_header.InputStatus = (uint16_t) message_cutter ();
  meas_header.OutputStatus = (uint16_t) message_cutter ();
  index_k += 2;
  meas_header.ReservedByteA = (uint8_t) message_cutter ();
  meas_header.ScanningFrequency = (uint32_t) message_cutter ();
  meas_header.MeasurementFrequency = (uint32_t) message_cutter ();
  meas_header.NumberEncoders = (uint8_t) message_cutter ();
  meas_header.NumberChannels16bit = (uint8_t) message_cutter ();
  index_k += 6;
  meas_header.ScalingFactor = (uint32_t) message_cutter ();
  //a scaling factor of 1000 is used to convert from mm to m
  meas_header.ScalingFactor = (uint32_t) 1000.0;
  meas_header.ScalingOffset = (uint32_t) message_cutter ();
  meas_header.StartingAngle = (int32_t) message_cutter ();
  meas_header.AngularStepWidth = (uint16_t) message_cutter ();
  meas_header.NumberData = (uint16_t) message_cutter ();
 
  // Currently min and max angle are hardcoded.
  Configuration.min_angle  = -90.0;//((meas_header.StartingAngle / 10000.0));
  Configuration.resolution = meas_header.AngularStepWidth / 10000.0;
  Configuration.max_angle = 90.0;
   // ((float) meas_header.NumberData-1) * Configuration.resolution + Configuration.min_angle;
  Configuration.scanning_frequency = meas_header.ScanningFrequency / 100;

  // Fill in the appropriate values
  player_data.min_angle       = DTOR (Configuration.min_angle);
  player_data.max_angle       = DTOR (Configuration.max_angle);
  player_data.resolution      = DTOR (Configuration.resolution);
  player_data.max_range       = 20;
  player_data.intensity_count = 0;
  player_data.id              = meas_header.ScanCounter;
  player_data.intensity = new uint8_t[  player_data.intensity_count];

  // copy range data from the buffer into player_data struct.
  // only values between 0 and 180 degrees are used
  // In player_data it is -90 to 90 since we scan in the x-direction
  if(Configuration.resolution == 0.5){
    player_data.ranges_count = (meas_header.NumberData-180);
    player_data.ranges = new float[  player_data.ranges_count];
    for (int i = 0; i < 90 ; i++){
      message_cutter ();
    }
    for (int i = 0; i < (meas_header.NumberData-180) ; i++){
      player_data.ranges[i] =  (((float) message_cutter ()) / meas_header.ScalingFactor);

      if (verbose == 2)
        printf (" >>> [%i] dist: %f\n", i, player_data.ranges[i]);
    }
  }

  if(Configuration.resolution == 0.25){
    player_data.ranges_count = (meas_header.NumberData-360);
    player_data.ranges = new float[  player_data.ranges_count];
    for (int i = 0; i < 180 ; i++){
      message_cutter ();
    }
    for (int i = 0; i < (meas_header.NumberData-360) ; i++){
      player_data.ranges[i] =  (((float) message_cutter ()) / meas_header.ScalingFactor);

      if (verbose == 2)
        printf (" >>> [%i] dist: %f\n", i, player_data.ranges[i]);
    }
  }

  return (player_data);
}

////////////////////////////////////////////////////////////////////////////////
// Stop a measurement
int
  lms100_cola::StopMeasurement ()
{
  char cmd[CMD_BUFFER_SIZE];
  snprintf (cmd, CMD_BUFFER_SIZE, "sMN LMCstopmeas");//"sMN mLRstopdata"
  SendCommand (cmd); 
  return (ReadAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Send a command to the laser unit. Returns -1 on error.
int
  lms100_cola::SendCommand (const char* cmd)
{
  if (verbose && strncmp ((const char*)cmd, "sRN STlms", 9) != 0)
    printf (">> Sent: \"%s\"\n", cmd);
  assemblecommand ((unsigned char *) cmd, strlen (cmd));

  n = write (sockfd, command, commandlength);

  if (n < 0)
    return (-1);

  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Read a result from the laser unit.
int
  lms100_cola::ReadResult ()
{
  bzero (buffer, 3000);
  n = read (sockfd, buffer, 1);

  if (n < 0)
    return (-1);

  if (buffer[0] != 0x02)
  {
    if (verbose) printf ("> E: expected STX!\n");
    n = read (sockfd, buffer, 3000);
    return (-1);
  }
  
  // Put the message in the buffer
  n = read (sockfd, buffer, 3000);

  if ((verbose) && (buffer[0] != 0x20))
    printf (">> Received: \"%s\"\n", buffer);

  // Check for error
  if (strncmp ((const char*)buffer, "sFA", 3) == 0)
  {
    strtok ((char*)buffer, " ");
    printf (">> E: Got an error message with code 0x%s\n", strtok (NULL, " "));
    return (-1);
  }

  if (buffer[0] == 's')
    return (0);
  else if (buffer[0] == 0x20)
    return (ReadResult ());

  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Read an answer from the laser unit
int
  lms100_cola::ReadAnswer ()
{
  return ReadResult ();
}

////////////////////////////////////////////////////////////////////////////////
// Read a confirmation and an answer from the laser unit
int
  lms100_cola::ReadConfirmationAndAnswer ()
{
  ReadResult ();
  if (buffer[0] == 's' && buffer[1] == 'F' && buffer[2] == 'A')
    return (-1);
  else{
    return ReadResult ();
  }
}

////////////////////////////////////////////////////////////////////////////////
// adds STX and ETX to command to be sent
int
  lms100_cola::assemblecommand (unsigned char* cmd, int len)
{
  int index = 0;

  command[0]  = 0x02;  // Messages start with 1 STX

  for (index = 0; index < len; index++)
  {
    command[index + 1]  = cmd[index];
  }

  command[1 + len] = 0x03; // Messages end with 1 ETX

  commandlength = 2 + len;
}

////////////////////////////////////////////////////////////////////////////////
// Cuts out data from the buffer and converts from string to integer.
uint64_t lms100_cola::message_cutter (){
  uint8_t counter = 0;
  uint64_t output = 0;

  // Data is seperated with white space (hex 0x20 or decimal 32)
  while(buffer[index_k] != 32){
    index_k += 1;
    counter += 1;
  } 

  // Create a string
  char streng[counter+1];
  for(int i = 0 ; i < counter ; i++){
    streng[i]=buffer[index_k-counter+i];
  }
  streng[counter]=NULL;

  // Convert the string to an integer
  output = (uint64_t) strtol(streng, NULL, 16);

  // Increment index_k to point to the next data in the message
  index_k += 1;

  return output;
}
