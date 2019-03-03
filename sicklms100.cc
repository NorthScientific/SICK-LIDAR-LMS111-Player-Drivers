/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2007
 *     Radu Bogdan Rusu (rusu@cs.tum.edu)
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/*
 Desc: Driver for the SICK LMS100 unit
 Author: Kasper Vinther (Aalborg University)
 Date: 29/3 -2009
*/

/** @ingroup drivers Drivers */
/** @{ */
/** @defgroup driver_sicklms100 sicklms100
 * @brief SICK LMS 100 laser range-finder

The sicklms100 driver controls the SICK LMS 100 scanning laser range-finder.

@par Compile-time dependencies

- none

@par Provides

- @ref interface_laser

@par Requires

- none

@par Configuration requests

- PLAYER_LASER_REQ_GET_CONFIG
- PLAYER_LASER_REQ_GET_GEOM
- PLAYER_LASER_REQ_GET_ID
- PLAYER_LASER_REQ_SET_CONFIG
- PLAYER_LASER_REQ_SET_FILTER

@par Configuration file options

- hostname (string)
  - Default: "169.254.248.234"
  - IP address of the SICK LMS 100 (Ethernet version)

- port (integer)
  - Default: 2111
  - TCP port of the SICK LMS 100 (Ethernet version)

- angular_resolution (float)
  - Default: 0.5 degrees
  - Angular resolution. Valid values are: 0.25 or 0.5

- scanning_frequency (float)
  - Default: 50 Hz.
  - Scanning frequency. Valid values are:
    - 25 or 50 Hz

- min_angle (float)
  - Default: -90 degrees.
  - Defines the minimum angle of the laser unit (where the scan should start).
    minimum -135 degrees

- max_angle (float)
  - Default: 90 degrees.
  - Defines the maximum angle of the laser unit (where the scan should end).
    max 135 degrees

- pose (length tuple)
  - Default: [0.0 0.0 0.0]
  - Pose (x,y,theta) of the laser, relative to its parent object (e.g.,
    the robot to which the laser is attached).

- size (length tuple)
  - Default: [0.10 0.10]
  - Footprint (x,y) of the laser.

- password (string)
  - Default: servicelevel/81BE23AA
  - Service (userlevel 4) password. Used for enabling/disabling and/or setting
    the filter parameters.

- debug (int)
  - Default: 0
  - Enable debugging mode (read/writes to the device are printed on screen).
    Valid values: 0 (disabled), 1 (enabled for standard messages), 2 (enabled
    for all messages including measurements - warning: this slows down your
    throughoutput date!).

@par Example

@verbatim
driver
(
  name "sicklms100"
  plugin "libsicklms100"
  provides ["laser:0"]
  hostname "169.254.248.234"
  port 2111

  # Set the angular resolution (0.5 degrees) and scanning frequency (50 Hz)
  angular_resolution 0.5
  scanning_frequency 50

  # Userlevel 4 password (hashed). Default: servicelevel/81BE23AA
  password "81BE23AA"

  #(if 2 read/writes to the device are printed on screen)
  debug 0     

  #(x,y,theta) of the laser, relative to its parent object
  pose [0.25 0.0 0.0] 

  alwayson 1

)
@endverbatim

@author Nico Blodow, Radu Bogdan Rusu and modified by Kasper Vinther

*/
/** @} */

#include <math.h>
#include "lms100_cola.h"

#define DEFAULT_LMS100_HOSTNAME    "169.254.248.234"
#define DEFAULT_LMS100_PORT         2111
#define DEFAULT_LMS100_L4_PASSWORD "NULL"
#define DEFAULT_LMS100_FREQUENCY    50
#define DEFAULT_LMS100_ANGULAR_RES  0.5
#define DEFAULT_LMS100_MINANGLE     -90.0
#define DEFAULT_LMS100_MAXANGLE     90.0

#include <libplayercore/playercore.h>
#include <libplayerxdr/playerxdr.h>

// The SICK LMS 100 laser device class.
class SickLMS100 : public Driver
{
  public:

    // Constructor/Destructor
    SickLMS100  (ConfigFile* cf, int section);
    ~SickLMS100 ();

    int Setup    ();
    int Shutdown ();

    // MessageHandler
    int ProcessMessage (QueuePointer &resp_queue,
		        player_msghdr* hdr,
		        void* data);
  private:
    // Main function for device thread.
    virtual void Main ();

    // Laser pose in robot cs.
    double pose[3];
    double size[2];

    // TCP/IP connection parameters
    const char* hostname;
    int         port_number;

    // Reference to lms100_cola
    lms100_cola* lms100;

    // Turn intensity data on/off
    bool intensity;

    // Turn laser on/off
    bool laser_enabled;

    // Basic measurement parameters
    double angular_resolution, scanning_frequency;
    double min_angle, max_angle;

    // Password for changing to userlevel 4 (service)
    const char* password;
    bool loggedin;

    int debug;
};

////////////////////////////////////////////////////////////////////////////////
// Constructor.  Retrieve options from the configuration file and do any
// pre-Setup() setup.
SickLMS100::SickLMS100 (ConfigFile* cf, int section)
    : Driver (cf, section, true,
              PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_LASER_CODE)
{
  // Laser geometry.
  pose[0] = cf->ReadTupleLength(section, "pose", 0, 0.0);
  pose[1] = cf->ReadTupleLength(section, "pose", 1, 0.0);;
  pose[2] = cf->ReadTupleLength(section, "pose", 2, 0.0);;
  size[0] = 0.10;
  size[1] = 0.10;

  intensity = true;

  // Read TCP/IP connection settings
  hostname    = cf->ReadString (section, "hostname", DEFAULT_LMS100_HOSTNAME);
  port_number = cf->ReadInt (section, "port", DEFAULT_LMS100_PORT);

  // Basic measurement parameters
  angular_resolution =
    cf->ReadFloat (section, "angular_resolution", DEFAULT_LMS100_ANGULAR_RES);
  scanning_frequency =
    cf->ReadFloat (section, "scanning_frequency", DEFAULT_LMS100_FREQUENCY);

  min_angle = cf->ReadFloat (section, "min_angle", DEFAULT_LMS100_MINANGLE);
  max_angle = cf->ReadFloat (section, "max_angle", DEFAULT_LMS100_MAXANGLE);

  // Password for changing to userlevel 3 (service)
  password = cf->ReadString (section, "password", DEFAULT_LMS100_L4_PASSWORD);
  loggedin = false;

  laser_enabled = true;

  debug = cf->ReadInt (section, "debug", 0);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor.
SickLMS100::~SickLMS100 (){
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device.  Return 0 if things go well, and -1 otherwise.
int
  SickLMS100::Setup ()
{
  // Create a lms100_cola object
  lms100 = new lms100_cola (hostname, port_number, debug);

  // Attempt to connect to the laser unit
  if (lms100->Connect () != 0){
    PLAYER_ERROR2 ("> Connecting to SICK LMS100 on [%s:%d]...[failed!]",
                    hostname, port_number);
    return (-1);
  }
  PLAYER_MSG2 (1, "> Connecting to SICK LMS100 on [%s:%d]... [done]",
               hostname, port_number);
  
  if (strncmp (password, "NULL", 4) != 0){
    // Login to userlevel 4
    if (lms100->SetUserLevel (4, password) != 0)
      PLAYER_WARN1 ("> Unable to change userlevel to 'Service' using %s", password);
    else{
      loggedin = true;
      // Stop the measurement process (in case it's running from another instance)
      lms100->StopMeasurement (); 
    }
  }
  else
    PLAYER_WARN ("> Userlevel 4 password not given. Filter(s) disabled!");

  // Set scanning parameters
  if (lms100->SetResolutionAndFrequency (
           scanning_frequency,
 	   angular_resolution,
	   min_angle,
	   max_angle - min_angle) != 0)
    PLAYER_ERROR ("> Couldn't set values for resolution, frequency, and min/max angle. Using previously set values.");
  else
    PLAYER_MSG0 (1, "> Enabling user values for resolution, frequency and min/max angle... [done]");

  // Configure scan data output
  if (lms100->ConfigureScanDataOutput() !=0) 
    PLAYER_ERROR ("> Couldn't configure scan data output.");
  else
    PLAYER_MSG0 (1, "> Configuring scan data output... [done]");

  // Start the device thread
  StartThread ();

  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
int
  SickLMS100::Shutdown ()
{
  // shutdown laser device
  StopThread ();

  // Stop the measurement process
  lms100->StopMeasurement ();

  // Set back to userlevel 0
  lms100->TerminateConfiguration ();

  // Disconnect from the laser unit
  lms100->Disconnect ();

  PLAYER_MSG0 (1, "> SICK LMS100 driver shutting down... [done]");

  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// ProcessMessage
int
  SickLMS100::ProcessMessage (QueuePointer &resp_queue,
                              player_msghdr* hdr,
                              void* data)
{
  assert (hdr);

  // ---[ Get geometry
  if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_REQ,
	PLAYER_LASER_REQ_GET_GEOM, device_addr))
  {
    player_laser_geom_t geom;
    memset(&geom, 0, sizeof(geom));
    geom.pose.px = pose[0];
    geom.pose.py = pose[1];
    geom.pose.pyaw = pose[2];
    geom.size.sl = size[0];
    geom.size.sw = size[1];

    Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK,
             PLAYER_LASER_REQ_GET_GEOM, (void*)&geom, sizeof(geom), NULL);
    return (0);
  }
  // --]

  // ---[ Set power
  else if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_REQ,
	PLAYER_LASER_REQ_POWER, device_addr))
  {
    player_laser_power_config_t* config =
      reinterpret_cast<player_laser_power_config_t *> (data);

    if (config->state == 0)
    {
      lms100->StopMeasurement ();
      laser_enabled = false;
    }
    else
    {
      lms100->StartMeasurement (intensity);
      laser_enabled = true;
    }

    Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK,
             hdr->subtype);
    return (0);
  }
  // --]

  // ---[ Set configuration
  else if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_REQ,
	PLAYER_LASER_REQ_SET_CONFIG, device_addr))
  {
    player_laser_config_t* config =
      reinterpret_cast<player_laser_config_t *> (data);

    // Set intensity locally, ignore range_max and range_res (unused for LMS100)
    intensity = config->intensity;
    // Setting {min, max}_angle, resolution (angular), and scanning_frequency
    angular_resolution = RTOD (config->resolution);
    min_angle  = RTOD (config->min_angle);
    max_angle  = RTOD (config->max_angle);
    scanning_frequency = config->scanning_frequency;

    // Stop the measurement process
    lms100->StopMeasurement ();

    // Change userlevel to 4
    if (lms100->SetUserLevel (4, password) != 0)
      PLAYER_WARN1 ("> Unable to change userlevel to 'Service' using %s", password);
    else
      // Set the angular resolution and frequency
      if (lms100->SetResolutionAndFrequency (
           scanning_frequency,
 	   angular_resolution,
	   min_angle,
	   max_angle - min_angle) == 0)
      {
        // Re-start the measurement process
        if (laser_enabled) lms100->StartMeasurement (intensity);

        // Configuration succeeded; send an ACK
        Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK,
                 hdr->subtype);
        return (0);
      }

    // Re-start the measurement process
    if (laser_enabled) lms100->StartMeasurement (intensity);

    // Configuration failed; send a NACK
    Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_NACK,
             hdr->subtype);
    return (-1);
  }

  // ---[ Get configuration
  else if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_REQ,
	PLAYER_LASER_REQ_GET_CONFIG, device_addr))
  {
    // Get min_angle, max_angle, resolution and scanning_frequency
    player_laser_config_t config = lms100->GetConfiguration ();
    config.max_range = 20; // maximum measurable distance
    config.range_res = 0.5; // bogus value
    config.intensity = intensity;

    Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK,
             PLAYER_LASER_REQ_GET_CONFIG, (void*)&config, sizeof (config), NULL);
    return (0);
  }

  // ---[ Get IDentification information
  else if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_REQ,
	PLAYER_LASER_REQ_GET_ID, device_addr))
  {
    PLAYER_WARN ("> LMS100 driver doesn't support Get Identification.");
    return (-1);
  }
  // --]

  // ---[ Set filter settings
  else if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_REQ,
	PLAYER_LASER_REQ_SET_FILTER, device_addr))
  {
    PLAYER_WARN ("> LMS100 driver doesn't support filters.");
    return (-1);
  }
  else
    return (-1);
}

////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void
  SickLMS100::Main ()
{
  // Start Continous measurements
  lms100->StartMeasurement (intensity);

  while (true)
  {
    // test if we are supposed to cancel
    pthread_testcancel ();

    // Request/replies handler
    ProcessMessages ();

    // Refresh data only if laser power is on
    if (laser_enabled)
    {
      player_laser_data_t data = lms100->ReadMeasurement ();

      // Make data available
      if (data.ranges_count != (unsigned int)-1)
        Publish (device_addr, PLAYER_MSGTYPE_DATA, PLAYER_LASER_DATA_SCAN, &data);
      player_laser_data_t_cleanup(&data);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Factory creation function. This functions is given as an argument when the
// driver is added to the driver table
Driver*
  SickLMS100_Init (ConfigFile* cf, int section)
{
  return ((Driver*)(new SickLMS100 (cf, section)));
}

////////////////////////////////////////////////////////////////////////////////
// Registers the driver in the driver table. Called from the player_driver_init
// function that the loader looks for
void
  SickLMS100_Register (DriverTable* table)
{
  table->AddDriver ("sicklms100", SickLMS100_Init);
}

/// Extra stuff for building a shared object.

//Needed to avoid C++ name-mangling

extern "C"
{
	int player_driver_init (DriverTable * table)
	{
		SickLMS100_Register (table);
		return (0);
	}
}