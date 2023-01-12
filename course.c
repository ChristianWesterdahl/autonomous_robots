#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <getopt.h> // Added

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

// Define pi
#ifndef M_PI
#define M_PI (3.14159265358979323846264338327950288)
#endif


struct xml_in *xmldata;
struct xml_in *xmllaser;
struct
{
  double x, y, z, omega, phi, kappa, code, id, crc;
} gmk;
double visionpar[10];
double laserpar[10];
volatile int running = 1;
int robot_port;

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

componentservertype lmssrv, camsrv;

symTableElement *
getinputref(const char *sym_name, symTableElement *tab)
{
  int i;
  for (i = 0; i < getSymbolTableSize('r'); i++)
    if (strcmp(tab[i].name, sym_name) == 0)
      return &tab[i];
  return 0;
}

symTableElement *
getoutputref(const char *sym_name, symTableElement *tab)
{
  int i;
  for (i = 0; i < getSymbolTableSize('w'); i++)
    if (strcmp(tab[i].name, sym_name) == 0)
      return &tab[i];
  return 0;
}

/*****************************************
 * odometry
 */

#define WHEEL_DIAMETER 0.06522 /* m */
#define WHEEL_SEPARATION 0.26  /* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define DEFAULT_ROBOTPORT 24902

typedef struct
{                          // input signals
  int left_enc, right_enc; // encoderticks
  // parameters
  double w;      // wheel separation
  double cr, cl; // meters per encodertick
                 // output signals
  double right_pos, left_pos;
  // internal variables
  int left_enc_old, right_enc_old;
  // Odometry
  double x, y, theta;
} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);

/********************************************
 * Motion control
 */

typedef struct
{ // input
  int cmd;
  int curcmd;
  double speedcmd;
  double dist;
  double angle;
  double theta_ref; // This is new
  double left_pos, right_pos;
  // parameters
  double w;
  // output
  double motorspeed_l, motorspeed_r;
  int finished;
  // internal variables
  double startpos;
  // For sensors
  int linecolor; //0 for black 1 for white
  int orientation;
  bool sensorstop;
  bool crossingline;


} motiontype;

enum
{
  mot_stop = 1,
  mot_move,
  mot_turn,
  mot_line, //Homemade line follow function
  mot_wall //Homemade wall follow function
};

enum
{
  ms_init,
  ms_fwd,
  ms_turn,
  ms_line,
  ms_wall,
  ms_end
};

typedef struct
{
  int state, oldstate;
  int time;
} smtype;

/********************************************
 * Custom Headers
 */
void sm_update(smtype *p);
void update_motcon(motiontype *p, odotype *o);
int fwd(double speed, int time, double dist, bool sensorstop);
int turn(double angle, double speed, int time);
int line(double speed, int linecolor, bool crossingline, int orientation, int time, double dist, bool sensorstop); // New
int wall(double speed, int orientation, int time, double dist, bool sensorstop); // New
double *calibrate_line(symTableElement *linesensor_values);
int find_line_min(double *sensor_values, int orientation, int linecolor);
bool compare_floats(float f1, float f2);

odotype odo;
smtype mission;
motiontype mot;

// SMR input/output data

symTableElement *inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl;


/********************************************
 * Error handling
 */

void segfaulthandler(int sig)
{
  //    perror(NULL);
  printf("Seg-error\n");
  exit(1);
}

void brokenpipehandler(int sig)
{
  printf("mrc: broken pipe \n");
  // savelog("log");
  exit(1);
}

void ctrlchandler(int sig)
{
  printf("mrc: ctrl-c \n");
  running = 0;
}

/********************************
*  __  __          _____ _   _  *
* |  \/  |   /\   |_   _| \ | | *
* | \  / |  /  \    | | |  \| | *
* | |\/| | / /\ \   | | | . ` | *
* | |  | |/ ____ \ _| |_| |\  | *
* |_|  |_/_/    \_\_____|_| \_| *
*                               *
*********************************/

int main(int argc, char **argv)
{
  int n = 0, arg, time = 0, opt, calibration;
  double dist = 0, angle = 0, speed = 0;
  // install sighandlers
  if (1)
  {
    if (signal(SIGSEGV, segfaulthandler) == SIG_ERR)
    {
      perror("signal");
      exit(1);
    }
  }
  if (signal(SIGPIPE, brokenpipehandler) == SIG_ERR)
  {
    perror("signal");
    exit(1);
  }
  if (signal(SIGINT, ctrlchandler) == SIG_ERR)
  {
    perror("signal");
    exit(1);
  }
  robot_port = DEFAULT_ROBOTPORT;
  while (EOF != (opt = getopt(argc, argv, "ct:v:l:s:h:u")))
  {
    switch (opt)
    {
    case 'c':
      calibration = 1;
      break;

    case 's':
      if (optarg)
      {
        int port;
        port = atoi(optarg);
        if (port != 0)
          robot_port = port;
      }
      else
        exit(1);
      break;

    default:;
    }
  }

  /* Establish connection to robot sensors and actuators.
   */
  if (rhdConnect('w', "localhost", robot_port) != 'w')
  {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }

  printf("connected to robot \n");
  if ((inputtable = getSymbolTable('r')) == NULL)
  {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }
  if ((outputtable = getSymbolTable('w')) == NULL)
  {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }
  // connect to robot I/O variables
  lenc = getinputref("encl", inputtable);
  renc = getinputref("encr", inputtable);
  linesensor = getinputref("linesensor", inputtable);
  irsensor = getinputref("irsensor", inputtable);

  speedl = getoutputref("speedl", outputtable);
  speedr = getoutputref("speedr", outputtable);
  resetmotorr = getoutputref("resetmotorr", outputtable);
  resetmotorl = getoutputref("resetmotorl", outputtable);
    


  // **************************************************
  //  Camera server code initialization
  //

  /* Create endpoint */
  lmssrv.port = 24919;
  strcpy(lmssrv.host, "127.0.0.1");
  strcpy(lmssrv.name, "laserserver");
  lmssrv.status = 1;
  camsrv.port = 24920;
  strcpy(camsrv.host, "127.0.0.1");
  camsrv.config = 1;
  strcpy(camsrv.name, "cameraserver");
  camsrv.status = 1;

  if (camsrv.config)
  {
    int errno = 0;
    camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (camsrv.sockfd < 0)
    {
      perror(strerror(errno));
      fprintf(stderr, " Can not make  socket\n");
      exit(errno);
    }

    serverconnect(&camsrv);

    xmldata = xml_in_init(4096, 32);
    printf(" camera server xml initialized \n");
  }

  // **************************************************
  //  LMS server code initialization
  //

  /* Create endpoint */
  lmssrv.config = 1;
  if (lmssrv.config)
  {
    char buf[256];
    int errno = 0, len;
    lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (lmssrv.sockfd < 0)
    {
      perror(strerror(errno));
      fprintf(stderr, " Can not make  socket\n");
      exit(errno);
    }

    serverconnect(&lmssrv);
    if (lmssrv.connected)
    {
      xmllaser = xml_in_init(4096, 32);
      printf(" laserserver xml initialized \n");
      len = sprintf(buf, "push  t=0.2 cmd='mrcobst width=0.4'\n");
      send(lmssrv.sockfd, buf, len, 0);
    }
  }
  /* Read sensors and zero our position.
   */
  rhdSync();

  odo.w = 0.256;
  odo.cr = DELTA_M;
  odo.cl = odo.cr;
  odo.left_enc = lenc->data[0];
  odo.right_enc = renc->data[0];
  reset_odo(&odo);
  // printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w = odo.w;

  // zero theoretical angle:
  mot.theta_ref = 0.0;
  running = 1;
  mission.state = ms_init;
  mission.oldstate = -1;

  /*******************************
  *  _      ____   ____  _____   *
  * | |    / __ \ / __ \|  __ \  * 
  * | |   | |  | | |  | | |__) | *
  * | |   | |  | | |  | |  ___/  *
  * | |___| |__| | |__| | |      *
  * |______\____/ \____/|_|      *
  *                              *
  ********************************/
  while (running)
  {
    if (lmssrv.config && lmssrv.status && lmssrv.connected)
    {
      while ((xml_in_fd(xmllaser, lmssrv.sockfd) > 0))
        xml_proca(xmllaser);
    }

    if (camsrv.config && camsrv.status && camsrv.connected)
    {
      while ((xml_in_fd(xmldata, camsrv.sockfd) > 0))
        xml_proc(xmldata);
    }

    rhdSync();
    odo.left_enc = lenc->data[0];
    odo.right_enc = renc->data[0];
    update_odo(&odo);

    /****************************************
    / mission statemachine
    */
    sm_update(&mission);
    switch (mission.state)
    {
    case ms_init:
      n = 1; // This should be the number of commands in the parsed smr-cl script.
      dist = 3;
      angle = 0.0 / 180 * M_PI;
      speed = 0.2;
      mission.state = ms_fwd; // Change between ms_fwd for square or straightline program, or ms_followline for followline program.
      break;
    }
    /*  end of mission  */

    // Update odometry

    mot.left_pos = odo.left_pos;
    mot.right_pos = odo.right_pos;
    update_motcon(&mot, &odo);
    speedl->data[0] = 100 * mot.motorspeed_l;
    speedl->updated = 1;
    speedr->data[0] = 100 * mot.motorspeed_r;
    speedr->updated = 1;

    if (time % 100 == 0)
      time++;

    /* stop if keyboard is activated
     *
     */
    ioctl(0, FIONREAD, &arg);
    if (arg != 0)
      running = 0;

  } /* end of main control loop */
  speedl->data[0] = 0;
  speedl->updated = 1;
  speedr->data[0] = 0;
  speedr->updated = 1;
  rhdSync();
  rhdDisconnect();

  printf("Done Executing Program! \n");
  exit(0);

}
/************************
*  ______ _   _ _____   *
* |  ____| \ | |  __ \  *
* | |__  |  \| | |  | | *
* |  __| | . ` | |  | | *
* | |____| |\  | |__| | *
* |______|_| \_|_____/  *
*                       *
*************************/

/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */

void reset_odo(odotype *p)
{
  p->right_pos = p->left_pos = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
  p->x = 0.0; // can be changed to 0.5
  p->y = 0.0; // can be changed to 2.0
  p->theta = 0.0;
}

void update_odo(odotype *p)
{
  int delta;
  double delta_right; // The delta is the N_r or N_l (encoder output count)
  double delta_left;
  double delta_U;
  double delta_theta;

  delta = p->right_enc - p->right_enc_old;
  if (delta > 0x8000)
    delta -= 0x10000;
  else if (delta < -0x8000)
    delta += 0x10000;
  p->right_enc_old = p->right_enc;
  delta_right = delta * p->cr;
  p->right_pos += delta_right;

  delta = p->left_enc - p->left_enc_old;
  if (delta > 0x8000)
    delta -= 0x10000;
  else if (delta < -0x8000)
    delta += 0x10000;
  p->left_enc_old = p->left_enc;
  delta_left = delta * p->cl;
  p->left_pos += delta_left;

  // Update odometry measures x, y, theta
  delta_U = (delta_right + delta_left) / 2.0; // Note that right pos and left pos constitues delta U_r and delta U_l
  delta_theta = (delta_right - delta_left) / WHEEL_SEPARATION;

  p->theta = p->theta + delta_theta;
  p->x = p->x + delta_U * cos(p->theta / 180 * M_PI);
  p->y = p->y + delta_U * sin(p->theta / 180 * M_PI);
}

/******************************************************
*    _____ ____  _   _ _______ _____   ____  _        *
*   / ____/ __ \| \ | |__   __|  __ \ / __ \| |       *
*  | |   | |  | |  \| |  | |  | |__) | |  | | |       *
*  | |   | |  | | . ` |  | |  |  _  /| |  | | |       *
*  | |___| |__| | |\  |  | |  | | \ \| |__| | |____   *
*   \_____\____/|_| \_|  |_|  |_|  \_\\____/|______|  *
*                                                     *
*******************************************************/

void update_motcon(motiontype *p, odotype *o)
{
  if (p->cmd != 0)
  {

    p->finished = 0;
    switch (p->cmd)
    {
    case mot_stop:
      p->curcmd = mot_stop;
      break;
    case mot_move:
      p->startpos = (p->left_pos + p->right_pos) / 2;
      p->curcmd = mot_move;
      break;

    case mot_turn:
      p->theta_ref = p->theta_ref + p->angle;
      if (p->angle > 0)
        p->startpos = p->right_pos;
      else
        p->startpos = p->left_pos;
      p->curcmd = mot_turn;
      break;

    case mot_line: //This is new
      p->startpos = (p->left_pos + p->right_pos) / 2;
      p->curcmd = mot_line;
      break;
    
    case mot_wall: //
      break;
    }

    p->cmd = 0;
  }

  double remaining_dist;
  double max_V;
  double remaining_angle;
  
  // Custom values for follow_line sensor functionality
  double *calibrated_sensorvalues;
  int sensor_index;
  int delta_V;

  switch (p->curcmd)
  {
  case mot_stop:
    p->motorspeed_l = 0;
    p->motorspeed_r = 0;
    break;

  case mot_move:
    remaining_dist = p->dist - ((p->right_pos + p->left_pos) / 2 - p->startpos);
    max_V = sqrt(2*0.5*fabs(remaining_dist));

    // Check if goal is reached:
    if (((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist) | mot.sensorstop)
    {
      p->finished = 1;
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
    }

    // Decrease velocity:
    else if (max_V < p->speedcmd)
    {
      p->motorspeed_l = max_V;
      p->motorspeed_r = max_V;
    }

    // If not goal is reached then we should accelerate until we hit max velocity
    else
    {
      // If the motorspeed of any motor is higher than the allowed speed given by the speedcmd, set top speed.
      if ((p->speedcmd < p->motorspeed_l) | (p->speedcmd < p->motorspeed_r))
      {
        p->motorspeed_l = p->speedcmd;
        p->motorspeed_r = p->speedcmd;
      }
      //Otherwise accelerate!
      else
      {
        p->motorspeed_l += 0.5/100;
        p->motorspeed_r += 0.5/100;
      }
    }
    break;

  case mot_turn:
    // If we have to turn left (the angle is positive)
    if (p->angle>0){

      //Calculate the remaining angle to turn:
      remaining_angle = ((p->angle*p->w)/2) - (p->right_pos-p->startpos);
	    max_V = sqrt(2 * 0.5 * fabs(remaining_angle));

      // Have we finished turning? If so stop!
      if (p->right_pos-p->startpos > (p->angle*p->w)/2){
        p->motorspeed_r=0;
        p->motorspeed_l=0;
        p->finished=1;
	    }

      //Decrease velocity for turning:
      else if (max_V < p->speedcmd/2)
      {
        p->motorspeed_r = max_V;
        p->motorspeed_l = -max_V;
      }

      //Else, if not angular goal is reached, we accelerate
      else
      {
        //If we hit max speed on any wheel (in positive or negative direction, set to max speed)
        if ((p->motorspeed_r > p->speedcmd/2) | (p->motorspeed_l < -p->speedcmd/2))
        {
          p->motorspeed_r = p->speedcmd/2;
          p->motorspeed_l = -p->speedcmd/2;
        }

        //Else we should accelerate!
        else 
        {
          p->motorspeed_r += 0.5 / 100;
          p->motorspeed_l += -(0.5 / 100);
        }
      }	
	  }
	  else //Else we do a right turn
    {
      //Calculate the remaining angle to turn:
      remaining_angle = (fabs(p->angle*p->w)/2) - (p->left_pos-p->startpos);
	    max_V = sqrt(2 * 0.5 * fabs(remaining_angle));

      //Have we finished turning? If so stop!
	    if (p->left_pos-p->startpos > (fabs(p->angle)*p->w)/2)
      {
        p->motorspeed_l=0;
        p->motorspeed_r=0;
        p->finished=1;

	    }
      //Decrease velocity for turning:
      else if (max_V < p->speedcmd/2)
      {
        p->motorspeed_r = -max_V;
        p->motorspeed_l = max_V;
      }
	    else {
        //If we hit max speed on any wheel (in positive or negative direction, set to max speed)
        if ((p->motorspeed_r < -p->speedcmd/2) | (p->motorspeed_l > p->speedcmd/2))
        {
          p->motorspeed_r = -p->speedcmd/2;
          p->motorspeed_l = p->speedcmd/2;
        }

        //Else we should accelerate!
        else 
        {
          p->motorspeed_r += -(0.5 / 100);
          p->motorspeed_l += 0.5 / 100;
        }
	    }
	  }
    break;

  case mot_line:
    calibrated_sensorvalues = calibrate_line(linesensor);
    sensor_index = find_line_min(calibrated_sensorvalues, mot.orientation, mot.linecolor); // 0, hold right, 1 hold left.
    
    remaining_dist = p->dist -((p->right_pos + p->left_pos) / 2 - p->startpos); // Calculate remaining distance
    max_V = sqrt(2*0.5*fabs(remaining_dist));
    delta_V = 0.01 * (3-sensor_index);

    // Check if destination is reached or sensors tell motor to stop.
    if (((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist) |  mot.sensorstop)
    {
      p->finished = 1;
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
    }

    // Stop if meeting af crossling line of any color
    if (mot.crossingline && (sensor_index == -1))
    {
      p->finished = 1;
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
    }

    if (sensor_index == -2)
    {
      printf("Error! - DO SOMETHING!");
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
    }

    // Check if any speed  surpasses max allowed speed, or else set it.
    if (p->speedcmd < fabs(p->motorspeed_l)) p->motorspeed_l = p->speedcmd;
    if (p->speedcmd < fabs(p->motorspeed_r)) p->motorspeed_r = p->speedcmd;
    
    if (delta_V == 0)
    {
      // Decrease velocity:
      if (max_V < p->speedcmd)
      {
        p->motorspeed_l = max_V;
        p->motorspeed_r = max_V;
      }
      else // Else accelerate
      {
        p->motorspeed_l += 0.5 / 100;
        p->motorspeed_r += 0.5 / 100;
      }
    }
    // If less than 0, this means the sensor index is large (ie. to the left, and we should turn this way)
    else if (delta_V < 0) p->motorspeed_l += delta_V; //Delta in this case is negative, we should decrease speed on left wheel
    
    // If more than 0, this means the sensor index is small (ie. to the right, and we should turn this way)
    else if (delta_V > 0) p->motorspeed_r -= delta_V; // Delta in this case is positive, we should decrease speed on right wheel
    break;
  
  case mot_wall:
    printf("Not yet implemented!");
    break;

  }
}

/*************************************************
*    __                  _   _                   *
*   / _|                | | (_)                  *
*  | |_ _   _ _ __   ___| |_ _  ___  _ __  ___   *
*  |  _| | | | '_ \ / __| __| |/ _ \| '_ \/ __|  *
*  | | | |_| | | | | (__| |_| | (_) | | | \__ \  *
*  |_|  \__,_|_| |_|\___|\__|_|\___/|_| |_|___/  *
*                                                *
*************************************************/

int fwd(double speed, int time, double dist, bool sensorstop)
{
  if (time == 0)
  {
    mot.cmd = mot_move;
    mot.speedcmd = speed;
    mot.dist = dist;
    mot.sensorstop = sensorstop;
    return 0;
  }
  else
    return mot.finished;
}

int turn(double angle, double speed, int time)
{
  if (time == 0)
  {
    mot.cmd = mot_turn;
    mot.speedcmd = speed;
    mot.angle = angle;
    return 0;
  }
  else
    return mot.finished;
}

int line(double speed, int linecolor, bool crossingline, int orientation, int time, double dist, bool sensorstop)
{
  if (time == 0)
  {
    mot.cmd = mot_line;
    mot.speedcmd = speed;
    mot.linecolor = linecolor;
    mot.crossingline = crossingline;
    mot.dist = dist;
    mot.sensorstop = sensorstop;
    mot.orientation = orientation;
    return 0;
  }
  else
    return mot.finished;
}

int wall(double speed, int orientation, int time, double dist, bool sensorstop)
{
  printf("Not implemented!");
}

void sm_update(smtype *p)
{
  if (p->state != p->oldstate)
  {
    p->time = 0;
    p->oldstate = p->state;
  }
  else
  {
    p->time++;
  }
}

double *calibrate_line(symTableElement *linesensor_values)
{
  // Converta an uncalibrated linesnesor value to a calibrated one.
  static double readings[8];
  int i;

  for (i = 0; i < 8; i++)
  {
    readings[i] = (linesensor_values->data[i])/ 255.0;
  }
  return readings;
  }

int find_line_min(double *sensor_values, int orientation, int linecolor)
{
  // This function finds the position of the black line 
  // using the position of the lowest calibrated linesensor.
  // Note that we need to loop backwards over the values

  int chosen_index = -2;
  int i;
  int all_equal = 1;
  double sum = 0.0;

  if (linecolor == 0)
  {
    int curr_min = 1.0;
    // Loop backwards over the input array of sensor values:
    for (i=7; i--> 0;)
    {
      if (orientation == 0)
      {
        if (sensor_values[i] < curr_min)
        {
          curr_min = sensor_values[i];
          chosen_index = i;
        }
      }
      else
      {
        if (sensor_values[i] <= curr_min)
        {
          curr_min = sensor_values[i];
          chosen_index = i;
        }
      }
      // Finally we desire to check if all the sensor values are the same
      sum += sensor_values[i];
    }
  }
  else if (linecolor == 1)
  {
    int curr_max = 0.0;
    // Loop backwards over the input array of sensor values:
    for (i=7; i--> 0;)
    {
      if (orientation == 0)
      {
        if (sensor_values[i] > curr_max)
        {
          curr_max = sensor_values[i];
          chosen_index = i;
        }
      }
      else
      {
        if (sensor_values[i] >= curr_max)
        {
          curr_max = sensor_values[i];
          chosen_index = i;
        }
      }
      // Finally we desire to check if all the sensor values are the same
      sum += sensor_values[i];
    }
  }
  for (i = 0; i<8; i++)
  {
    if (compare_floats(sum/8.0, sensor_values[i]))
    {
      all_equal *= 1;
    }
    else all_equal *= 0;
  }
  if (all_equal == 1) return -1;
  else return chosen_index; 
}

bool compare_floats(float f1, float f2)
{
  // Code taken from: 
  // https://how-to.fandom.com/wiki/Howto_compare_floating_point_numbers_in_the_C_programming_language
  float precision = 0.00001;
  if (((f1 - precision) < f2) && ((f1 + precision) > f2)) // Check if we are within interval
  {
    return true;
  }
  else
  {
    return false;
  }
}

