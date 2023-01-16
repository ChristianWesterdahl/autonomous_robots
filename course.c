#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
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

/********************************************
 * Motion control
 */

typedef struct
{ // input
  int cmd;
  int curcmd;
  double speedcmd;
  double accelerationcmd;
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
  //Sensor variables
  int linecolor; //0 for black 1 for white
  int followDir;
  bool sensorstop;
  bool crossingline;
  double walldist;

} motiontype;

/*****************************************
 * odometry
 */
#define WHEEL_DIAMETER 0.06522 /* m */
#define WHEEL_SEPARATION 0.26  /* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define DEFAULT_ROBOTPORT 24902
#define k 0.001
#define KA 16.0 // For ir sensor
#define KB 76.0 // For ir sensor
#define MAX_LINE 130.0 // For linesensor 

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

void reset_odo(motiontype *p, odotype *o);
void update_odo(odotype *p);


enum mot
{
  mot_stop = 1,
  mot_move,
  mot_turn,
  mot_line, //Homemade line follow function
  mot_wall
};

enum ms
{
  ms_init,
  ms_fwd,
  ms_turn,
  ms_line,
  ms_resetOdo,
  ms_wall,
  ms_end
};

typedef struct
{
  int state, oldstate;
  int time;
} smtype;

void sm_update(smtype *p);

void update_motcon(motiontype *p, odotype *o);

int fwd(double dist, double speed, double acceleration, bool sensorstop, int time);
int turn(double angle, double speed, int time);
//int follow(double dist, double speed, int time, int dir); // New (0 = left, 1 = right)
int line(double dist, double speed, int dir, int linecolor, bool crossingline, bool sensorstop, int time); // New
int wall(double speed, int dir, double walldist, int time, double dist, bool sensorstop); // New
//auxilory function (all new)
double *calibrate_line(symTableElement *linesensor_values);
//int find_line_min(double *sensor_values, int orientation); old follow line
int find_line_min(double *sensor_values, int orientation, int linecolor);
bool compare_floats(float f1, float f2);
double line_COM(double *sensor_values);
bool sensorstop(int sensor, double condition, int mode);
double readsensor(int sensor);

//------------------------deffining the course---------------------------
/*
fwd(dist, speed)
turn(angle, speed)
line(dist, speed, dir, col, crossingLine, sensorStop) #col: 0-black, 1-white #dir: 0-right 1-left
resetOdo()
measure(laser_compensation)
*/

//methods
enum ms course_methods[500] = {
  //course here
  //ms_line,
  //ms_resetOdo,
  ms_turn,
  ms_fwd,
  ms_end
  };

//method variables (make sure these fit together with the methods list, and use all variables acording to the list above)
double course_vars[500] = {
  //1.0, 0.2, 0, 0, //line
  90/180.0*M_PI, 0.2,
  0.5, 0.4
  //course variables here
  };
//------------------------end of course----------------------------------

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
  bool change_var; //to change the method variable counter var_i
  int n = 0, courseLength = 0, i_var = 0, dir = 0, linecolor = 0, arg, time = 0, opt, calibration;
  double dist = 0, angle = 0, speed = 0, acceleration = 0;
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
      len=sprintf(buf,"scanpush cmd='zoneobst'\n");
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
  reset_odo(&mot, &odo);
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
    printf("var_i: %d, course state: %d\n", i_var, courseLength-n);
    sm_update(&mission);
    switch (mission.state)
    {
    case ms_init:
      n = (sizeof(course_methods)/sizeof(course_methods[0]))-1;
      courseLength = n;
      acceleration = 0.5;
      mission.state = course_methods[courseLength-n];
      printf("Mission state: %d\n", mission.state);
      change_var = 1;
    break;
    
    //forward
    case ms_fwd:
      if (change_var) 
      {
        dist = course_vars[i_var]; i_var++;
        speed = course_vars[i_var]; i_var++;
        printf("fwd: (%f,%f)\n", dist, speed);
        change_var = false;
      }
      mot.sensorstop = sensorstop(10, 10.0, 0); // Replace with course_vars[];
      if (fwd(dist, speed, acceleration, 0, mission.time)) 
      {
        n = n-1;
        mission.state = course_methods[courseLength-n];
        change_var = true;
        printf("fwd ended\n");
      }
    break;

    //turn
    case ms_turn:
      if (change_var) 
      {
        angle = course_vars[i_var]; i_var++;
        speed = course_vars[i_var]; i_var++;
        change_var = false;
      }
      printf("turn: (%d,%f)\n", i_var, angle);
      if (turn(angle, speed, mission.time)) 
      {
        n = n-1;
        mission.state = course_methods[courseLength-n];
        change_var = true;    
      }
    break;

    //follow line
    case ms_line:
      if (change_var) 
      {
        dist = course_vars[i_var]; i_var++;
        speed = course_vars[i_var]; i_var++;
        dir = course_vars[i_var]; i_var++;
        linecolor = course_vars[i_var]; i_var++;
        printf("follow: (%f,%f,%d,%d)\n", dist, speed, dir, linecolor);
        change_var = false;
      }
      mot.sensorstop = sensorstop(10, 10.0, 0);
      printf("follow middle");
      if(line(dist, speed, dir, linecolor, false, false, mission.time)) 
      {
        n = n-1;
        mission.state = course_methods[courseLength-n];
        change_var = true;
        
      }
    break;

    case ms_resetOdo:
      reset_odo(&mot, &odo);
      n = n-1;
      mission.state = course_methods[courseLength-n];
      change_var = true;
      printf("odo reset\n");      
    break;

    //end
    case ms_end:
    printf("end\n");
      mot.cmd = mot_stop;
      running = 0;
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

void reset_odo(motiontype *p ,odotype *o)
{
  o->right_pos = o->left_pos = 0.0;
  o->right_enc_old = o->right_enc;
  o->left_enc_old = o->left_enc;
  o->x = 0.0;
  o->y = 0.0; 
  o->theta = 0.0;
  p->cmd = mot_stop;
  p->theta_ref = 0;

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
  p->x = p->x + delta_U * cos(p->theta / 180 * M_PI); //might be changed to use radiends on some systems
  p->y = p->y + delta_U * sin(p->theta / 180 * M_PI); //might be changed to use radiends on some systems
}

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

    case mot_wall:
      p->startpos = (p->left_pos + p->right_pos) / 2;
      p->curcmd = mot_wall;
    }
    p->cmd = 0;
  }

  double remaining_dist;
  double v_max;
  double v_delta;
  double remaining_angle;
  
  // Custom values for follow_line sensor functionality
  double *calibrated_sensorvalues;
  int sensor_index;
  double sensor_value;

  switch (p->curcmd)
  {
  //-----------------------------------------------forward---------------------------------------
  case mot_move:
    printf("GOING FORWARD!\n");
    if ( ((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist && p->dist > 0.0) || ((p->right_pos + p->left_pos) / 2 - p->startpos < p->dist && p->dist < 0.0) || p->dist == 0 || mot.sensorstop)
    {  
      p->finished = 1;
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
    }
    else
    {
      int forward = 1;
      if (p->dist < 0.0) {forward = -1;}
      //addding deceleration
      double distance_to_destination = fabs((p->dist-((p->right_pos + p->left_pos) / 2 - p->startpos)));
      v_max = sqrt(2*p->accelerationcmd*distance_to_destination); //calculating V_max=sqrt(2*a_max*d)


      if ( (p->motorspeed_l > v_max && p->dist > 0.0) || (p->motorspeed_l < -v_max && p->dist < 0.0) ) //decellerating cap
      {
        p->motorspeed_l = forward*v_max;
        p->motorspeed_r = forward*v_max;
      }
      else if (p->speedcmd-fabs(p->motorspeed_l) > p->accelerationcmd/100) //if new speed diffurence from old speed is less than accelleration(/100 do to update rate)
      {
        p->motorspeed_l = p->motorspeed_l + forward*(p->accelerationcmd/100);
        p->motorspeed_r = p->motorspeed_r + forward*(p->accelerationcmd/100);
      } 
      else 
      {
        p->motorspeed_l = forward*p->speedcmd;
        p->motorspeed_r = forward*p->speedcmd;
      }
    }
  break;

  //------------------------------------turning-----------------------------------
  case mot_turn:
    printf("TURNING THE ROBOT!\n");
    printf("Angle: %f\n", p->angle);

    // If we have to turn left (the angle is positive)
    if (p->angle>0){
      printf("POSITIVE ANGLE!\n");
      //Calculate the remaining angle to turn:
      remaining_angle = ((p->angle*p->w)/2) - (p->right_pos-p->startpos);
	    v_max = sqrt(2 * 0.5 * fabs(remaining_angle));

      // Have we finished turning? If so stop!
      if (p->right_pos-p->startpos > (p->angle*p->w)/2){
        printf("WE STOP TURNING!\n");
        p->motorspeed_r=0;
        p->motorspeed_l=0;
        p->finished=1;
	    }

      //Decrease velocity for turning:
      else if (v_max < p->speedcmd/2)
      {
        p->motorspeed_r = v_max;
        p->motorspeed_l = -v_max;
      }

      //Else, if not angular goal is reached, we accelerate
      else
      {
        printf("Accelerate!\n");
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
	    v_max = sqrt(2 * 0.5 * fabs(remaining_angle));

      //Have we finished turning? If so stop!
	    if (p->left_pos-p->startpos > (fabs(p->angle)*p->w)/2)
      {
        p->motorspeed_l=0;
        p->motorspeed_r=0;
        p->finished=1;

	    }
      //Decrease velocity for turning:
      else if (v_max < p->speedcmd/2)
      {
        p->motorspeed_r = -v_max;
        p->motorspeed_l = v_max;
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
  /*
  case mot_turn:
    printf("TURNING THE ROBOT\n");
    angular_distance = p->theta_ref - o->theta;
    if (angular_distance > 0 && angular_distance <= M_PI)
    { // If we have to turn left (the angle is less than 180 degrees and postiive)
      if (angular_distance > 0.5 / 180 * M_PI)
      { // Condition suff small keep turning
        p->motorspeed_r = p->motorspeed_r + k * angular_distance;
        p->motorspeed_l = p->motorspeed_l - k * angular_distance;
      }
      else
      {
        //printf("Done turning!. Final angular distance: %f\n", angular_distance);
        p->motorspeed_r = 0;
        p->motorspeed_l = 0;
        p->finished = 1;
      }
    }
    else
    { // Else we turn right
      if (angular_distance < -0.5 / 180 * M_PI)
      {
        p->motorspeed_l = p->motorspeed_l - k * angular_distance;
        p->motorspeed_r = p->motorspeed_r + k * angular_distance;
      }
      else
      {
        //printf("Done turning!. Final angular distance: %f\n", angular_distance);
        p->motorspeed_l = 0;
        p->motorspeed_r = 0;
        p->finished = 1;
      }
    }
  break;
  */
  
  //------------------------following line------------------------------------------

  case mot_line:
      calibrated_sensorvalues = calibrate_line(linesensor);
      sensor_index = find_line_min(calibrated_sensorvalues, mot.followDir, mot.linecolor); // 0, hold right, 1 hold left.
      
      remaining_dist = p->dist -((p->right_pos + p->left_pos) / 2 - p->startpos); // Calculate remaining distance
      v_max = sqrt(2*0.5*fabs(remaining_dist));
      v_delta = 0.00005 * (3-sensor_index);
      
      printf("sensor index: %d, v_delta: %f, v_max: %f, maxspeed: %f\n", sensor_index, v_delta, v_max, p->speedcmd);
      // Check if destination is reached or sensors tell motor to stop.
      if (((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist)) // | mot.sensorstop)
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
      
      if (v_delta == 0)
      {
        // Decrease velocity:
        if (v_max < p->speedcmd)
        {
          p->motorspeed_l = v_max;
          p->motorspeed_r = v_max;
        }
        else // Else accelerate
        {
          p->motorspeed_l += 0.5 / 100;
          p->motorspeed_r += 0.5 / 100;
        }
      }
      // If less than 0, this means the sensor index is large (ie. to the left, and we should turn this way)
      else if (v_delta < 0) 
      {
        p->motorspeed_l += v_delta; //Delta in this case is negative, we should decrease speed on left wheel
        p->motorspeed_r -= v_delta;
      }
      // If more than 0, this means the sensor index is small (ie. to the right, and we should turn this way)
      else if (v_delta > 0)
      {
        p->motorspeed_r -= v_delta; // Delta in this case is positive, we should decrease speed on right wheel
        p->motorspeed_l += v_delta;
      }
      printf("Motorspeeds: l - %f, r - %f\n", p->motorspeed_l, p->motorspeed_r);
  break;

  //--------------------------------------------wall---------------------------------------------
  case mot_wall:
    remaining_dist = p->dist -((p->right_pos + p->left_pos) / 2 - p->startpos); // Calculate remaining distance
    v_max = sqrt(2*0.5*fabs(remaining_dist));

    if (mot.followDir == 0) //If 0 then keep wall to left
    {
      sensor_value = readsensor(10);
      v_delta = 0.1*(mot.walldist - sensor_value); // If this is negative, robot is too far from wall, if positive too close. It should be zero.

      // From here we utilize the same code as for the linesensor
      // Check if destination is reached or sensors tell motor to stop.
      if (((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist) | mot.sensorstop)
      {
        p->finished = 1;
        p->motorspeed_l = 0;
        p->motorspeed_r = 0;
      }

      // Check if any speed  surpasses max allowed speed, or else set it.
      if (p->speedcmd < fabs(p->motorspeed_l)) p->motorspeed_l = p->speedcmd;
      if (p->speedcmd < fabs(p->motorspeed_r)) p->motorspeed_r = p->speedcmd;
      
      if (fabs(v_delta) < 0.00001) //Compare with zero, using precision.
      {
        // Decrease velocity:
        if (v_max < p->speedcmd)
        {
          p->motorspeed_l = v_max;
          p->motorspeed_r = v_max;
        }
        else // Else accelerate
        {
          p->motorspeed_l += 0.5 / 100;
          p->motorspeed_r += 0.5 / 100;
        }
      }
      // If less than 0, this means the distance to the wall is too large, in that case we should turn left (as we want the wall on the LEFT side of the robot)
      else if (v_delta < 0) 
      {
        p->motorspeed_l += v_delta; //Delta in this case is negative, we should decrease speed on left wheel
        p->motorspeed_r -= v_delta;
      }
      // If more than 0, this means the sensor index is small (ie. to the right, and we should turn this way)
      else if (v_delta > 0)
      {
        p->motorspeed_r -= v_delta; // Delta in this case is positive, we should decrease speed on right wheel
        p->motorspeed_l += v_delta;
      }


    }
    else if (mot.followDir == 1) //If 1 then keep wall to right
    {
      sensor_value = readsensor(14);
      v_delta = 0.1*(mot.walldist - sensor_value); // If this is negative, robot is too far from wall, if positive too close. It should be zero.

      // From here we utilize the same code as for the linesensor
      // Check if destination is reached or sensors tell motor to stop.
      if (((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist) | mot.sensorstop)
      {
        p->finished = 1;
        p->motorspeed_l = 0;
        p->motorspeed_r = 0;
      }

      // Check if any speed  surpasses max allowed speed, or else set it.
      if (p->speedcmd < fabs(p->motorspeed_l)) p->motorspeed_l = p->speedcmd;
      if (p->speedcmd < fabs(p->motorspeed_r)) p->motorspeed_r = p->speedcmd;
      
      if (fabs(v_delta) < 0.00001) //Compare with zero, using precision.
      {
        // Decrease velocity:
        if (v_max < p->speedcmd)
        {
          p->motorspeed_l = v_max;
          p->motorspeed_r = v_max;
        }
        else // Else accelerate
        {
          p->motorspeed_l += 0.5 / 100;
          p->motorspeed_r += 0.5 / 100;
        }
      }
      // If less than 0, this means the distance to the wall is too large, in that case we should turn right (as we want the wall on the RIGHT side of the robot)
      else if (v_delta < 0) 
      {
        p->motorspeed_l -= v_delta; 
        p->motorspeed_r += v_delta;
      }
      else if (v_delta > 0)
      {
        p->motorspeed_r += v_delta;
        p->motorspeed_l -= v_delta;
      }
    }

  break;

  //------------------------------------------stopping-------------------------------------------
  case mot_stop:
    p->motorspeed_l = 0;
    p->motorspeed_r = 0;
  break;
  }
}

int fwd(double dist, double speed, double acceleration, bool sensorstop, int time)
{
  if (time == 0)
  {
    mot.cmd = mot_move;
    mot.speedcmd = speed;
    mot.accelerationcmd = acceleration;
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

int line(double dist, double speed, int dir, int linecolor, bool crossingline, bool sensorstop, int time)
{ // New{
  if (time == 0)
  {
    mot.cmd = mot_line;
    mot.speedcmd = speed;
    mot.dist = dist;
    mot.followDir = dir;
    mot.linecolor = linecolor;
    mot.sensorstop = sensorstop;
    mot.crossingline = crossingline;
    return 0;
  }
  else
    return mot.finished;
}

int wall(double speed, int dir, double walldist, int time, double dist, bool sensorstop)
{
  mot.cmd = mot_wall;
  mot.speedcmd = speed;
  mot.dist = dist,
  mot.followDir = dir;
  mot.sensorstop = sensorstop;
  mot.walldist = walldist;
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

//aouxilory methods
double *calibrate_line(symTableElement *linesensor_values)
{
  // This function converts uncalibrated linesensor values to calibrated
  // using the linear transformation described in the calibration exercise.
  static double r[8];
  int i;

  printf("Linesensor:");
  for (i = 0; i < 8; i++){
    //printf(" %d", linesensor_values->data[i]);
    r[i] = (linesensor_values->data[i]) / MAX_LINE;
    if (r[i] > 0.62) r[i] += 0.2;
    else r[i] -= 0.2;
    printf(" %f", r[i]);
  }
  printf("\n");
  return r;
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
    double curr_min = 1.0;
    // Loop backwards over the input array of sensor values:
    for (i=8; i--> 0;)
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
    double curr_max = 0.0;
    // Loop backwards over the input array of sensor values:
    for (i=8; i--> 0;)
    {
      printf(", %f", sensor_values[i]);
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
    printf("\n");
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
  else 
  {
    return chosen_index; 
  }
}

double line_COM(double *sensor_values)
{
  double distance[8] = {-0.065,-0.045,-0.027,-0.009,0.009,0.027,0.045,0.065};
  double intensity_sum = 0;
  double distance_intensity_sum = 0;
  int i;
  for (i = 0; i < 8; i++)
  {
    distance_intensity_sum = distance_intensity_sum + distance[i]*sensor_values[i];
    intensity_sum = intensity_sum+sensor_values[i]; //denominator
  }
  return distance_intensity_sum/intensity_sum;
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

bool sensorstop(int sensor, double condition, int mode)
{
  double sensor_value;

  sensor_value = readsensor(sensor);

  // Now check if condition is true given the values
  if (mode == 0) return sensor_value <= condition ? true : false; // If sensor value is less than condition, the robot should stop!
  else if (mode == 1) return sensor_value >= condition? true : false; // If sensor value is more than condition, the robot should stop!
}

double readsensor(int sensor)
{
  // *sensor is a string determining which sensor to check, l0 for the first lasersensor and r0 for the first infrared sensor 
  // if (sensor == 'N') return false;
  double sensor_value;
  if (sensor < 10)
  {
    // Get value from sensor
    sensor_value = laserpar[sensor];
  }
  else if (sensor >= 10)
  {
    // Get value from sensor
    sensor_value = irsensor->data[sensor-10];
    sensor_value = (KA)/(sensor_value - KB);
  }
  return sensor_value;
}
