/*
 * An example SMR program.
 *
 */
#include <stdio.h>
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

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"


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
int index_for_logging = 0;

float logged_data[100000][3];
float logged_odo[100000][4];

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

// HOMEMADE HEADERS:
double *calibrate_line(symTableElement *linesensor_values);
int find_line_min(double *sensor_values, int orientation);
double line_COM(double *sensor_values);

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


} motiontype;

enum
{
  mot_stop = 1,
  mot_move,
  mot_turn,
  mot_follow
};

void update_motcon(motiontype *p, odotype *o);

int fwd(double dist, double speed, int time);
int turn(double angle, double speed, int time);

// New:
int follow(double dist, double speed, int time);

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

typedef struct
{
  int state, oldstate;
  int time;
} smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum
{
  ms_init,
  ms_fwd,
  ms_turn,
  ms_followline,
  ms_end
};

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
  mission.state = ms_init; // Change between ms_fwd for square or straightline program, or ms_followline for
  mission.oldstate = -1;
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
      n = 1;
      dist = 3;
      angle = 0.0 / 180 * M_PI;
      speed = 0.2;
      mission.state = ms_followline; // Change between ms_fwd for square or straightline program, or ms_followline for followline program.
      break;

    case ms_fwd:
      // printf("Going Forward \n");
      if (fwd(dist, speed, mission.time))
        mission.state = ms_turn;

      break;

    case ms_turn:
      // printf("Turning \n");
      if (turn(angle, speed, mission.time))
      {
        n = n - 1;
        if (n == 0)
          mission.state = ms_end;
        else
          mission.state = ms_fwd;
      }
      break;

    case ms_followline: // This is new
      //printf("We follow a line!\n");
      if (follow(dist, speed, mission.time))
        mission.state = ms_end;
      break;


    case ms_end:
      printf("Ending mission\n");
      mot.cmd = mot_stop;
      running = 0;
      break;
    }
    /*  end of mission  */

    logged_data[index_for_logging][0] = mission.time;
    logged_data[index_for_logging][1] = mot.motorspeed_l;
    logged_data[index_for_logging][2] = mot.motorspeed_r;

    logged_odo[index_for_logging][0] = mission.time;
    logged_odo[index_for_logging][1] = odo.x;
    logged_odo[index_for_logging][2] = odo.y;
    logged_odo[index_for_logging][3] = odo.theta;

    index_for_logging++;

    mot.left_pos = odo.left_pos;
    mot.right_pos = odo.right_pos;
    update_motcon(&mot, &odo);
    speedl->data[0] = 100 * mot.motorspeed_l;
    speedl->updated = 1;
    speedr->data[0] = 100 * mot.motorspeed_r;
    speedr->updated = 1;
    if (time % 100 == 0)
      //    printf(" laser %f \n",laserpar[3]);
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

  printf("Logging data");
  int index = 0;
  FILE *fp;
  FILE *odo_p;
  fp = fopen("logged_data.dat", "w");
  odo_p = fopen("logged_odo.dat", "w");
  while (index < index_for_logging)
  {
    fprintf(fp, "%f %f %f\n", logged_data[index][0], logged_data[index][1], logged_data[index][2]);
    fprintf(odo_p, "%f %f %f %f\n", logged_odo[index][0], logged_odo[index][1], logged_odo[index][2], logged_odo[index][3]);
    index++;
  }
  fclose(fp);
  fclose(odo_p);
  printf("Done Executing Program! \n");

  exit(0);
}

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

    case mot_follow: //This is new
      p->startpos = (p->left_pos + p->right_pos) / 2;
      p->curcmd = mot_follow;
      break;
    }

    p->cmd = 0;
  }

  double remaining_dist;
  double v_max;
  double v_delta;
  double angular_distance;
  
  // Custom values for follow_line sensor functionality
  double *calibrated_values;
  int sensor_index;
  int i;
  double com;

  //printf("Linesensor value: %d", linesensor);
  //printf("Linesensor no of indexes: %d %d %d %d %d %d %d %d \n", linesensor[0].name, linesensor[1], linesensor[2], linesensor[3], linesensor[4], linesensor[5], linesensor[6], linesensor[7]);

  switch (p->curcmd)
  {

  case mot_stop:
    p->motorspeed_l = 0;
    p->motorspeed_r = 0;
    break;
  case mot_move:
    remaining_dist = p->dist - ((p->right_pos + p->left_pos) / 2 - p->startpos);
    v_max = sqrt(2 * 0.5 * fabs(remaining_dist));
    // printf("%f\n", v_max);
    //  Have we reached
    if ((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist)
    {
      p->finished = 1;
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
    }
    if (v_max < p->speedcmd)
    {
      p->motorspeed_l = v_max;
      p->motorspeed_r = v_max;
    }
    else
    { // Else we should move:
      if (((p->speedcmd - p->motorspeed_l) < 0.5 / 100) | ((p->speedcmd - p->motorspeed_r) < 0.5 / 100))
      {
        p->motorspeed_l = p->speedcmd;
        p->motorspeed_r = p->speedcmd;
      }
      else
      {
        p->motorspeed_l += 0.5 / 100;
        p->motorspeed_r += 0.5 / 100;
      }
    }
    break;

  case mot_turn:
    angular_distance = p->theta_ref - o->theta;
    if (angular_distance > 0 && angular_distance <= M_PI)
    { // If we have to turn left (the angle is less than 180 degrees and postiive)
      if (angular_distance > 0.5 / 180 * M_PI)
      { // Condition suff small keep turning
        p->motorspeed_r = p->motorspeed_r + 0.001 * angular_distance;
        p->motorspeed_l = p->motorspeed_l - 0.001 * angular_distance;
      }
      else
      {
        p->motorspeed_r = 0;
        p->motorspeed_l = 0;
        p->finished = 1;
      }
    }
    else
    { // Else we turn right

      if (angular_distance < -0.5 / 180 * M_PI)
      {
        p->motorspeed_l = p->motorspeed_l - 0.001 * angular_distance;
        p->motorspeed_r = p->motorspeed_r + 0.001 * angular_distance;
      }
      else
      {
        p->motorspeed_l = 0;
        p->motorspeed_r = 0;
        p->finished = 1;
      }
    }

    break;

    case mot_follow:
      calibrated_values = calibrate_line(linesensor);
      sensor_index = find_line_min(calibrated_values, 0); //0, keeps left, 1 keeps right.
      com = line_COM(calibrated_values);
      remaining_dist = p->dist - ((p->right_pos + p->left_pos) / 2 - p->startpos);
      v_max = p->speedcmd;
      //v_delta = 0.01 * (3-sensor_index); // This version works with sensor index
      v_delta = 1*(-0.001286-com); // This version works with com
      //  Have we reached
      if (((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist) | sensor_index == -1)
      {
        p->finished = 1;
        p->motorspeed_l = 0;
        p->motorspeed_r = 0;
      }
      if (v_max < fabs(p->motorspeed_l))
      {
        p->motorspeed_l = v_max;
      }
      if (v_max < fabs(p->motorspeed_r))
      {
        p->motorspeed_r = v_max;
      }
      printf("delta V: %f\n", v_delta);
      //if (v_delta == 0) { //Use this for sensor
      if (v_delta <= 0.001 && v_delta >= -0.001) { //If we are straight on the line, then full speed ahead! (use this for com)
        p->motorspeed_l = v_max;
        p->motorspeed_r = v_max;
      }
      else if (v_delta < 0){ // Large sensor index, turn right
        p->motorspeed_r += v_delta;
      }
      else if (v_delta > 0){ // Small sensor index, turn left
        p->motorspeed_l -= v_delta;
      }
  }
}

int fwd(double dist, double speed, int time)
{
  if (time == 0)
  {
    mot.cmd = mot_move;
    mot.speedcmd = speed;
    mot.dist = dist;
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

int follow(double dist, double speed, int time){
  if (time == 0)
  {
    mot.cmd = mot_follow;
    mot.speedcmd = speed;
    mot.dist = dist;
    return 0;
  }
  else
    return mot.finished;
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

// Homemade functions:

double *calibrate_line(symTableElement *linesensor_values)
{
  // This function converts uncalibrated linesensor values to calibrated
  // using the linear transformation described in the calibration exercise.
  static double r[8];
  int i;

  for (i = 0; i < 8; i++){
    r[i] = (linesensor_values->data[i]) / 255.0;
  }
  return r;
}

int find_line_min(double *sensor_values, int orientation)
{
  // This function finds the position of the black line using the position of the
  // lowest calibrated linesensor.
  double curr_min = 1.0;
  int index;
  int i;
  int all_same = 1;
  for (i=7; i --> 0;){ //reverse loop
    if (orientation == 0) {
      if (sensor_values[i] < curr_min) {
        curr_min = sensor_values[i];
        index = 7-i;
      }
    }
    else if (orientation == 1){
      if (sensor_values[i] <= curr_min) {
        curr_min = sensor_values[i];
        index = 7-i;
      }
    }
    if (i<7 && (sensor_values[i+1] == sensor_values[i])){
      all_same *=1;
    }
    else if (i<7 && (sensor_values[i+1] != sensor_values[i])) {
      all_same *= 0;
    }
  }
  if (all_same == 1) return -1;
  else return index;
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