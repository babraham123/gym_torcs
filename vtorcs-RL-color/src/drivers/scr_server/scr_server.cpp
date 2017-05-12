/***************************************************************************

    file                 : scr_server.cpp
    copyright            : (C) 2007 Daniele Loiacono

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/


#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <ctime>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include "sensors.h"
#include "SimpleParser.h"
#include "CarControl.h"
#include "ObstacleSensors.h"


#ifdef _WIN32
typedef sockaddr_in tSockAddrIn;
typedef int socklen_t;
#define CLOSE(x) closesocket(x)
#define INVALID(x) x == INVALID_SOCKET
#else
typedef int SOCKET;
typedef struct sockaddr_in tSockAddrIn;
#define CLOSE(x) close(x)
#define INVALID(x) x < 0
#endif

#ifndef _WIN32
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>
#endif

/*** defines for UDP *****/
// GIUSE - port is now provided from command line (e.g. torcs -p 3001)
//#define UDP_LISTEN_PORT 3001
#define UDP_ID "SCR-VIS"
// GIUSE - for evolution we load the server over the limits - we need higher timeouts!
#define UDP_DEFAULT_TIMEOUT 100000
//#define UDP_DEFAULT_TIMEOUT 100000
// GIUSE - size has to be increased to accomodate larger images
static int UDP_MSGLEN = 128*128+1000;

//#define UDP_MSGLEN 650000
//#define __UDP_SERVER_VERBOSE__
/************************/

static int UDP_TIMEOUT = UDP_DEFAULT_TIMEOUT;

#define NBBOTS 10

#define RACE_RESTART 1
//#define __STEP_LIMIT__ 10000
//#define __DISABLE_RESTART__
//#define __PRINT_RACE_RESULTS__

double __SENSORS_RANGE__;
#define __FOCUS_RANGE__ 200

/*** Noise definitions ***/
#define __NOISE_STD__ 0.1
#define __OPP_NOISE_STD__ 0.02
#define __FOCUS_NOISE_STD__ 0.01

#ifdef __PRINT_RACE_RESULTS__
static tdble bestLap[NBBOTS];
static tdble damages[NBBOTS];
static tdble totalTime[NBBOTS];
static int position[NBBOTS];
static int curPosition=0;
static int bonusBest;
static int bonusDamage;
static char *trackName;
#endif

static tTrack   *curTrack;
static int RESTARTING[NBBOTS];

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newrace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt);

static double normRand(double avg,double std);

/**** variables for UDP ***/
static int listenSocket[NBBOTS];
socklen_t clientAddressLength[NBBOTS];
tSockAddrIn clientAddress[NBBOTS], serverAddress[NBBOTS];
/************************************************/

static tdble oldAccel[NBBOTS];
static tdble oldBrake[NBBOTS];
static tdble oldSteer[NBBOTS];
static tdble oldClutch[NBBOTS];
static tdble prevDist[NBBOTS];
static tdble distRaced[NBBOTS];

static int oldFocus[NBBOTS];//ML
static int oldGear[NBBOTS];

static Sensors *trackSens[NBBOTS];
static ObstacleSensors *oppSens[NBBOTS];
static Sensors *focusSens[NBBOTS];//ML
static float trackSensAngle[NBBOTS][19];

static char* botname[NBBOTS] = {"scr_server 1", "scr_server 2", "scr_server 3", "scr_server 4", "scr_server 5", "scr_server 6", "scr_server 7", "scr_server 8", "scr_server 9", "scr_server 10"};

static unsigned long total_tics[NBBOTS];

/*
 * Module entry point
 */
extern "C" int
    scr_server(tModInfo *modInfo)
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    for (int i = 0; i < NBBOTS; i++) {
        modInfo[i].name    = botname[i];  // name of the module (short).
        modInfo[i].desc    = "";    // Description of the module (can be long).
        modInfo[i].fctInit = InitFuncPt;// Init function.
        modInfo[i].gfId    = ROB_IDENT; // Supported framework version.
        modInfo[i].index   = i;     // Indices from 0 to 9.
    }
    return 0;
}

/* Module interface initialization. */
static int
InitFuncPt(int index, void *pt)
{
    tRobotItf *itf  = (tRobotItf *)pt;

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */
    /* for every track change or new race */
    itf->rbNewRace  = newrace;   /* Start a new race */
    itf->rbDrive    = drive;     /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;   /* End of the current race */
    itf->rbShutdown = shutdown;  /* Called before the module is unloaded */
    itf->index      = index;     /* Index used if multiple interfaces */

    #ifdef _WIN32
     /* WinSock Startup */

     WSADATA wsaData={0};
     WORD wVer = MAKEWORD(2,2);
     int nRet = WSAStartup(wVer,&wsaData);

     if(nRet == SOCKET_ERROR)
     {
    std::cout << "Failed to init WinSock library" << std::endl;
    exit(1);
     }
#endif

    return 0;
}

/* Called for every track change or new race. */
static void
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s)
{
    curTrack = track;
    *carParmHandle = NULL;
#ifdef _PRINT_RACE_RESULTS__
    trackName = strrchr(track->filename, '/') + 1;
#endif
}

/* Start a new race. */
static void
newrace(int index, tCarElt* car, tSituation *s)
{

    total_tics[index]=0;

    /***********************************************************************************
    ************************* UDP client identification ********************************
    ***********************************************************************************/

    bool identified=false;
    char line[UDP_MSGLEN];

    // Set timeout
    if (getTimeout()>0)
        UDP_TIMEOUT = getTimeout();

    //Set sensor range
    if (strcmp(getVersion(),"2009")==0)
    {
        __SENSORS_RANGE__ = 100;
        printf("*****2009*****\n");
    }
    else if (strcmp(getVersion(),"2010")==0 || strcmp(getVersion(),"2011")==0)
        __SENSORS_RANGE__ = 200;
    else
    {
        printf("%s is not a recognized version",getVersion());
        exit(0);
    }

    listenSocket[index] = socket(AF_INET, SOCK_DGRAM, 0);
    if (listenSocket[index] < 0)
    {
        std::cerr << "Error: cannot create listenSocket!";
        exit(1);
    }

    srand(time(NULL));

    // Bind listen socket to listen port.
    serverAddress[index].sin_family = AF_INET;
    serverAddress[index].sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddress[index].sin_port = htons(getUDPListenPort()+index);

    if (bind(listenSocket[index],
             (struct sockaddr *) &serverAddress[index],
             sizeof(serverAddress[index])) < 0)
    {
        std::cerr << "cannot bind socket";
        exit(1);
    }

    // Wait for connections from clients.
    listen(listenSocket[index], 5);

    std::cout << "Waiting for request on port " << getUDPListenPort()+index << "\n";

    // Loop until a client identifies correctly
    while (!identified)
    {

        clientAddressLength[index] = sizeof(clientAddress[index]);

        // Set line to all zeroes
        memset(line, 0x0, UDP_MSGLEN);
        if (recvfrom(listenSocket[index], line, UDP_MSGLEN, 0,
                     (struct sockaddr *) &clientAddress[index],
                     &clientAddressLength[index]) < 0)
        {
            std::cerr << "Error: problem in receiving from the listen socket";
            exit(1);
        }

#ifdef __UDP_SERVER_VERBOSE__
        // show the client's IP address
        std::cout << "  from " << inet_ntoa(clientAddress[index].sin_addr);

        // show the client's port number.
        std::cout << ":" << ntohs(clientAddress[index].sin_port) << "\n";

        // Show the line
        std::cout << "  Received: " << line << "\n";
#endif

        // compare received string with the ID
        if (strncmp(line,UDP_ID,3)==0)
        {
#ifdef __UDP_SERVER_VERBOSE__
            std::cout << "IDENTIFIED" << std::endl;
#endif
            std::string initStr(line);
            if (SimpleParser::parse(initStr,std::string("init"),trackSensAngle[index],19)==false)
            {
                for (int i = 0; i < 19; ++i) {
                    trackSensAngle[index][i] =-90 + 10*i;
                }
            }
    //         char line[UDP_MSGLEN];
            sprintf(line,"***identified***");
            // Sending the car state to the client
            if (sendto(listenSocket[index], line, strlen(line) + 1, 0,
                       (struct sockaddr *) &clientAddress[index],
                       sizeof(clientAddress[index])) < 0)
                std::cerr << "Error: cannot send identification message";
            identified=true;
        }
    }

    focusSens[index] = new Sensors(car, 5);//ML
    for (int i = 0; i < 5; ++i) {//ML
        focusSens[index]->setSensor(i,(car->_focusCmd)+i-2,200);//ML
    }//ML

    // Initialization of track sensors
    trackSens[index] = new Sensors(car, 19);
    for (int i = 0; i < 19; ++i) {
        trackSens[index]->setSensor(i,trackSensAngle[index][i],__SENSORS_RANGE__);
#ifdef __UDP_SERVER_VERBOSE__
        std::cout << "Set Track Sensors " << i+1 << " at angle " << trackSensAngle[index][i] << std::endl;
#endif
    }
    // Initialization of opponents sensors
    oppSens[index] = new ObstacleSensors(36, curTrack, car, s, __SENSORS_RANGE__);

    prevDist[index]=-1;
}


/* check if the car is stuck */
const float MAX_UNSTUCK_SPEED = 5.0;   /* [m/s] */
const float MIN_UNSTUCK_DIST = 3.0;    /* [m] */
const float MAX_UNSTUCK_ANGLE = 20.0/180.0*PI;
const int MAX_UNSTUCK_COUNT = 250;
static int stuck = 0;

bool isStuck(tCarElt* car)
{
    float angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle);

    if (fabs(angle) > MAX_UNSTUCK_ANGLE &&
        car->_speed_x < MAX_UNSTUCK_SPEED &&
        fabs(car->_trkPos.toMiddle) > MIN_UNSTUCK_DIST) {
        if (stuck > MAX_UNSTUCK_COUNT && car->_trkPos.toMiddle*angle < 0.0) {
            return true;
        } else {
            stuck++;
            return false;
        }
    } else {
        stuck = 0;
        return false;
    }
}

/* Compute gear. */
const float SHIFT = 0.85;         /* [-] (% of rpmredline) */
const float SHIFT_MARGIN = 4.0;  /* [m/s] */

int getGear(tCarElt *car)
{
    if (car->_gear <= 0)
        return 1;
    float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
    float omega = car->_enginerpmRedLine/gr_up;
    float wr = car->_wheelRadius(2);

    if (omega*wr*SHIFT < car->_speed_x) {
        return car->_gear + 1;
    } else {
        float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
        omega = car->_enginerpmRedLine/gr_down;
        if (car->_gear > 1 && omega*wr*SHIFT > car->_speed_x + SHIFT_MARGIN) {
            return car->_gear - 1;
        }
    }
    return car->_gear;
}

// Compute the length to the start of the segment.
float getDistToSegStart(tCarElt *ocar)
{
    if (ocar->_trkPos.seg->type == TR_STR) {
        return ocar->_trkPos.toStart;
    } else {
        return ocar->_trkPos.toStart*ocar->_trkPos.seg->radius;
    }
}


/* Drive during race. */
static void
drive(int index, tCarElt* car, tSituation *s)
{

    total_tics[index]++;
    char line[UDP_MSGLEN];

#ifdef __PRINT_RACE_RESULTS__
    bestLap[index]=car->_bestLapTime;
    damages[index]=car->_dammage;
    totalTime[index]=car->_timeBehindLeader;
#endif


#ifdef __DISABLE_RESTART__
    if (RESTARTING[index]==1)
    {

        clientAddressLength[index] = sizeof(clientAddress[index]);

        // Set line to all zeroes
        memset(line, 0x0, 101);
        if (recvfrom(listenSocket[index], line, 100, 0,
                     (struct sockaddr *) &clientAddress[index],
                     &clientAddressLength[index]) < 0)
        {
            std::cerr << "Error: problem in receiving from the listen socket";
            exit(1);
        }

  #ifdef __UDP_SERVER_VERBOSE__
        // show the client's IP address
        std::cout << "  from " << inet_ntoa(clientAddress[index].sin_addr);

        // show the client's port number.
        std::cout << ":" << ntohs(clientAddress[index].sin_port) << "\n";

        // Show the line
        std::cout << "  Received: " << line << "\n";
  #endif

        // compare received string with the ID
        if (strncmp(line,UDP_ID,3)==0)
        {
  #ifdef __UDP_SERVER_VERBOSE__
            std::cout << "IDENTIFIED" << std::endl;
  #endif
    //         char line[UDP_MSGLEN];
            sprintf(line,"***identified***");
            // Sending the car state to the client
            if (sendto(listenSocket[index], line, strlen(line) + 1, 0,
                       (struct sockaddr *) &clientAddress[index],
                       sizeof(clientAddress[index])) < 0)
                std::cerr << "Error: cannot send identification message";
        RESTARTING[index]=0;
        }
    }
#endif

    // local variables for UDP
    struct timeval timeVal;
    fd_set readSet;

    // computing distance to middle
    float dist_to_middle = 2*car->_trkPos.toMiddle/(car->_trkPos.seg->width);
    // computing the car angle wrt the track axis
    float angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // normalize the angle between -PI and + PI

    //Update focus sensors' angle
    for (int i = 0; i < 5; ++i) {
        focusSens[index]->setSensor(i,(car->_focusCmd)+i-2,200);
    }

    // update the value of track sensors only as long as the car is inside the track
    float trackSensorOut[19];
    float focusSensorOut[5];//ML
    if (dist_to_middle<=1.0 && dist_to_middle >=-1.0 )
    {
        trackSens[index]->sensors_update();
        for (int i = 0; i < 19; ++i)
        {
            trackSensorOut[i] = trackSens[index]->getSensorOut(i);
            if (getNoisy())
                trackSensorOut[i] *= normRand(1,__NOISE_STD__);
        }
        focusSens[index]->sensors_update();//ML
        if ((car->_focusCD <= car->_curLapTime + car->_curTime)//ML Only send focus sensor reading if cooldown is over
            && (car->_focusCmd != 360))//ML Only send focus reading if requested by client
        {//ML
            for (int i = 0; i < 5; ++i)
            {
                focusSensorOut[i] = focusSens[index]->getSensorOut(i);
                if (getNoisy())
                    focusSensorOut[i] *= normRand(1,__FOCUS_NOISE_STD__);
            }
            car->_focusCD = car->_curLapTime + car->_curTime + 1.0;//ML Add cooldown [seconds]
        }//ML
        else//ML
        {//ML
            for (int i = 0; i < 5; ++i)//ML
                focusSensorOut[i] = -1;//ML During cooldown send invalid focus reading
        }//ML
    }
    else
    {
        for (int i = 0; i < 19; ++i)
        {
            trackSensorOut[i] = -1;
        }
        for (int i = 0; i < 5; ++i)
        {
            focusSensorOut[i] = -1;
        }
    }

    // update the value of opponent sensors
    float oppSensorOut[36];
    oppSens[index]->sensors_update(s);
    for (int i = 0; i < 36; ++i)
    {
        oppSensorOut[i] = oppSens[index]->getObstacleSensorOut(i);
        if (getNoisy())
            oppSensorOut[i] *= normRand(1,__OPP_NOISE_STD__);
    }

    float wheelSpinVel[4];
    for (int i=0; i<4; ++i)
    {
        wheelSpinVel[i] = car->_wheelSpinVel(i);
    }

    if (prevDist[index]<0)
    {
        prevDist[index] = car->race.distFromStartLine;
    }
    float curDistRaced = car->race.distFromStartLine - prevDist[index];
    prevDist[index] = car->race.distFromStartLine;
    if (curDistRaced>100)
    {
        curDistRaced -= curTrack->length;
    }
    if (curDistRaced<-100)
    {
        curDistRaced += curTrack->length;
    }

    distRaced[index] += curDistRaced;

    float totdist = curTrack->length * (car->race.laps -1) + car->race.distFromStartLine;

    // std::cerr << "totraced: " << totdist << std::endl;


    //////////////////////////// from DeepDriving chenyi code
    // if (isStuck(car)) {
    //     float angle = -RtTrackSideTgAngleL(&(car->_trkPos)) + car->_yaw;
    //     NORM_PI_PI(angle); // put the angle back in the range from -PI to PI

    //     car->ctrl.steer = angle / car->_steerLock;
    //     car->ctrl.gear = -1; // reverse gear
    //     car->ctrl.accelCmd = 0.9; // 90% accelerator pedal
    //     car->ctrl.brakeCmd = 0.0; // no brakes
    // } 
    // else {

    // drive on the road with 3 lanes

    car->ctrl.gear = getGear(car);

    car->ctrl.steer = *psteerCmd;
    car->ctrl.accelCmd = *paccelCmd;
    car->ctrl.brakeCmd = *pbrakeCmd;

    const float lane_width=4.0;
    const float half_lane_width=lane_width/2;
    const float max_dist_range=60.0;
    const float deactivated_dist_range=75.0;

    const float max_toMarking_LR=7.0;
    const float max_toMarking_M=3.5;
    const float max_toMarking_LLRR=9.5;
    const float max_toMarking_MM=5.5;

    float min_dist[3]={99999,99999,99999};
    float distance;
    tCarElt* min_car[3]={0,0,0};

    float fast=0.0;

    float dist_L=deactivated_dist_range;
    float dist_R=deactivated_dist_range;

    float toMarking_L=-max_toMarking_LR;
    float toMarking_M=max_toMarking_M;
    float toMarking_R=max_toMarking_LR;

    float dist_LL=deactivated_dist_range;
    float dist_MM=deactivated_dist_range;
    float dist_RR=deactivated_dist_range;

    float toMarking_LL=-max_toMarking_LLRR;
    float toMarking_ML=-max_toMarking_MM;
    float toMarking_MR=max_toMarking_MM;
    float toMarking_RR=max_toMarking_LLRR;

    for (int i = 0; i < s->_ncars; i++) {
        if (s->cars[i] != car) {
            distance = s->cars[i]->_trkPos.seg->lgfromstart + getDistToSegStart(s->cars[i]) - car->_distFromStartLine;
            if (distance > curTrack->length/2.0f) {
                distance -= curTrack->length;
            } else if (distance < -curTrack->length/2.0f) {
                distance += curTrack->length;
            }

            if (distance>0) {
                if (s->cars[i]->_trkPos.toMiddle>half_lane_width && distance<min_dist[0]) {  // on left lane
                    min_dist[0]=distance;
                    min_car[0]=s->cars[i];
                } else if (s->cars[i]->_trkPos.toMiddle<-half_lane_width && distance<min_dist[2]) {  // on right lane
                    min_dist[2]=distance;
                    min_car[2]=s->cars[i];
                } else if (s->cars[i]->_trkPos.toMiddle>=-half_lane_width && s->cars[i]->_trkPos.toMiddle<=half_lane_width && distance<min_dist[1]) {  // on central lane
                    min_dist[1]=distance;
                    min_car[1]=s->cars[i];
                }
            }
        }
    }

    if (min_dist[0]>max_dist_range) min_dist[0]=max_dist_range;
    if (min_dist[1]>max_dist_range) min_dist[1]=max_dist_range;
    if (min_dist[2]>max_dist_range) min_dist[2]=max_dist_range;

    if (car->_trkPos.toMiddle<=-(half_lane_width+0.5) && car->_trkPos.toMiddle>=-(lane_width+half_lane_width-0.5)) { // on the right lane
        toMarking_LL=car->_trkPos.toMiddle-half_lane_width;
        toMarking_ML=car->_trkPos.toMiddle+half_lane_width;
        toMarking_MR=car->_trkPos.toMiddle+lane_width+half_lane_width;
        toMarking_RR=max_toMarking_LLRR;
        
        if (min_car[1]!=0) dist_LL=min_dist[1];
        else dist_LL=max_dist_range;
        
        if (min_car[2]!=0) dist_MM=min_dist[2];
        else dist_MM=max_dist_range;

        // dist_RR=deactivated_dist_range;
    } else if (car->_trkPos.toMiddle<=(lane_width+half_lane_width-0.5) && car->_trkPos.toMiddle>=(half_lane_width+0.5)) { // on the left lane
        toMarking_LL=-max_toMarking_LLRR;
        toMarking_ML=car->_trkPos.toMiddle-lane_width-half_lane_width;
        toMarking_MR=car->_trkPos.toMiddle-half_lane_width;
        toMarking_RR=car->_trkPos.toMiddle+half_lane_width;
        // dist_LL=deactivated_dist_range;
        
        if (min_car[0]!=0) dist_MM=min_dist[0];
        else dist_MM=max_dist_range;
        
        if (min_car[1]!=0) dist_RR=min_dist[1]; 
        else dist_RR=max_dist_range;

    } else if (car->_trkPos.toMiddle<=(half_lane_width-0.5) && car->_trkPos.toMiddle>=-(half_lane_width-0.5)) { // on the central lane
        toMarking_LL=car->_trkPos.toMiddle-lane_width-half_lane_width;
        toMarking_ML=car->_trkPos.toMiddle-half_lane_width;
        toMarking_MR=car->_trkPos.toMiddle+half_lane_width;
        toMarking_RR=car->_trkPos.toMiddle+lane_width+half_lane_width;
        
        if (min_car[0]!=0) dist_LL=min_dist[0];
        else dist_LL=max_dist_range;
        
        if (min_car[1]!=0) dist_MM=min_dist[1]; 
        else dist_MM=max_dist_range;
        
        if (min_car[2]!=0) dist_RR=min_dist[2];   
        else dist_RR=max_dist_range;        
    }

    if (car->_trkPos.toMiddle<=(half_lane_width+1) && car->_trkPos.toMiddle>=(half_lane_width-1)) { // on the center L marking
        toMarking_L=car->_trkPos.toMiddle-lane_width-half_lane_width;
        toMarking_M=car->_trkPos.toMiddle-half_lane_width;
        toMarking_R=car->_trkPos.toMiddle+half_lane_width;
        
        if (min_car[0]!=0) dist_L=min_dist[0];
        else dist_L=max_dist_range;
        
        if (min_car[1]!=0) dist_R=min_dist[1];
        else dist_R=max_dist_range;

    } else if (car->_trkPos.toMiddle<=-(half_lane_width-1) && car->_trkPos.toMiddle>=-(half_lane_width+1)) { // on the center R marking
        toMarking_L=car->_trkPos.toMiddle-half_lane_width;
        toMarking_M=car->_trkPos.toMiddle+half_lane_width;
        toMarking_R=car->_trkPos.toMiddle+lane_width+half_lane_width;
        
        if (min_car[1]!=0) dist_L=min_dist[1];
        else dist_L=max_dist_range;
        
        if (min_car[2]!=0) dist_R=min_dist[2];
        else dist_R=max_dist_range;

    } else if (car->_trkPos.toMiddle<=-(lane_width+half_lane_width-1) && car->_trkPos.toMiddle>=-(lane_width+half_lane_width+2)) { // on the right marking
        toMarking_L=car->_trkPos.toMiddle+half_lane_width;
        toMarking_M=car->_trkPos.toMiddle+lane_width+half_lane_width;
        toMarking_R=max_toMarking_LR;
        
        if (min_car[2]!=0) dist_L=min_dist[2];
        else dist_L=max_dist_range;

        //dist_R=deactivated_dist_range;
    } else if (car->_trkPos.toMiddle<=(lane_width+half_lane_width+2) && car->_trkPos.toMiddle>=(lane_width+half_lane_width-1)) { // on the left marking
        toMarking_L=-max_toMarking_LR;
        toMarking_M=car->_trkPos.toMiddle-lane_width-half_lane_width;
        toMarking_R=car->_trkPos.toMiddle-half_lane_width;
        //dist_L=deactivated_dist_range;
        
        if (min_car[0]!=0) dist_R=min_dist[0];
        else dist_R=max_dist_range;
    }

    if (car->_trkPos.seg->type == TR_STR) {
        float str_length=car->_trkPos.seg->length - car->_trkPos.toStart;
        tTrackSeg *segptr = car->_trkPos.seg->next;

        while (segptr->type == TR_STR) {
            str_length=str_length+segptr->length;
            segptr = segptr->next;
        }

        if (str_length>60) fast=1.0;
    }

    float c_angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(c_angle);

    /**********************************************************************
     ****************** Building state string *****************************
     **********************************************************************/

    string stateString;

    stateString =  SimpleParser::stringify("angle", angle);
    stateString += SimpleParser::stringify("curLapTime", float(car->_curLapTime));
    stateString += SimpleParser::stringify("damage",        ( getDamageLimit() ? car->_dammage : car->_fakeDammage ) );
    stateString += SimpleParser::stringify("distFromStart", car->race.distFromStartLine);
    stateString += SimpleParser::stringify("totalDistFromStart", totdist);
    stateString += SimpleParser::stringify("distRaced", distRaced[index]);
    stateString += SimpleParser::stringify("fuel", car->_fuel);
    stateString += SimpleParser::stringify("gear", car->_gear);
    stateString += SimpleParser::stringify("lastLapTime", float(car->_lastLapTime));
    stateString += SimpleParser::stringify("opponents", oppSensorOut, 36);
    stateString += SimpleParser::stringify("racePos", car->race.pos);
    stateString += SimpleParser::stringify("rpm", car->_enginerpm*10);
    stateString += SimpleParser::stringify("speedX", float(car->_speed_x  * 3.6));
    stateString += SimpleParser::stringify("speedY", float(car->_speed_y  * 3.6));
    stateString += SimpleParser::stringify("speedZ", float(car->_speed_z  * 3.6));
    stateString += SimpleParser::stringify("track", trackSensorOut, 19);
    stateString += SimpleParser::stringify("trackPos", dist_to_middle);
    stateString += SimpleParser::stringify("wheelSpinVel", wheelSpinVel, 4);
    stateString += SimpleParser::stringify("z", car->_pos_Z  - RtTrackHeightL(&(car->_trkPos)));
    stateString += SimpleParser::stringify("focus", focusSensorOut, 5);//ML

    // pass affordances to client
    stateString +=  SimpleParser::stringify("c_angle", c_angle);
    stateString +=  SimpleParser::stringify("toMarking_L", toMarking_L);
    stateString +=  SimpleParser::stringify("toMarking_M", toMarking_M);
    stateString +=  SimpleParser::stringify("toMarking_R", toMarking_R);
    stateString +=  SimpleParser::stringify("dist_L", dist_L);
    stateString +=  SimpleParser::stringify("dist_R", dist_R);
    stateString +=  SimpleParser::stringify("toMarking_LL", toMarking_LL);
    stateString +=  SimpleParser::stringify("toMarking_ML", toMarking_ML);
    stateString +=  SimpleParser::stringify("toMarking_MR", toMarking_MR);
    stateString +=  SimpleParser::stringify("toMarking_RR", toMarking_RR);
    stateString +=  SimpleParser::stringify("dist_LL", dist_LL);
    stateString +=  SimpleParser::stringify("dist_MM", dist_MM);
    stateString +=  SimpleParser::stringify("dist_RR", dist_RR);
    stateString +=  SimpleParser::stringify("fast", fast);

    //GIUSE - VISION HERE!
    // printf("size: %d\n",car->vision->imgsize);

    if( getVision() ){
    //   std::cout << car->vision->imgsize << std::endl;
        // 128 x 128 grayscale
        stateString += SimpleParser::stringify("img", car->vision->img, car->vision->imgsize);
    }

    // for(int i=0; i < car->vision->imgsize; i++){
    //   std::cout << (int)car->vision->img[i] << " ";
    // }

  // GIUSE - that's UGLY, can we stay coherent with either char* or string??
    // char line[UDP_MSGLEN];
    // memset(line, 0x0,UDP_MSGLEN ); //
    // sprintf(line,"%s",stateString.c_str());

    if (RESTARTING[index]==0)
    {
#ifdef __UDP_SERVER_VERBOSE__

        std::cout << "Sending: " << stateString.c_str() << std::endl;
        std::cout << "Sending: " << stateString.c_str() << std::endl;

#endif

#ifdef __STEP_LIMIT__

        if (total_tics[index]>__STEP_LIMIT__)
        {
        RESTARTING[index] = 1;
        car->RESTART=1;

        char fileName[200];
        sprintf(fileName,"%s.txt",trackName);
        printf("%s.txt\n",trackName);
        FILE *f = fopen (fileName,"a");

        printf("Dist_raced %lf\n",distRaced[index]);
        fprintf(f,"Dist_raced %lf\n",distRaced[index]);

        fclose(f);
        return;
        }
#endif

        // Sending the car state to the client
        // if (sendto(listenSocket[index], line, strlen(line) + 1, 0,
        if (sendto(listenSocket[index], stateString.c_str(), stateString.length() + 1, 0,
                   (struct sockaddr *) &clientAddress[index],
                   sizeof(clientAddress[index])) < 0)
            std::cerr << "Error: cannot send car state";


        // Set timeout for client answer
        FD_ZERO(&readSet);
        FD_SET(listenSocket[index], &readSet);
        timeVal.tv_sec = 0;
        timeVal.tv_usec = UDP_TIMEOUT;
        memset(line, 0x0,UDP_MSGLEN ); // GIUSE - BUG, THERE WAS A 1000 HARDCODED

        if (select(listenSocket[index]+1, &readSet, NULL, NULL, &timeVal))
        {
            // Read the client controller action
            // memset(line, 0x0,UDP_MSGLEN );  // Zero out the buffer. GIUSE - already done
            int numRead = recv(listenSocket[index], line, UDP_MSGLEN, 0);
            if (numRead < 0)
            {
                std::cerr << "Error, cannot get any response from the client!";
                CLOSE(listenSocket[index]);
                exit(1);
            }

#ifdef __UDP_SERVER_VERBOSE__
            std::cout << "Received: " << line << std::endl;
#endif

            std::string lineStr(line);
            CarControl carCtrl(lineStr);
            if (carCtrl.getMeta()==RACE_RESTART)
            {
                RESTARTING[index] = 1;
#ifdef __DISABLE_RESTART__
                // char line[UDP_MSGLEN];
                memset(line, 0x0,UDP_MSGLEN );
                sprintf(line,"***restart***");
                // Sending the car state to the client
                if (sendto(listenSocket[index], line, strlen(line) + 1, 0,
                        (struct sockaddr *) &clientAddress[index],
                        sizeof(clientAddress[index])) < 0)
                    std::cerr << "Error: cannot send restart message";
#else
                car->RESTART=1;
#endif
            }

            // Set controls command and store them in variables
            oldAccel[index] = car->_accelCmd = carCtrl.getAccel();
            oldBrake[index] = car->_brakeCmd = carCtrl.getBrake();
            oldGear[index]  = car->_gearCmd  = carCtrl.getGear();
            oldSteer[index] = car->_steerCmd = carCtrl.getSteer();
            oldClutch[index] = car->_clutchCmd = carCtrl.getClutch();

            oldFocus[index] = car->_focusCmd = carCtrl.getFocus();//ML
        }
        else
        {
//#ifdef __UDP_SERVER_VERBOSE__
            std::cout << "Timeout for client answer\n";
//#endif

            // If no new controls are availables uses old ones...
            car->_accelCmd = oldAccel[index];
            car->_brakeCmd = oldBrake[index];
            car->_gearCmd  = oldGear[index];
            car->_steerCmd = oldSteer[index];
            car->_clutchCmd = oldClutch[index];

            car->_focusCmd = oldFocus[index];//ML
        }
    }
    else
    {
        car->_accelCmd = oldAccel[index];
        car->_brakeCmd = oldBrake[index];
        car->_gearCmd  = oldGear[index];
        car->_steerCmd = oldSteer[index];
        car->_clutchCmd = oldClutch[index];
        car->_focusCmd = oldFocus[index];//ML
    }
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
    RESTARTING[index]=0;
    if (trackSens != NULL)
    {
        delete trackSens[index];
        trackSens[index] = NULL;
    }

    if (oppSens[index] != NULL)
    {
        delete oppSens[index];
        oppSens[index] = NULL;
    }
    if (focusSens[index] != NULL)//ML
    {
        delete focusSens[index];
        focusSens[index] = NULL;
    }
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
    char line[UDP_MSGLEN];
    memset(line, 0x0,UDP_MSGLEN );

#ifdef __PRINT_RACE_RESULTS__
#define  max_pos 8

    int points[]={10,8,6,5,4,3,2,1};
    curPosition++;
    position[index]=curPosition;
    if (curPosition==1)
    {
        bonusBest=index;
        bonusDamage=index;
    }
    else
    {
        if (bestLap[index] < bestLap[bonusBest] && bestLap[index] > 0)
            bonusBest=index;
        if (damages[index] < damages[bonusDamage])
            bonusDamage=index;
    }
    if (curPosition==max_pos)
    {
        char fileName[200];
        sprintf(fileName,"%s.txt",trackName);
        printf("%s.txt\n",trackName);
        FILE *f = fopen (fileName,"a");
        for(int i = 0; i<max_pos; i++)
        {
            int curPoints = points[position[i]-1];
            if (bonusBest==i)
                curPoints+=2;
            if (bonusDamage==i)
                curPoints+=2;
            fprintf(f,"driver-%d,%d,%d,%f,%f,%f\n",i+1,position[i],curPoints,totalTime[i],bestLap[i],damages[i]);
            printf("driver-%d,%d,%d,%f,%f,%f\n",i+1,position[i],curPoints,totalTime[i],bestLap[i],damages[i]);
        }
        fprintf(f,"\n\n\n");
        fclose(f);
    }
    //std::cout << "car,pos,points,time,bestLap,damages"<< std::endl;
    //std::cout << "champ" << (index+1) <<"," << position <<"," << points[position-1] <<"," << totalTime[index] <<"," << bestLap[index] <<"\t" << damages[index]<< std::endl;
#endif

    if (RESTARTING[index]!=1)
    {
    //     char line[UDP_MSGLEN];
        sprintf(line,"***shutdown***");
        // Sending the car state to the client
        if (sendto(listenSocket[index], line, strlen(line) + 1, 0,
                   (struct sockaddr *) &clientAddress[index],
                   sizeof(clientAddress[index])) < 0)
            std::cerr << "Error: cannot send shutdown message";
    }
    else
    {
    //     char line[UDP_MSGLEN];
        sprintf(line,"***restart***");
        // Sending the car state to the client
        if (sendto(listenSocket[index], line, strlen(line) + 1, 0,
                   (struct sockaddr *) &clientAddress[index],
                   sizeof(clientAddress[index])) < 0)
            std::cerr << "Error: cannot send shutdown message";
    }
    RESTARTING[index]=0;
    if (trackSens[index] != NULL)
    {
        delete trackSens[index];
        trackSens[index] = NULL;
    }
    if (focusSens[index] != NULL)//ML
    {
        delete focusSens[index];
        focusSens[index] = NULL;
    }
    if (oppSens[index] != NULL)
    {
        delete oppSens[index];
        oppSens[index] = NULL;
    }
    CLOSE(listenSocket[index]);
}

double normRand(double avg,double std)
{
    double x1, x2, w, y1, y2;

    do {
        x1 = 2.0 * rand()/(double(RAND_MAX)) - 1.0;
        x2 = 2.0 * rand()/(double(RAND_MAX)) - 1.0;
        w = x1 * x1 + x2 * x2;
    } while ( w >= 1.0 );

    w = sqrt( (-2.0 * log( w ) ) / w );
    y1 = x1 * w;
    y2 = x2 * w;
    return y1*std + avg;
}
