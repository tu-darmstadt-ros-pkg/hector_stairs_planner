/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cmath>
#include <cstring>
#include <ctime>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <hector_sbpl_flipper_control_env/hector_sbpl_flipper_control_env.h>
#include <sbpl/utils/2Dgridsearch.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>

using namespace std;
using namespace hector_sbpl_flipper_control_env;

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_s = 0;
#endif

static long int checks = 0;

#define XYTHETAFLIP2INDEX(X,Y,THETA, FLIP) (THETA + X*EnvNAVXYTHETAFLIPCfg.NumThetaDirs + Y*EnvNAVXYTHETAFLIPCfg.EnvWidth_c*EnvNAVXYTHETAFLIPCfg.NumThetaDirs + FLIP*EnvNAVXYTHETAFLIPCfg.EnvWidth_c*EnvNAVXYTHETAFLIPCfg.NumThetaDirs*EnvNAVXYTHETAFLIPCfg.EnvHeight_c)

//-----------------constructors/destructors-------------------------------

HectorSBPLFlipperControlEnv::HectorSBPLFlipperControlEnv()
{
    SBPL_PRINTF("create HectorSBPLFlipperControlEnv");
    EnvNAVXYTHETAFLIPCfg.obsthresh = ENVNAVXYTHETALAT_DEFAULTOBSTHRESH;
    //the value that pretty much makes it disabled
    EnvNAVXYTHETAFLIPCfg.cost_inscribed_thresh = EnvNAVXYTHETAFLIPCfg.obsthresh;
    //the value that pretty much makes it disabled
    EnvNAVXYTHETAFLIPCfg.cost_possibly_circumscribed_thresh = EnvNAVXYTHETAFLIPCfg.obsthresh;

    EnvNAVXYTHETAFLIPCfg.Start_flipperFlag=0;
    EnvNAVXYTHETAFLIPCfg.End_flipperFlag=0;

    EnvNAVXYTHETAFLIPCfg.lowerStaisThresh=2;

    grid2Dsearchfromstart = NULL;
    grid2Dsearchfromgoal = NULL;
    bNeedtoRecomputeStartHeuristics = true;
    bNeedtoRecomputeGoalHeuristics = true;
    iteration = 0;

    EnvNAVXYTHETAFLIP.bInitialized = false;

    EnvNAVXYTHETAFLIPCfg.actionwidth = NAVXYTHETALAT_DEFAULT_ACTIONWIDTH;

    EnvNAVXYTHETAFLIPCfg.NumThetaDirs = NAVXYTHETALAT_THETADIRS;

    //no memory allocated in cfg yet
    EnvNAVXYTHETAFLIPCfg.Grid2D = NULL;
    EnvNAVXYTHETAFLIPCfg.ActionsV = NULL;
    EnvNAVXYTHETAFLIPCfg.PredActionsV = NULL;

    HashTableSize = 0;
    Coord2StateIDHashTable = NULL;
    Coord2StateIDHashTable_lookup = NULL;
}

HectorSBPLFlipperControlEnv::~HectorSBPLFlipperControlEnv()
{
    SBPL_PRINTF("destroying XYTHETALATTICE\n");
    //delete the states themselves first
    for (int i = 0; i < (int)StateID2CoordTable.size(); i++) {
        delete StateID2CoordTable.at(i);
        StateID2CoordTable.at(i) = NULL;
    }
    StateID2CoordTable.clear();

    //delete hashtable
    if (Coord2StateIDHashTable != NULL) {
        delete[] Coord2StateIDHashTable;
        Coord2StateIDHashTable = NULL;
    }
    if (Coord2StateIDHashTable_lookup != NULL) {
        delete[] Coord2StateIDHashTable_lookup;
        Coord2StateIDHashTable_lookup = NULL;
    }

    if (grid2Dsearchfromstart != NULL) delete grid2Dsearchfromstart;
    grid2Dsearchfromstart = NULL;

    if (grid2Dsearchfromgoal != NULL) delete grid2Dsearchfromgoal;
    grid2Dsearchfromgoal = NULL;

    if (EnvNAVXYTHETAFLIPCfg.Grid2D != NULL) {
        for (int x = 0; x < EnvNAVXYTHETAFLIPCfg.EnvWidth_c; x++)
            delete[] EnvNAVXYTHETAFLIPCfg.Grid2D[x];
        delete[] EnvNAVXYTHETAFLIPCfg.Grid2D;
        EnvNAVXYTHETAFLIPCfg.Grid2D = NULL;
    }

    //delete actions
    if (EnvNAVXYTHETAFLIPCfg.ActionsV != NULL) {
        for (int tind = 0; tind < EnvNAVXYTHETAFLIPCfg.NumThetaDirs; tind++)
            delete[] EnvNAVXYTHETAFLIPCfg.ActionsV[tind];
        delete[] EnvNAVXYTHETAFLIPCfg.ActionsV;
        EnvNAVXYTHETAFLIPCfg.ActionsV = NULL;
    }
    if (EnvNAVXYTHETAFLIPCfg.PredActionsV != NULL) {
        delete[] EnvNAVXYTHETAFLIPCfg.PredActionsV;
        EnvNAVXYTHETAFLIPCfg.PredActionsV = NULL;
    }
}

//---------------------------------------------------------------------

//-------------------problem specific and local functions---------------------

static unsigned int inthash(unsigned int key)
{
    key += (key << 12);
    key ^= (key >> 22);
    key += (key << 4);
    key ^= (key >> 9);
    key += (key << 10);
    key ^= (key >> 2);
    key += (key << 7);
    key ^= (key >> 12);
    return key;
}

void HectorSBPLFlipperControlEnv::SetConfiguration(int width, int height, const unsigned char* mapdata, int startx,
                                                   int starty, int starttheta, bool startFlipper, int goalx, int goaly, int goaltheta, bool endFlipper,
                                                   double cellsize_m, double nominalvel_mpersecs,
                                                   double timetoturn45degsinplace_secs,
                                                   const vector<sbpl_2Dpt_t> & robot_perimeterV)
{
    EnvNAVXYTHETAFLIPCfg.EnvWidth_c = width;
    EnvNAVXYTHETAFLIPCfg.EnvHeight_c = height;
    EnvNAVXYTHETAFLIPCfg.StartX_c = startx;
    EnvNAVXYTHETAFLIPCfg.StartY_c = starty;
    EnvNAVXYTHETAFLIPCfg.StartTheta = starttheta;
    EnvNAVXYTHETAFLIPCfg.Start_flipperFlag=startFlipper;

    if (EnvNAVXYTHETAFLIPCfg.StartX_c < 0 || EnvNAVXYTHETAFLIPCfg.StartX_c >= EnvNAVXYTHETAFLIPCfg.EnvWidth_c) {
        SBPL_ERROR("ERROR: illegal start coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvNAVXYTHETAFLIPCfg.StartY_c < 0 || EnvNAVXYTHETAFLIPCfg.StartY_c >= EnvNAVXYTHETAFLIPCfg.EnvHeight_c) {
        SBPL_ERROR("ERROR: illegal start coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvNAVXYTHETAFLIPCfg.StartTheta < 0 || EnvNAVXYTHETAFLIPCfg.StartTheta >= EnvNAVXYTHETAFLIPCfg.NumThetaDirs) {
        SBPL_ERROR("ERROR: illegal start coordinates for theta\n");
        throw new SBPL_Exception();
    }

    EnvNAVXYTHETAFLIPCfg.EndX_c = goalx;
    EnvNAVXYTHETAFLIPCfg.EndY_c = goaly;
    EnvNAVXYTHETAFLIPCfg.EndTheta = goaltheta;
    EnvNAVXYTHETAFLIPCfg.End_flipperFlag=endFlipper;

    if (EnvNAVXYTHETAFLIPCfg.EndX_c < 0 || EnvNAVXYTHETAFLIPCfg.EndX_c >= EnvNAVXYTHETAFLIPCfg.EnvWidth_c) {
        SBPL_ERROR("ERROR: illegal goal coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvNAVXYTHETAFLIPCfg.EndY_c < 0 || EnvNAVXYTHETAFLIPCfg.EndY_c >= EnvNAVXYTHETAFLIPCfg.EnvHeight_c) {
        SBPL_ERROR("ERROR: illegal goal coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvNAVXYTHETAFLIPCfg.EndTheta < 0 || EnvNAVXYTHETAFLIPCfg.EndTheta >= EnvNAVXYTHETAFLIPCfg.NumThetaDirs) {
        SBPL_ERROR("ERROR: illegal goal coordinates for theta\n");
        throw new SBPL_Exception();
    }

    EnvNAVXYTHETAFLIPCfg.FootprintPolygon = robot_perimeterV;

    EnvNAVXYTHETAFLIPCfg.nominalvel_mpersecs = nominalvel_mpersecs;
    EnvNAVXYTHETAFLIPCfg.cellsize_m = cellsize_m;
    EnvNAVXYTHETAFLIPCfg.timetoturn45degsinplace_secs = timetoturn45degsinplace_secs;

    //allocate the 2D environment
    EnvNAVXYTHETAFLIPCfg.Grid2D = new unsigned char*[EnvNAVXYTHETAFLIPCfg.EnvWidth_c];
    for (int x = 0; x < EnvNAVXYTHETAFLIPCfg.EnvWidth_c; x++) {
        EnvNAVXYTHETAFLIPCfg.Grid2D[x] = new unsigned char[EnvNAVXYTHETAFLIPCfg.EnvHeight_c];
    }

    //environment:
    if (0 == mapdata) {
        for (int y = 0; y < EnvNAVXYTHETAFLIPCfg.EnvHeight_c; y++) {
            for (int x = 0; x < EnvNAVXYTHETAFLIPCfg.EnvWidth_c; x++) {
                EnvNAVXYTHETAFLIPCfg.Grid2D[x][y] = 0;
            }
        }
    }
    else {
        for (int y = 0; y < EnvNAVXYTHETAFLIPCfg.EnvHeight_c; y++) {
            for (int x = 0; x < EnvNAVXYTHETAFLIPCfg.EnvWidth_c; x++) {
                EnvNAVXYTHETAFLIPCfg.Grid2D[x][y] = mapdata[x + y * width];
            }
        }
    }
}

bool HectorSBPLFlipperControlEnv::ReadinCell(sbpl_xy_theta_cell_t* cell, FILE* fIn)
{
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    cell->x = atoi(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    cell->y = atoi(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    cell->theta = atoi(sTemp);

    //normalize the angle
    cell->theta = NORMALIZEDISCTHETA(cell->theta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

    return true;
}

bool HectorSBPLFlipperControlEnv::ReadinPose(sbpl_xy_theta_pt_t* pose, FILE* fIn)
{
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    pose->x = atof(sTemp);

    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    pose->y = atof(sTemp);

    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    pose->theta = atof(sTemp);
    pose->theta = normalizeAngle(pose->theta);

    return true;
}

bool HectorSBPLFlipperControlEnv::ReadinMotionPrimitive(SBPL_xythetaflip_mprimitive* pMotPrim, FILE* fIn)
{
    char sTemp[1024];
    int dTemp;
    char sExpected[1024];
    int numofIntermPoses;

    //read in actionID
    strcpy(sExpected, "primID:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &pMotPrim->motprimID) != 1) return false;

    //read in start angle
    strcpy(sExpected, "startangle_c:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) == 0) {
        SBPL_ERROR("ERROR reading startangle\n");
        return false;
    }
    pMotPrim->starttheta_c = dTemp;

    //read in end pose
    strcpy(sExpected, "endpose_c:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }

    if (ReadinCell(&pMotPrim->endcell, fIn) == false) {
        SBPL_ERROR("ERROR: failed to read in endsearchpose\n");
        return false;
    }

    //read in action cost
    strcpy(sExpected, "additionalactioncostmult:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) != 1) return false;
    pMotPrim->additionalactioncostmult = dTemp;

    //read in intermediate poses
    strcpy(sExpected, "intermediateposes:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &numofIntermPoses) != 1) return false;
    //all intermposes should be with respect to 0,0 as starting pose since it will be added later and should be done
    //after the action is rotated by initial orientation
    for (int i = 0; i < numofIntermPoses; i++) {
        sbpl_xy_theta_pt_t intermpose;
        if (ReadinPose(&intermpose, fIn) == false) {
            SBPL_ERROR("ERROR: failed to read in intermediate poses\n");
            return false;
        }
        pMotPrim->intermptV.push_back(intermpose);
    }

    //set FillperFlag = false
    pMotPrim->flipperActuationFlag=false;

    //check that the last pose corresponds correctly to the last pose
    sbpl_xy_theta_pt_t sourcepose;
    sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    sourcepose.theta = DiscTheta2Cont(pMotPrim->starttheta_c, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);
    double mp_endx_m = sourcepose.x + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].x;
    double mp_endy_m = sourcepose.y + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].y;
    double mp_endtheta_rad = pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].theta;
    int endx_c = CONTXY2DISC(mp_endx_m, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    int endy_c = CONTXY2DISC(mp_endy_m, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    int endtheta_c = ContTheta2Disc(mp_endtheta_rad, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);
    if (endx_c != pMotPrim->endcell.x || endy_c != pMotPrim->endcell.y || endtheta_c != pMotPrim->endcell.theta) {
        SBPL_ERROR( "ERROR: incorrect primitive %d with startangle=%d "
                    "last interm point %f %f %f does not match end pose %d %d %d\n",
                    pMotPrim->motprimID, pMotPrim->starttheta_c,
                    pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].x,
                pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].y,
                pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].theta,
                pMotPrim->endcell.x, pMotPrim->endcell.y,
                pMotPrim->endcell.theta);
        return false;
    }

    return true;
}

bool HectorSBPLFlipperControlEnv::ReadMotionPrimitives(FILE* fMotPrims)
{
    char sTemp[1024], sExpected[1024];
    float fTemp;
    int dTemp;
    int totalNumofActions = 0;

    SBPL_PRINTF("Reading in motion primitives...");

    //read in the resolution
    strcpy(sExpected, "resolution_m:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%f", &fTemp) == 0) return false;
    if (fabs(fTemp - EnvNAVXYTHETAFLIPCfg.cellsize_m) > ERR_EPS) {
        SBPL_ERROR("ERROR: invalid resolution %f (instead of %f) in the dynamics file\n", fTemp,
                   EnvNAVXYTHETAFLIPCfg.cellsize_m);
        return false;
    }

    //read in the angular resolution
    strcpy(sExpected, "numberofangles:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%d", &dTemp) == 0) return false;
    if (dTemp != EnvNAVXYTHETAFLIPCfg.NumThetaDirs) {
        SBPL_ERROR("ERROR: invalid angular resolution %d angles (instead of %d angles) in the motion primitives file\n",
                   dTemp, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);
        return false;
    }

    //read in the total number of actions
    strcpy(sExpected, "totalnumberofprimitives:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%d", &totalNumofActions) == 0) {
        return false;
    }

    for (int i = 0; i < totalNumofActions; i++) {
        SBPL_xythetaflip_mprimitive motprim;

        if (HectorSBPLFlipperControlEnv::ReadinMotionPrimitive(&motprim, fMotPrims) == false) return false;

        EnvNAVXYTHETAFLIPCfg.mprimV.push_back(motprim);
    }

    //Insert Flipper Motion dX, dY, dThata==0; only flipperFlag=true number intermPoses=10;
    SBPL_PRINTF("add FillperPrimitiver");
    int numofIntermPoses= 10;
    int id= EnvNAVXYTHETAFLIPCfg.NumThetaDirs +1;
    for(int i=0; i< EnvNAVXYTHETAFLIPCfg.NumThetaDirs; i++){
        SBPL_xythetaflip_mprimitive motprim;
        sbpl_xy_theta_cell_t cell;

        motprim.motprimID= id;
        motprim.flipperActuationFlag= true;
        motprim.additionalactioncostmult=1;
        motprim.starttheta_c=i;

        cell.x =0;
        cell.y = 0;
        cell.theta = i;
        cell.theta = NORMALIZEDISCTHETA(cell.theta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);
        motprim.endcell=cell;


        for (int j = 0; j < numofIntermPoses; j++) {
            sbpl_xy_theta_pt_t intermpose;

            intermpose.x = 0;
            intermpose.y = 0;
            intermpose.theta =i * (2*M_PI/EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

            intermpose.theta = normalizeAngle(intermpose.theta);
            motprim.intermptV.push_back(intermpose);
        }

        EnvNAVXYTHETAFLIPCfg.mprimV.push_back(motprim);

    }

    //Insert Flipper Motion dX, dY, dThata==0; only flipperFlag=false number intermPoses=10;
    SBPL_PRINTF("add FillperPrimitiver");
    numofIntermPoses= 10;
    id= EnvNAVXYTHETAFLIPCfg.NumThetaDirs +2;
    for(int i=0; i< EnvNAVXYTHETAFLIPCfg.NumThetaDirs; i++){
        SBPL_xythetaflip_mprimitive motprim;
        sbpl_xy_theta_cell_t cell;

        motprim.motprimID= id;
        motprim.flipperActuationFlag= false;
        motprim.additionalactioncostmult=1;
        motprim.starttheta_c=i;

        cell.x =0;
        cell.y = 0;
        cell.theta = i;
        cell.theta = NORMALIZEDISCTHETA(cell.theta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);
        motprim.endcell=cell;


        for (int j = 0; j < numofIntermPoses; j++) {
            sbpl_xy_theta_pt_t intermpose;

            intermpose.x = 0;
            intermpose.y = 0;
            intermpose.theta =i * (2*M_PI/EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

            intermpose.theta = normalizeAngle(intermpose.theta);
            motprim.intermptV.push_back(intermpose);
        }

        EnvNAVXYTHETAFLIPCfg.mprimV.push_back(motprim);

    }

    SBPL_PRINTF("done ");

    return true;
}

void HectorSBPLFlipperControlEnv::ComputeReplanningDataforAction(EnvNAVXYTHETAFLIPAction_t* action)
{
    int j;

    //iterate over all the cells involved in the action
    sbpl_xy_theta_cell_t startcell3d, endcell3d;
    for (int i = 0; i < (int)action->intersectingcellsV.size(); i++) {
        //compute the translated affected search Pose - what state has an
        //outgoing action whose intersecting cell is at 0,0
        startcell3d.theta = action->starttheta;
        startcell3d.x = -action->intersectingcellsV.at(i).x;
        startcell3d.y = -action->intersectingcellsV.at(i).y;

        //compute the translated affected search Pose - what state has an
        //incoming action whose intersecting cell is at 0,0
        endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);
        endcell3d.x = startcell3d.x + action->dX;
        endcell3d.y = startcell3d.y + action->dY;

        //store the cells if not already there
        for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
            if (affectedsuccstatesV.at(j) == endcell3d) break;
        }
        if (j == (int)affectedsuccstatesV.size()) affectedsuccstatesV.push_back(endcell3d);

        for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
            if (affectedpredstatesV.at(j) == startcell3d) break;
        }
        if (j == (int)affectedpredstatesV.size()) affectedpredstatesV.push_back(startcell3d);
    }//over intersecting cells

    //add the centers since with h2d we are using these in cost computations
    //---intersecting cell = origin
    //compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
    startcell3d.theta = action->starttheta;
    startcell3d.x = -0;
    startcell3d.y = -0;

    //compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
    endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);
    endcell3d.x = startcell3d.x + action->dX;
    endcell3d.y = startcell3d.y + action->dY;

    //store the cells if not already there
    for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
        if (affectedsuccstatesV.at(j) == endcell3d) break;
    }
    if (j == (int)affectedsuccstatesV.size()) affectedsuccstatesV.push_back(endcell3d);

    for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
        if (affectedpredstatesV.at(j) == startcell3d) break;
    }
    if (j == (int)affectedpredstatesV.size()) affectedpredstatesV.push_back(startcell3d);

    //---intersecting cell = outcome state
    //compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
    startcell3d.theta = action->starttheta;
    startcell3d.x = -action->dX;
    startcell3d.y = -action->dY;

    //compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
    endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);
    endcell3d.x = startcell3d.x + action->dX;
    endcell3d.y = startcell3d.y + action->dY;

    for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
        if (affectedsuccstatesV.at(j) == endcell3d) break;
    }
    if (j == (int)affectedsuccstatesV.size()) affectedsuccstatesV.push_back(endcell3d);

    for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
        if (affectedpredstatesV.at(j) == startcell3d) break;
    }
    if (j == (int)affectedpredstatesV.size()) affectedpredstatesV.push_back(startcell3d);
}

//computes all the 3D states whose outgoing actions are potentially affected
//when cell (0,0) changes its status it also does the same for the 3D states
//whose incoming actions are potentially affected when cell (0,0) changes its
//status
void HectorSBPLFlipperControlEnv::ComputeReplanningData()
{
    //iterate over all actions
    //orientations
    for (int tind = 0; tind < EnvNAVXYTHETAFLIPCfg.NumThetaDirs; tind++) {
        //actions
        for (int aind = 0; aind < EnvNAVXYTHETAFLIPCfg.actionwidth; aind++) {
            //compute replanning data for this action
            ComputeReplanningDataforAction(&EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind]);
        }
    }
}

//here motionprimitivevector contains actions for all angles
void HectorSBPLFlipperControlEnv::PrecomputeActionswithCompleteMotionPrimitive(
        vector<SBPL_xythetaflip_mprimitive>* motionprimitiveV)
{
    SBPL_PRINTF("Pre-computing action data using motion primitives for every angle...\n");
    EnvNAVXYTHETAFLIPCfg.ActionsV = new EnvNAVXYTHETAFLIPAction_t*[EnvNAVXYTHETAFLIPCfg.NumThetaDirs];
    EnvNAVXYTHETAFLIPCfg.PredActionsV = new vector<EnvNAVXYTHETAFLIPAction_t*> [EnvNAVXYTHETAFLIPCfg.NumThetaDirs];

    if (motionprimitiveV->size() % EnvNAVXYTHETAFLIPCfg.NumThetaDirs != 0) {
        SBPL_ERROR("ERROR: motionprimitives should be uniform across actions\n");
        throw new SBPL_Exception();
    }

    EnvNAVXYTHETAFLIPCfg.actionwidth = ((int)motionprimitiveV->size()) / EnvNAVXYTHETAFLIPCfg.NumThetaDirs;

    //iterate over source angles
    int maxnumofactions = 0;
    for (int tind = 0; tind < EnvNAVXYTHETAFLIPCfg.NumThetaDirs; tind++) {
        SBPL_PRINTF("pre-computing for angle %d out of %d angles\n", tind, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

        EnvNAVXYTHETAFLIPCfg.ActionsV[tind] = new EnvNAVXYTHETAFLIPAction_t[EnvNAVXYTHETAFLIPCfg.actionwidth];

        //compute sourcepose
        sbpl_xy_theta_pt_t sourcepose;
        sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETAFLIPCfg.cellsize_m);
        sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETAFLIPCfg.cellsize_m);
        sourcepose.theta = DiscTheta2Cont(tind, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

        //iterate over motion primitives
        int numofactions = 0;
        int aind = -1;
        for (int mind = 0; mind < (int)motionprimitiveV->size(); mind++) {
            //find a motion primitive for this angle
            if (motionprimitiveV->at(mind).starttheta_c != tind) continue;

            aind++;
            numofactions++;

            //action index
            EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].aind = aind;

            //start angle
            EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].starttheta = tind;

            //compute dislocation
            EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].endtheta = motionprimitiveV->at(mind).endcell.theta;
            EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].dX = motionprimitiveV->at(mind).endcell.x;
            EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].dY = motionprimitiveV->at(mind).endcell.y;
            EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].flipperActuationFlag = motionprimitiveV->at(mind).flipperActuationFlag;

            //compute and store interm points as well as intersecting cells
            EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].intersectingcellsV.clear();
            EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].intermptV.clear();
            EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].interm3DcellsV.clear();

            sbpl_xy_theta_cell_t previnterm3Dcell;
            previnterm3Dcell.x = 0;
            previnterm3Dcell.y = 0;

            // Compute all the intersected cells for this action (intermptV and interm3DcellsV)
            for (int pind = 0; pind < (int)motionprimitiveV->at(mind).intermptV.size(); pind++) {
                sbpl_xy_theta_pt_t intermpt = motionprimitiveV->at(mind).intermptV[pind];
                EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);

                // also compute the intermediate discrete cells if not there already
                sbpl_xy_theta_pt_t pose;
                pose.x = intermpt.x + sourcepose.x;
                pose.y = intermpt.y + sourcepose.y;
                pose.theta = intermpt.theta;

                sbpl_xy_theta_cell_t intermediate2dCell;
                intermediate2dCell.x = CONTXY2DISC(pose.x, EnvNAVXYTHETAFLIPCfg.cellsize_m);
                intermediate2dCell.y = CONTXY2DISC(pose.y, EnvNAVXYTHETAFLIPCfg.cellsize_m);

                // add unique cells to the list
                if (EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].interm3DcellsV.size() == 0 || intermediate2dCell.x
                        != previnterm3Dcell.x || intermediate2dCell.y != previnterm3Dcell.y) {
                    EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].interm3DcellsV.push_back(intermediate2dCell);
                }

                previnterm3Dcell = intermediate2dCell;
            }

            if(isFlipperAction(&EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind])){
                //make the cost the max of the two times
                EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].cost = HECTOR_COSTMULT*HECTOR_FLIPPER_ACTUATION_COST;
                //use any additional cost multiplier
                EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].cost *= motionprimitiveV->at(mind).additionalactioncostmult;
            }else{

                //compute linear and angular time
                double linear_distance = 0;
                for (unsigned int i = 1; i < EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].intermptV.size(); i++) {
                    double x0 = EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].intermptV[i - 1].x;
                    double y0 = EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].intermptV[i - 1].y;
                    double x1 = EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].intermptV[i].x;
                    double y1 = EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].intermptV[i].y;
                    double dx = x1 - x0;
                    double dy = y1 - y0;
                    linear_distance += sqrt(dx * dx + dy * dy);
                }
                double linear_time = linear_distance / EnvNAVXYTHETAFLIPCfg.nominalvel_mpersecs;
                double angular_distance =
                        fabs(computeMinUnsignedAngleDiff(DiscTheta2Cont(EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].endtheta,
                                                                        EnvNAVXYTHETAFLIPCfg.NumThetaDirs),
                                                         DiscTheta2Cont(EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].starttheta,
                                                                        EnvNAVXYTHETAFLIPCfg.NumThetaDirs)));
                double angular_time = angular_distance / ((PI_CONST / 4.0) /
                                                          EnvNAVXYTHETAFLIPCfg.timetoturn45degsinplace_secs);
                //make the cost the max of the two times
                EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].cost =
                        (int)(ceil(HECTOR_COSTMULT * max(linear_time, angular_time)));
                //use any additional cost multiplier
                EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].cost *= motionprimitiveV->at(mind).additionalactioncostmult;

            }
            //now compute the intersecting cells for this motion (including ignoring the source footprint)
            get_2d_motion_cells(EnvNAVXYTHETAFLIPCfg.FootprintPolygon, motionprimitiveV->at(mind).intermptV,
                                &EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].intersectingcellsV,
                                EnvNAVXYTHETAFLIPCfg.cellsize_m);

#if DEBUG
            SBPL_FPRINTF(fDeb,
                         "action tind=%2d aind=%2d: dX=%3d dY=%3d endtheta=%3d (%6.2f degs -> %6.2f degs) "
                         "cost=%4d (mprimID %3d: %3d %3d %3d) numofintermcells = %d numofintercells=%d\n",
                         tind,
                         aind,
                         EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].dX,
                         EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].dY,
                         EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].endtheta,
                         EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].intermptV[0].theta * 180 / PI_CONST,
                    EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].intermptV[EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].intermptV.size() - 1].theta * 180 / PI_CONST, EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].cost,
                    motionprimitiveV->at(mind).motprimID, motionprimitiveV->at(mind).endcell.x,
                    motionprimitiveV->at(mind).endcell.y, motionprimitiveV->at(mind).endcell.theta,
                    (int)EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].interm3DcellsV.size(),
                    (int)EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].intersectingcellsV.size());
#endif
            //add to the list of backward actions
            int targettheta = EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind].endtheta;
            if (targettheta < 0) targettheta = targettheta + EnvNAVXYTHETAFLIPCfg.NumThetaDirs;
            EnvNAVXYTHETAFLIPCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETAFLIPCfg.ActionsV[tind][aind]));
        }

        if (maxnumofactions < numofactions) maxnumofactions = numofactions;
    }

    //at this point we don't allow nonuniform number of actions
    if (motionprimitiveV->size() != (size_t)(EnvNAVXYTHETAFLIPCfg.NumThetaDirs * maxnumofactions)) {
        SBPL_ERROR("ERROR: nonuniform number of actions is not supported "
                   "(maxnumofactions=%d while motprims=%d thetas=%d\n",
                   maxnumofactions, (unsigned int)motionprimitiveV->size(), EnvNAVXYTHETAFLIPCfg.NumThetaDirs);
        throw new SBPL_Exception();
    }

    //now compute replanning data
    ComputeReplanningData();

    SBPL_PRINTF("done pre-computing action data based on motion primitives\n");
}

void HectorSBPLFlipperControlEnv::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
    SBPL_ERROR("ERROR setAllActionsandAllOutcomesi\n");
    throw new SBPL_Exception();
    int cost;

#if DEBUG
    if(state->StateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in Env... function: stateID illegal\n");
        throw new SBPL_Exception();
    }

    if((int)state->Actions.size() != 0)
    {
        SBPL_ERROR("ERROR in Env_setAllActionsandAllOutcomes: actions already exist for the state\n");
        throw new SBPL_Exception();
    }
#endif

    //goal state should be absorbing
    if (state->StateID == EnvNAVXYTHETAFLIP.goalstateid) return;

    //get X, Y for the state
    EnvNAVXYTHETAFLIPHashEntry_t* HashEntry = StateID2CoordTable[state->StateID];

    //iterate through actions
    for (int aind = 0; aind < EnvNAVXYTHETAFLIPCfg.actionwidth; aind++) {
        EnvNAVXYTHETAFLIPAction_t* nav3daction = &EnvNAVXYTHETAFLIPCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
        int newY = HashEntry->Y + nav3daction->dY;
        int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

        //skip the invalid cells
        if (!IsValidCell(newX, newY, EnvNAVXYTHETAFLIPCfg.Grid2D[newX][newY])) continue;

        //check if stairs possible and flipperFlag
        if(PossibleStairsCell(newX, newY, newTheta,  EnvNAVXYTHETAFLIPCfg.Grid2D[newX][newY]) == false && isFlipperAction(nav3daction)) continue;

        //get cost
        cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction, HashEntry->currentRobotHeight);
        if (cost >= INFINITECOST) continue;

        //add the action
        CMDPACTION* action = state->AddAction(aind);

#if TIME_DEBUG
        clock_t currenttime = clock();
#endif

        EnvNAVXYTHETAFLIPHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, nav3daction->flipperActuationFlag)) == NULL) {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, nav3daction->flipperActuationFlag);
        }
        OutHashEntry->currentRobotHeight=EnvNAVXYTHETAFLIPCfg.Grid2D[newX][newY];
        OutHashEntry->flipperInStairsDrivingPos=HashEntry->flipperInStairsDrivingPos;
        action->AddOutcome(OutHashEntry->stateID, cost, 1.0);

#if TIME_DEBUG
        time3_addallout += clock()-currenttime;
#endif
    }
}

void HectorSBPLFlipperControlEnv::InitializeEnvConfig(vector<SBPL_xythetaflip_mprimitive>* motionprimitiveV)
{
    //aditional to configuration file initialization of EnvNAVXYTHETALATCfg if necessary

    //dXY dirs
    EnvNAVXYTHETAFLIPCfg.dXY[0][0] = -1;
    EnvNAVXYTHETAFLIPCfg.dXY[0][1] = -1;
    EnvNAVXYTHETAFLIPCfg.dXY[1][0] = -1;
    EnvNAVXYTHETAFLIPCfg.dXY[1][1] = 0;
    EnvNAVXYTHETAFLIPCfg.dXY[2][0] = -1;
    EnvNAVXYTHETAFLIPCfg.dXY[2][1] = 1;
    EnvNAVXYTHETAFLIPCfg.dXY[3][0] = 0;
    EnvNAVXYTHETAFLIPCfg.dXY[3][1] = -1;
    EnvNAVXYTHETAFLIPCfg.dXY[4][0] = 0;
    EnvNAVXYTHETAFLIPCfg.dXY[4][1] = 1;
    EnvNAVXYTHETAFLIPCfg.dXY[5][0] = 1;
    EnvNAVXYTHETAFLIPCfg.dXY[5][1] = -1;
    EnvNAVXYTHETAFLIPCfg.dXY[6][0] = 1;
    EnvNAVXYTHETAFLIPCfg.dXY[6][1] = 0;
    EnvNAVXYTHETAFLIPCfg.dXY[7][0] = 1;
    EnvNAVXYTHETAFLIPCfg.dXY[7][1] = 1;

    sbpl_xy_theta_pt_t temppose;
    temppose.x = 0.0;
    temppose.y = 0.0;
    temppose.theta = 0.0;
    vector<sbpl_2Dcell_t> footprint;
    get_2d_footprint_cells(EnvNAVXYTHETAFLIPCfg.FootprintPolygon, &footprint, temppose, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    SBPL_PRINTF("number of cells in footprint of the robot = %d\n", (unsigned int)footprint.size());

    for (vector<sbpl_2Dcell_t>::iterator it = footprint.begin(); it != footprint.end(); ++it) {
        SBPL_PRINTF("Footprint cell at (%d, %d)\n", it->x, it->y);
    }

#if DEBUG
    SBPL_FPRINTF(fDeb, "footprint cells (size=%d):\n", (int)footprint.size());
    for(int i = 0; i < (int) footprint.size(); i++)
    {
        SBPL_FPRINTF(fDeb, "%d %d (cont: %.3f %.3f)\n", footprint.at(i).x, footprint.at(i).y,
                     DISCXY2CONT(footprint.at(i).x, EnvNAVXYTHETALATCfg.cellsize_m),
                     DISCXY2CONT(footprint.at(i).y, EnvNAVXYTHETALATCfg.cellsize_m));
    }
#endif

    if (motionprimitiveV == NULL)
        SBPL_ERROR("no motion pimitives to precompute actions");
    else
        PrecomputeActionswithCompleteMotionPrimitive(motionprimitiveV);
}

bool HectorSBPLFlipperControlEnv::IsValidCell(int X, int Y, int currentRobotHeight)
{
    return (X >= 0 && X < EnvNAVXYTHETAFLIPCfg.EnvWidth_c && Y >= 0 && Y < EnvNAVXYTHETAFLIPCfg.EnvHeight_c &&
            fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[X][Y] - currentRobotHeight) < EnvNAVXYTHETAFLIPCfg.obsthresh);
}

bool HectorSBPLFlipperControlEnv::IsValidCellFlipper(int X, int Y, int currentRobotHeight, bool flipperflag)
{
    if(flipperflag){
        return (X >= 0 && X < EnvNAVXYTHETAFLIPCfg.EnvWidth_c && Y >= 0 && Y < EnvNAVXYTHETAFLIPCfg.EnvHeight_c &&
                EnvNAVXYTHETAFLIPCfg.Grid2D[X][Y] != 100 &&
                fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[X][Y] - currentRobotHeight) < EnvNAVXYTHETAFLIPCfg.obsthresh);
    }else{
        return (X >= 0 && X < EnvNAVXYTHETAFLIPCfg.EnvWidth_c && Y >= 0 && Y < EnvNAVXYTHETAFLIPCfg.EnvHeight_c &&
                EnvNAVXYTHETAFLIPCfg.Grid2D[X][Y] != 100 &&
                fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[X][Y] - currentRobotHeight) < EnvNAVXYTHETAFLIPCfg.obsthresh &&
                fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[X][Y] - currentRobotHeight) < EnvNAVXYTHETAFLIPCfg.lowerStaisThresh);
    }
}

bool HectorSBPLFlipperControlEnv::IsPossibleStairsTransitionFront(int X, int Y, int dX, int dY, int currentRobotHeight, int times, int theta, bool inFront)
{
    int dir=0;
    if(inFront){
        dir=1;
    }else{
        dir=-1;
    }
    sbpl_xy_theta_pt_t pose;

    //compute continuous pose
    pose.x = DISCXY2CONT(X, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.y = DISCXY2CONT(Y, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(theta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

    float maxX=0;
    for (int i = 0; i < (int)EnvNAVXYTHETAFLIPCfg.FootprintPolygon.size(); i++) {
        if(fabs(EnvNAVXYTHETAFLIPCfg.FootprintPolygon.at(i).x) > maxX){
            maxX=fabs(EnvNAVXYTHETAFLIPCfg.FootprintPolygon.at(i).x);
        }
    }

//    maxX=maxX+inflationRadius_;
    float offsetX= cos(pose.theta) * maxX;
    float offsetY= sin(pose.theta) * maxX;

    X=CONTXY2DISC(pose.x-offsetX*dir, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    Y=CONTXY2DISC(pose.y-offsetY*dir, EnvNAVXYTHETAFLIPCfg.cellsize_m);

    return (X >= 0 && X < EnvNAVXYTHETAFLIPCfg.EnvWidth_c && Y >= 0 && Y < EnvNAVXYTHETAFLIPCfg.EnvHeight_c &&
            fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[X][Y] - currentRobotHeight) < EnvNAVXYTHETAFLIPCfg.obsthresh &&
            //            fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[X][Y] - currentRobotHeight) != 0);
            fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[X][Y] - currentRobotHeight) >= EnvNAVXYTHETAFLIPCfg.lowerStaisThresh);
}

bool HectorSBPLFlipperControlEnv::RobotOnStairs(int X, int Y, int currentRobotHeight){
    return (X >= 0 && X < EnvNAVXYTHETAFLIPCfg.EnvWidth_c && Y >= 0 && Y < EnvNAVXYTHETAFLIPCfg.EnvHeight_c &&
            fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[X][Y] - currentRobotHeight) < EnvNAVXYTHETAFLIPCfg.obsthresh &&
            fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[X][Y] - currentRobotHeight) >= EnvNAVXYTHETAFLIPCfg.lowerStaisThresh);
}

bool HectorSBPLFlipperControlEnv::IsValidTransitionHeightDiff(int X, int Y, int currentRobotHeight, int flipper)
{
    if(flipper==1){
        return (X >= 0 && X < EnvNAVXYTHETAFLIPCfg.EnvWidth_c && Y >= 0 && Y < EnvNAVXYTHETAFLIPCfg.EnvHeight_c &&
                fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[X][Y] - currentRobotHeight) < 20);
    }else{
        ROS_INFO("TESST: %i",X >= 0 && X < EnvNAVXYTHETAFLIPCfg.EnvWidth_c && Y >= 0 && Y < EnvNAVXYTHETAFLIPCfg.EnvHeight_c &&
                 fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[X][Y] - currentRobotHeight) ==  0);
        return (X >= 0 && X < EnvNAVXYTHETAFLIPCfg.EnvWidth_c && Y >= 0 && Y < EnvNAVXYTHETAFLIPCfg.EnvHeight_c &&
                fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[X][Y] - currentRobotHeight) ==  0);
    }
}



bool HectorSBPLFlipperControlEnv::PossibleStairsCell(int X, int Y, int Theta, int currentRobotHeight)
{
    vector<sbpl_2Dcell_t> footprint;
    sbpl_xy_theta_pt_t pose;

    //compute continuous pose
    pose.x = DISCXY2CONT(X, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.y = DISCXY2CONT(Y, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(Theta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

    //compute footprint cells
    get_2d_footprint_cells(EnvNAVXYTHETAFLIPCfg.FootprintPolygon, &footprint, pose, EnvNAVXYTHETAFLIPCfg.cellsize_m);

    //iterate over all footprint cells
    for (int find = 0; find < (int)footprint.size(); find++) {
        int x = footprint.at(find).x;
        int y = footprint.at(find).y;

        if (x < 0 || x >= EnvNAVXYTHETAFLIPCfg.EnvWidth_c || y < 0 || y >= EnvNAVXYTHETAFLIPCfg.EnvHeight_c ||
                fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[x][y] - currentRobotHeight)>= EnvNAVXYTHETAFLIPCfg.obsthresh)
        {
            return false;
        }else{
            if(fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[x][y] - currentRobotHeight) > EnvNAVXYTHETAFLIPCfg.lowerStaisThresh){
                return true;
            }
        }
    }

    return false;
}

bool HectorSBPLFlipperControlEnv::PossibleStairsCellAtStartGoal(int X, int Y, int Theta, int currentRobotHeight)
{
    vector<sbpl_2Dcell_t> footprint;
    sbpl_xy_theta_pt_t pose;

    //compute continuous pose
    pose.x = DISCXY2CONT(X, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.y = DISCXY2CONT(Y, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(Theta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

    //compute footprint cells
    get_2d_footprint_cells(EnvNAVXYTHETAFLIPCfg.FootprintPolygon, &footprint, pose, EnvNAVXYTHETAFLIPCfg.cellsize_m);

    //iterate over all footprint cells
    for (int find = 0; find < (int)footprint.size(); find++) {
        int x = footprint.at(find).x;
        int y = footprint.at(find).y;

            if(fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[x][y] - currentRobotHeight) > EnvNAVXYTHETAFLIPCfg.lowerStaisThresh &&
                    fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[x][y] - currentRobotHeight) < EnvNAVXYTHETAFLIPCfg.obsthresh){
                return true;
            }
        }

    return false;
}

bool HectorSBPLFlipperControlEnv::IsWithinMapCell(int X, int Y)
{
    return (X >= 0 && X < EnvNAVXYTHETAFLIPCfg.EnvWidth_c && Y >= 0 && Y < EnvNAVXYTHETAFLIPCfg.EnvHeight_c);
}

bool HectorSBPLFlipperControlEnv::IsValidConfiguration(int X, int Y, int Theta, int currentRobotHeight)
{
    vector<sbpl_2Dcell_t> footprint;
    sbpl_xy_theta_pt_t pose;

    //compute continuous pose
    pose.x = DISCXY2CONT(X, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.y = DISCXY2CONT(Y, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(Theta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

    //compute footprint cells
    get_2d_footprint_cells(EnvNAVXYTHETAFLIPCfg.FootprintPolygon, &footprint, pose, EnvNAVXYTHETAFLIPCfg.cellsize_m);

    //iterate over all footprint cells
    for (int find = 0; find < (int)footprint.size(); find++) {
        int x = footprint.at(find).x;
        int y = footprint.at(find).y;

        if (x < 0 || x >= EnvNAVXYTHETAFLIPCfg.EnvWidth_c || y < 0 || y >= EnvNAVXYTHETAFLIPCfg.EnvHeight_c ||
                fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[x][y] - currentRobotHeight) >= EnvNAVXYTHETAFLIPCfg.obsthresh)
        {
            return false;
        }
    }

    return true;
}

bool HectorSBPLFlipperControlEnv::IsValidConfigurationWithFlipper(int X, int Y, int Theta, int currentRobotHeight)
{
    vector<sbpl_2Dcell_t> footprint;
    sbpl_xy_theta_pt_t pose;

    //compute continuous pose
    pose.x = DISCXY2CONT(X, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.y = DISCXY2CONT(Y, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(Theta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

    //compute footprint cells
    std::vector<sbpl_2Dpt_t> footprintPolygonWithFlipper= EnvNAVXYTHETAFLIPCfg.FootprintPolygon;
    footprintPolygonWithFlipper.at(0).x= footprintPolygonWithFlipper.at(1).x + EnvNAVXYTHETAFLIPCfg.flipperOffsetX;
    footprintPolygonWithFlipper.at(1).x= footprintPolygonWithFlipper.at(2).x - EnvNAVXYTHETAFLIPCfg.flipperOffsetX;
    footprintPolygonWithFlipper.at(2).x= footprintPolygonWithFlipper.at(1).x - EnvNAVXYTHETAFLIPCfg.flipperOffsetX;
    footprintPolygonWithFlipper.at(3).x= footprintPolygonWithFlipper.at(2).x + EnvNAVXYTHETAFLIPCfg.flipperOffsetX;
    int flipperTolerance=40;


    get_2d_footprint_cells(footprintPolygonWithFlipper, &footprint, pose, EnvNAVXYTHETAFLIPCfg.cellsize_m);

    //iterate over all footprint cells
    for (int find = 0; find < (int)footprint.size(); find++) {
        int x = footprint.at(find).x;
        int y = footprint.at(find).y;

        if (x < 0 || x >= EnvNAVXYTHETAFLIPCfg.EnvWidth_c || y < 0 || y >= EnvNAVXYTHETAFLIPCfg.EnvHeight_c ||
                fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[x][y] - currentRobotHeight) >= EnvNAVXYTHETAFLIPCfg.obsthresh+ flipperTolerance)
        {
            return false;
        }
    }

    return true;
}

bool HectorSBPLFlipperControlEnv::FlipperActionIsValid(int X, int Y, int Theta, int currentRobotHeight){
    sbpl_xy_theta_pt_t pose;

    //compute continuous pose
    pose.x = DISCXY2CONT(X, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.y = DISCXY2CONT(Y, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(Theta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

    float offsetX= cos(pose.theta) * EnvNAVXYTHETAFLIPCfg.flipperOffsetX;
    float offsetY= sin(pose.theta) * EnvNAVXYTHETAFLIPCfg.flipperOffsetX;

    X=CONTXY2DISC(pose.x-offsetX, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    Y=CONTXY2DISC(pose.y-offsetY, EnvNAVXYTHETAFLIPCfg.cellsize_m);

    if (X < 0 || X >= EnvNAVXYTHETAFLIPCfg.EnvWidth_c || Y < 0 || Y >= EnvNAVXYTHETAFLIPCfg.EnvHeight_c ||
            fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[X][Y] - currentRobotHeight) >= EnvNAVXYTHETAFLIPCfg.obsthresh)
    {
        return false;
    }else{
        return true;
    }

}

bool HectorSBPLFlipperControlEnv::ValidStartGoalPosition(int sourceX, int sourceY, int theta){
    sbpl_xy_theta_pt_t pose;

    //compute continuous pose
    pose.x = DISCXY2CONT(sourceX, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.y = DISCXY2CONT(sourceY, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(theta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

    float maxX=0;
    for (int i = 0; i < (int)EnvNAVXYTHETAFLIPCfg.FootprintPolygon.size(); i++) {
        if(fabs(EnvNAVXYTHETAFLIPCfg.FootprintPolygon.at(i).x) > maxX){
            maxX=fabs(EnvNAVXYTHETAFLIPCfg.FootprintPolygon.at(i).x);
        }
    }

    float offsetX= cos(pose.theta) * maxX;
    float offsetY= sin(pose.theta) * maxX;

    int X1=CONTXY2DISC(pose.x-offsetX, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    int Y1=CONTXY2DISC(pose.y-offsetY, EnvNAVXYTHETAFLIPCfg.cellsize_m);

    int X2=CONTXY2DISC(pose.x+offsetX, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    int Y2=CONTXY2DISC(pose.y+offsetY, EnvNAVXYTHETAFLIPCfg.cellsize_m);


   if(EnvNAVXYTHETAFLIPCfg.Grid2D[X1][Y1] != 100 && EnvNAVXYTHETAFLIPCfg.Grid2D[X2][Y2] != 100){
       return true;
   }else{
       return false;
   }

}

bool HectorSBPLFlipperControlEnv::PossibleStairsAtStartGoal(int sourceX, int sourceY, int theta, int currentRobotHeight){
    sbpl_xy_theta_pt_t pose;

    //compute continuous pose
    pose.x = DISCXY2CONT(sourceX, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.y = DISCXY2CONT(sourceY, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(theta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

    float maxX=0;
    for (int i = 0; i < (int)EnvNAVXYTHETAFLIPCfg.FootprintPolygon.size(); i++) {
        if(fabs(EnvNAVXYTHETAFLIPCfg.FootprintPolygon.at(i).x) > maxX){
            maxX=fabs(EnvNAVXYTHETAFLIPCfg.FootprintPolygon.at(i).x);
        }
    }

    float offsetX= cos(pose.theta) * maxX;
    float offsetY= sin(pose.theta) * maxX;

    int X1=CONTXY2DISC(pose.x-offsetX, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    int Y1=CONTXY2DISC(pose.y-offsetY, EnvNAVXYTHETAFLIPCfg.cellsize_m);

    int X2=CONTXY2DISC(pose.x+offsetX, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    int Y2=CONTXY2DISC(pose.y+offsetY, EnvNAVXYTHETAFLIPCfg.cellsize_m);


   if((currentRobotHeight != EnvNAVXYTHETAFLIPCfg.Grid2D[X1][Y1] || currentRobotHeight != EnvNAVXYTHETAFLIPCfg.Grid2D[X2][Y2]) && EnvNAVXYTHETAFLIPCfg.Grid2D[X1][Y1] != 100 && EnvNAVXYTHETAFLIPCfg.Grid2D[X2][Y2] != 100){
       return true;
   }else{
       return false;
   }

}

bool HectorSBPLFlipperControlEnv::RotationOnStairs(int sourceX, int sourceY, int sourceTheta, int newTheta, int currentRobotHeight){
    if(sourceTheta == newTheta){
        return false;
    }
    sbpl_xy_theta_pt_t pose;

    //compute continuous pose
    pose.x = DISCXY2CONT(sourceX, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.y = DISCXY2CONT(sourceY, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(newTheta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

    float offsetX= cos(pose.theta) * 0.1;
    float offsetY= sin(pose.theta) * 0.1;

    int X1=CONTXY2DISC(pose.x-offsetX, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    int Y1=CONTXY2DISC(pose.y-offsetY, EnvNAVXYTHETAFLIPCfg.cellsize_m);

    int X2=CONTXY2DISC(pose.x+offsetX, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    int Y2=CONTXY2DISC(pose.y+offsetY, EnvNAVXYTHETAFLIPCfg.cellsize_m);

   if(currentRobotHeight != EnvNAVXYTHETAFLIPCfg.Grid2D[X1][Y1] || currentRobotHeight != EnvNAVXYTHETAFLIPCfg.Grid2D[X2][Y2]){
       return true;
   }else{
       return false;
   }

}

bool HectorSBPLFlipperControlEnv::IsValidRotation(int sourceX, int sourceY, int sourceTheta, int newTheta, int currentRobotHeight){
    if(sourceTheta == newTheta){
        return true;
    }
    sbpl_xy_theta_pt_t pose;

    //compute continuous pose
    pose.x = DISCXY2CONT(sourceX, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.y = DISCXY2CONT(sourceY, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(newTheta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

    float maxX=0;
    for (int i = 0; i < (int)EnvNAVXYTHETAFLIPCfg.FootprintPolygon.size(); i++) {
        if(fabs(EnvNAVXYTHETAFLIPCfg.FootprintPolygon.at(i).x) > maxX){
            maxX=fabs(EnvNAVXYTHETAFLIPCfg.FootprintPolygon.at(i).x);
        }
    }

    maxX=maxX - inflationRadius_+0.1;
    float offsetX= cos(pose.theta) * maxX;
    float offsetY= sin(pose.theta) * maxX;

    int X1=CONTXY2DISC(pose.x-offsetX, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    int Y1=CONTXY2DISC(pose.y-offsetY, EnvNAVXYTHETAFLIPCfg.cellsize_m);

    int X2=CONTXY2DISC(pose.x+offsetX, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    int Y2=CONTXY2DISC(pose.y+offsetY, EnvNAVXYTHETAFLIPCfg.cellsize_m);

   if(currentRobotHeight != EnvNAVXYTHETAFLIPCfg.Grid2D[X1][Y1] || currentRobotHeight != EnvNAVXYTHETAFLIPCfg.Grid2D[X2][Y2]){
       return false;
   }else{
       return true;
   }

}

bool HectorSBPLFlipperControlEnv::RobotCompleteOnStairs(int X, int Y, int Theta, int currentRobotHeight)
{
    vector<sbpl_2Dcell_t> footprint;
    sbpl_xy_theta_pt_t pose;
    int maxHeightDiff=0;
    int heightDiffTresh=4;

    //compute continuous pose
    pose.x = DISCXY2CONT(X, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.y = DISCXY2CONT(Y, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(Theta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

    //compute footprint cells
    get_2d_footprint_cells(EnvNAVXYTHETAFLIPCfg.FootprintPolygon, &footprint, pose, EnvNAVXYTHETAFLIPCfg.cellsize_m);

    //iterate over all footprint cells
    for (int find = 0; find < (int)footprint.size(); find++) {
        int x = footprint.at(find).x;
        int y = footprint.at(find).y;

        if (fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[x][y] - currentRobotHeight) >= maxHeightDiff){
            maxHeightDiff=fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[x][y] - currentRobotHeight);
        }
    }

    if(maxHeightDiff>heightDiffTresh){
        return true;
    }else{
        return false;
    }
}

int HectorSBPLFlipperControlEnv::getFootprintMaxHeightDistScaled(int X, int Y, int Theta, float scale){
    vector<sbpl_2Dcell_t> footprint;
    sbpl_xy_theta_pt_t pose;
    int maxHeight=0;
    int minHeight=101;

    //compute continuous pose
    pose.x = DISCXY2CONT(X, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.y = DISCXY2CONT(Y, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(Theta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

    //compute footprint cells
    get_2d_footprint_cells(EnvNAVXYTHETAFLIPCfg.FootprintPolygon, &footprint, pose, EnvNAVXYTHETAFLIPCfg.cellsize_m);

    //iterate over all footprint cells
    for (int find = 0; find < (int)footprint.size(); find++) {
        int x = footprint.at(find).x;
        int y = footprint.at(find).y;

        if (EnvNAVXYTHETAFLIPCfg.Grid2D[x][y] >= maxHeight){
            maxHeight=EnvNAVXYTHETAFLIPCfg.Grid2D[x][y];
        }
        if (EnvNAVXYTHETAFLIPCfg.Grid2D[x][y] <= minHeight){
            minHeight=EnvNAVXYTHETAFLIPCfg.Grid2D[x][y];
        }
    }

    return maxHeight-(maxHeight-minHeight)*scale;
}

int HectorSBPLFlipperControlEnv::GetActionCost(int SourceX, int SourceY, int SourceTheta,
                                               EnvNAVXYTHETAFLIPAction_t* action, int currentRobotHeight)
{

    if(EnvNAVXYTHETAFLIPCfg.Grid2D[SourceX + action->dX][SourceY + action->dY]==100){
        return INFINITECOST;
    }

    if(isFlipperAction(action)){
        // flipper collision check
        if(FlipperActionIsValid(SourceX, SourceY, SourceTheta, currentRobotHeight)){
            return action->cost;
        }else{
            return INFINITECOST;
        }
    }
    sbpl_2Dcell_t cell;
    sbpl_xy_theta_cell_t interm3Dcell;
    int i;

    //TODO - go over bounding box (minpt and maxpt) to test validity and skip
    //testing boundaries below, also order intersect cells so that the four
    //farthest pts go first

    if (fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[SourceX + action->dX][SourceY + action->dY] - EnvNAVXYTHETAFLIPCfg.Grid2D[SourceX][SourceY]) >=
            EnvNAVXYTHETAFLIPCfg.cost_inscribed_thresh)
    {
        return INFINITECOST;
    }

    //need to iterate over discretized center cells and compute cost based on them
    unsigned char maxcellcost = 0;
    for (i = 0; i < (int)action->interm3DcellsV.size(); i++) {
        interm3Dcell = action->interm3DcellsV.at(i);
        interm3Dcell.x = interm3Dcell.x + SourceX;
        interm3Dcell.y = interm3Dcell.y + SourceY;

        if (interm3Dcell.x < 0 || interm3Dcell.x >= EnvNAVXYTHETAFLIPCfg.EnvWidth_c || interm3Dcell.y < 0
                || interm3Dcell.y >= EnvNAVXYTHETAFLIPCfg.EnvHeight_c) {
            return INFINITECOST;
        }

        maxcellcost = __max(maxcellcost, fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[interm3Dcell.x][interm3Dcell.y] - EnvNAVXYTHETAFLIPCfg.Grid2D[SourceX][SourceY]));

        //check that the robot is NOT in the cell at which there is no valid orientation
        if (maxcellcost >= EnvNAVXYTHETAFLIPCfg.cost_inscribed_thresh) {
            return INFINITECOST;
        }
    }

    //check collisions that for the particular footprint orientation along the action
    if (EnvNAVXYTHETAFLIPCfg.FootprintPolygon.size() > 1 && (int)maxcellcost >=
            EnvNAVXYTHETAFLIPCfg.cost_possibly_circumscribed_thresh)
    {
        checks++;

        for (i = 0; i < (int)action->intersectingcellsV.size(); i++) {
            //get the cell in the map
            cell = action->intersectingcellsV.at(i);
            cell.x = cell.x + SourceX;
            cell.y = cell.y + SourceY;

            //check validity
            if (!IsValidCell(cell.x, cell.y, currentRobotHeight)){
                return INFINITECOST;
            }

            //if(EnvNAVXYTHETALATCfg.Grid2D[cell.x][cell.y] > currentmaxcost)
            ////cost computation changed: cost = max(cost of centers of the
            //robot along action)
            //	currentmaxcost = EnvNAVXYTHETALATCfg.Grid2D[cell.x][cell.y];
            //	//intersecting cells are only used for collision checking
        }
    }

    //to ensure consistency of h2D:
    maxcellcost = __max(maxcellcost, fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[SourceX][SourceY]- EnvNAVXYTHETAFLIPCfg.Grid2D[SourceX][SourceY]));
    int currentmaxcost =
            (int)__max(maxcellcost, fabs(EnvNAVXYTHETAFLIPCfg.Grid2D[SourceX + action->dX][SourceY + action->dY]-EnvNAVXYTHETAFLIPCfg.Grid2D[SourceX][SourceY]));


//    return action->cost * (currentmaxcost + 1); //use cell cost as multiplicative factor
    //    ROS_INFO("action cost: %i", action->cost);
        return action->cost * 1; //use cell cost as multiplicative factor
}

double HectorSBPLFlipperControlEnv::EuclideanDistance_m(int X1, int Y1, int X2, int Y2)
{
    int sqdist = ((X1 - X2) * (X1 - X2) + (Y1 - Y2) * (Y1 - Y2));
    return EnvNAVXYTHETAFLIPCfg.cellsize_m * sqrt((double)sqdist);
}

//calculates a set of cells that correspond to the specified footprint
//adds points to it (does not clear it beforehand)
void HectorSBPLFlipperControlEnv::CalculateFootprintForPose(sbpl_xy_theta_pt_t pose, vector<sbpl_2Dcell_t>* footprint,
                                                            const vector<sbpl_2Dpt_t>& FootprintPolygon)
{
    int pind;

#if DEBUG
    //  SBPL_PRINTF("---Calculating Footprint for Pose: %f %f %f---\n",
    //	 pose.x, pose.y, pose.theta);
#endif

    //handle special case where footprint is just a point
    if (FootprintPolygon.size() <= 1) {
        sbpl_2Dcell_t cell;
        cell.x = CONTXY2DISC(pose.x, EnvNAVXYTHETAFLIPCfg.cellsize_m);
        cell.y = CONTXY2DISC(pose.y, EnvNAVXYTHETAFLIPCfg.cellsize_m);

        for (pind = 0; pind < (int)footprint->size(); pind++) {
            if (cell.x == footprint->at(pind).x && cell.y == footprint->at(pind).y) break;
        }
        if (pind == (int)footprint->size()) footprint->push_back(cell);
        return;
    }

    vector<sbpl_2Dpt_t> bounding_polygon;
    unsigned int find;
    double max_x = -INFINITECOST, min_x = INFINITECOST, max_y = -INFINITECOST, min_y = INFINITECOST;
    sbpl_2Dpt_t pt(0, 0);
    for (find = 0; find < FootprintPolygon.size(); find++) {
        //rotate and translate the corner of the robot
        pt = FootprintPolygon[find];

        //rotate and translate the point
        sbpl_2Dpt_t corner;
        corner.x = cos(pose.theta) * pt.x - sin(pose.theta) * pt.y + pose.x;
        corner.y = sin(pose.theta) * pt.x + cos(pose.theta) * pt.y + pose.y;
        bounding_polygon.push_back(corner);
#if DEBUG
        //    SBPL_PRINTF("Pt: %f %f, Corner: %f %f\n", pt.x, pt.y, corner.x, corner.y);
#endif
        if (corner.x < min_x || find == 0) {
            min_x = corner.x;
        }
        if (corner.x > max_x || find == 0) {
            max_x = corner.x;
        }
        if (corner.y < min_y || find == 0) {
            min_y = corner.y;
        }
        if (corner.y > max_y || find == 0) {
            max_y = corner.y;
        }
    }

#if DEBUG
    //  SBPL_PRINTF("Footprint bounding box: %f %f %f %f\n", min_x, max_x, min_y, max_y);
#endif
    //initialize previous values to something that will fail the if condition during the first iteration in the for loop
    int prev_discrete_x = CONTXY2DISC(pt.x, EnvNAVXYTHETAFLIPCfg.cellsize_m) + 1;
    int prev_discrete_y = CONTXY2DISC(pt.y, EnvNAVXYTHETAFLIPCfg.cellsize_m) + 1;
    int prev_inside = 0;
    int discrete_x;
    int discrete_y;

    for (double x = min_x; x <= max_x; x += EnvNAVXYTHETAFLIPCfg.cellsize_m / 3) {
        for (double y = min_y; y <= max_y; y += EnvNAVXYTHETAFLIPCfg.cellsize_m / 3) {
            pt.x = x;
            pt.y = y;
            discrete_x = CONTXY2DISC(pt.x, EnvNAVXYTHETAFLIPCfg.cellsize_m);
            discrete_y = CONTXY2DISC(pt.y, EnvNAVXYTHETAFLIPCfg.cellsize_m);

            //see if we just tested this point
            if (discrete_x != prev_discrete_x || discrete_y != prev_discrete_y || prev_inside == 0) {

#if DEBUG
                //		SBPL_PRINTF("Testing point: %f %f Discrete: %d %d\n", pt.x, pt.y, discrete_x, discrete_y);
#endif

                if (IsInsideFootprint(pt, &bounding_polygon)) {
                    //convert to a grid point

#if DEBUG
                    //			SBPL_PRINTF("Pt Inside %f %f\n", pt.x, pt.y);
#endif

                    sbpl_2Dcell_t cell;
                    cell.x = discrete_x;
                    cell.y = discrete_y;

                    //insert point if not there already
                    int pind = 0;
                    for (pind = 0; pind < (int)footprint->size(); pind++) {
                        if (cell.x == footprint->at(pind).x && cell.y == footprint->at(pind).y) break;
                    }
                    if (pind == (int)footprint->size()) footprint->push_back(cell);

                    prev_inside = 1;

#if DEBUG
                    //			SBPL_PRINTF("Added pt to footprint: %f %f\n", pt.x, pt.y);
#endif
                }
                else {
                    prev_inside = 0;
                }

            }
            else {
#if DEBUG
                //SBPL_PRINTF("Skipping pt: %f %f\n", pt.x, pt.y);
#endif
            }

            prev_discrete_x = discrete_x;
            prev_discrete_y = discrete_y;
        }//over x_min...x_max
    }
}

//calculates a set of cells that correspond to the footprint of the base
//adds points to it (does not clear it beforehand)
void HectorSBPLFlipperControlEnv::CalculateFootprintForPose(sbpl_xy_theta_pt_t pose, vector<sbpl_2Dcell_t>* footprint)
{
    CalculateFootprintForPose(pose, footprint, EnvNAVXYTHETAFLIPCfg.FootprintPolygon);
}

//removes a set of cells that correspond to the specified footprint at the sourcepose
//adds points to it (does not clear it beforehand)
void HectorSBPLFlipperControlEnv::RemoveSourceFootprint(sbpl_xy_theta_pt_t sourcepose,
                                                        vector<sbpl_2Dcell_t>* footprint,
                                                        const vector<sbpl_2Dpt_t>& FootprintPolygon)
{
    vector<sbpl_2Dcell_t> sourcefootprint;

    //compute source footprint
    get_2d_footprint_cells(FootprintPolygon, &sourcefootprint, sourcepose, EnvNAVXYTHETAFLIPCfg.cellsize_m);

    //now remove the source cells from the footprint
    for (int sind = 0; sind < (int)sourcefootprint.size(); sind++) {
        for (int find = 0; find < (int)footprint->size(); find++) {
            if (sourcefootprint.at(sind).x == footprint->at(find).x && sourcefootprint.at(sind).y
                    == footprint->at(find).y) {
                footprint->erase(footprint->begin() + find);
                break;
            }
        }//over footprint
    }//over source
}

//removes a set of cells that correspond to the footprint of the base at the sourcepose
//adds points to it (does not clear it beforehand)
void HectorSBPLFlipperControlEnv::RemoveSourceFootprint(sbpl_xy_theta_pt_t sourcepose,
                                                        vector<sbpl_2Dcell_t>* footprint)
{
    RemoveSourceFootprint(sourcepose, footprint, EnvNAVXYTHETAFLIPCfg.FootprintPolygon);
}

//------------------------------------------------------------------------------

//------------------------------Heuristic computation--------------------------

void HectorSBPLFlipperControlEnv::EnsureHeuristicsUpdated(bool bGoalHeuristics)
{
    if (bNeedtoRecomputeStartHeuristics && !bGoalHeuristics) {
        grid2Dsearchfromstart->search(EnvNAVXYTHETAFLIPCfg.Grid2D, EnvNAVXYTHETAFLIPCfg.cost_inscribed_thresh,
                                      EnvNAVXYTHETAFLIPCfg.StartX_c, EnvNAVXYTHETAFLIPCfg.StartY_c,
                                      EnvNAVXYTHETAFLIPCfg.EndX_c, EnvNAVXYTHETAFLIPCfg.EndY_c,
                                      SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH);
        bNeedtoRecomputeStartHeuristics = false;
        SBPL_PRINTF("2dsolcost_infullunits=%d\n",
                    (int)(grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(EnvNAVXYTHETAFLIPCfg.EndX_c,
                                                                                   EnvNAVXYTHETAFLIPCfg.EndY_c) /
                          EnvNAVXYTHETAFLIPCfg.nominalvel_mpersecs));

    }

    if (bNeedtoRecomputeGoalHeuristics && bGoalHeuristics) {
        grid2Dsearchfromgoal->search(EnvNAVXYTHETAFLIPCfg.Grid2D, EnvNAVXYTHETAFLIPCfg.cost_inscribed_thresh,
                                     EnvNAVXYTHETAFLIPCfg.EndX_c, EnvNAVXYTHETAFLIPCfg.EndY_c,
                                     EnvNAVXYTHETAFLIPCfg.StartX_c, EnvNAVXYTHETAFLIPCfg.StartY_c,
                                     SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH);
        bNeedtoRecomputeGoalHeuristics = false;
        SBPL_PRINTF("2dsolcost_infullunits=%d\n",
                    (int)(grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(EnvNAVXYTHETAFLIPCfg.StartX_c,
                                                                                  EnvNAVXYTHETAFLIPCfg.StartY_c) /
                          EnvNAVXYTHETAFLIPCfg.nominalvel_mpersecs));
    }
}

void HectorSBPLFlipperControlEnv::ComputeHeuristicValues()
{
    //whatever necessary pre-computation of heuristic values is done here
    SBPL_PRINTF("Precomputing heuristics...\n");

    //allocated 2D grid searches
    grid2Dsearchfromstart = new SBPL2DGridSearch(EnvNAVXYTHETAFLIPCfg.EnvWidth_c, EnvNAVXYTHETAFLIPCfg.EnvHeight_c,
                                                 (float)EnvNAVXYTHETAFLIPCfg.cellsize_m);
    grid2Dsearchfromgoal = new SBPL2DGridSearch(EnvNAVXYTHETAFLIPCfg.EnvWidth_c, EnvNAVXYTHETAFLIPCfg.EnvHeight_c,
                                                (float)EnvNAVXYTHETAFLIPCfg.cellsize_m);

    //set OPEN type to sliding buckets
    grid2Dsearchfromstart->setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS);
    grid2Dsearchfromgoal->setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS);

    SBPL_PRINTF("done\n");
}

//-----------interface with outside functions-----------------------------------
bool HectorSBPLFlipperControlEnv::InitializeEnv(int width, int height, const unsigned char* mapdata, double startx,
                                                double starty, double starttheta, bool startFlipper, double goalx, double goaly,
                                                double goaltheta, bool endFlipper, double goaltol_x, double goaltol_y,
                                                double goaltol_theta, const vector<sbpl_2Dpt_t> & perimeterptsV,
                                                double cellsize_m, double nominalvel_mpersecs,
                                                double timetoturn45degsinplace_secs, unsigned char obsthresh,
                                                const char* sMotPrimFile, double flipperOffsetX, double flipperOffsetY, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double inflationRadius)
{
    //debug/visualization
    vis_pc=cloud;

    inflationRadius_=inflationRadius;

    SBPL_PRINTF("env: initialize with width=%d height=%d start=%.3f %.3f %.3f "
                "goalx=%.3f %.3f %.3f cellsize=%.3f nomvel=%.3f timetoturn=%.3f, obsthresh=%d\n",
                width, height, startx, starty, starttheta, goalx, goaly, goaltheta, cellsize_m, nominalvel_mpersecs,
                timetoturn45degsinplace_secs, obsthresh);

    SBPL_PRINTF("NOTE: goaltol parameters currently unused\n");

    SBPL_PRINTF("perimeter has size=%d\n", (unsigned int)perimeterptsV.size());

    for (int i = 0; i < (int)perimeterptsV.size(); i++) {
        SBPL_PRINTF("perimeter(%d) = %.4f %.4f\n", i, perimeterptsV.at(i).x, perimeterptsV.at(i).y);
    }

    EnvNAVXYTHETAFLIPCfg.obsthresh = obsthresh;
    EnvNAVXYTHETAFLIPCfg.flipperOffsetX=flipperOffsetX;
    EnvNAVXYTHETAFLIPCfg.flipperOffsetY=flipperOffsetY;

    //TODO - need to set the tolerance as well

    SetConfiguration(width, height, mapdata, CONTXY2DISC(startx, cellsize_m), CONTXY2DISC(starty, cellsize_m),
                     ContTheta2Disc(starttheta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs), startFlipper, CONTXY2DISC(goalx, cellsize_m),
                     CONTXY2DISC(goaly, cellsize_m), ContTheta2Disc(goaltheta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs), endFlipper,
                     cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs, perimeterptsV);

    if (sMotPrimFile != NULL) {
        FILE* fMotPrim = fopen(sMotPrimFile, "r");
        if (fMotPrim == NULL) {
            SBPL_ERROR("ERROR: unable to open %s\n", sMotPrimFile);
            throw new SBPL_Exception();
        }

        if (ReadMotionPrimitives(fMotPrim) == false) {
            SBPL_ERROR("ERROR: failed to read in motion primitive file\n");
            throw new SBPL_Exception();
        }
        fclose(fMotPrim);
    }

    if (EnvNAVXYTHETAFLIPCfg.mprimV.size() != 0) {
        InitGeneral(&EnvNAVXYTHETAFLIPCfg.mprimV);
    }
    else
        InitGeneral( NULL);

    return true;
}

bool HectorSBPLFlipperControlEnv::InitGeneral(vector<SBPL_xythetaflip_mprimitive>* motionprimitiveV)
{
    //Initialize other parameters of the environment
    InitializeEnvConfig(motionprimitiveV);

    //initialize Environment
    InitializeEnvironment();

    //pre-compute heuristics
    ComputeHeuristicValues();

    return true;
}

bool HectorSBPLFlipperControlEnv::InitializeMDPCfg(MDPConfig *MDPCfg)
{
    //initialize MDPCfg with the start and goal ids
    MDPCfg->goalstateid = EnvNAVXYTHETAFLIP.goalstateid;
    MDPCfg->startstateid = EnvNAVXYTHETAFLIP.startstateid;

    return true;
}

void HectorSBPLFlipperControlEnv::PrintHeuristicValues()
{
#ifndef ROS
    const char* heur = "heur.txt";
#endif
    FILE* fHeur = SBPL_FOPEN(heur, "w");
    if (fHeur == NULL) {
        SBPL_ERROR("ERROR: could not open debug file to write heuristic\n");
        throw new SBPL_Exception();
    }
    SBPL2DGridSearch* grid2Dsearch = NULL;

    for (int i = 0; i < 2; i++) {
        if (i == 0 && grid2Dsearchfromstart != NULL) {
            grid2Dsearch = grid2Dsearchfromstart;
            SBPL_FPRINTF(fHeur, "start heuristics:\n");
        }
        else if (i == 1 && grid2Dsearchfromgoal != NULL) {
            grid2Dsearch = grid2Dsearchfromgoal;
            SBPL_FPRINTF(fHeur, "goal heuristics:\n");
        }
        else
            continue;

        for (int y = 0; y < EnvNAVXYTHETAFLIPCfg.EnvHeight_c; y++) {
            for (int x = 0; x < EnvNAVXYTHETAFLIPCfg.EnvWidth_c; x++) {
                if (grid2Dsearch->getlowerboundoncostfromstart_inmm(x, y) < INFINITECOST)
                    SBPL_FPRINTF(fHeur, "%5d ", grid2Dsearch->getlowerboundoncostfromstart_inmm(x, y));
                else
                    SBPL_FPRINTF(fHeur, "XXXXX ");
            }
            SBPL_FPRINTF(fHeur, "\n");
        }
    }
    SBPL_FCLOSE(fHeur);
}

const EnvNAVXYTHETAFLIPConfig_t* HectorSBPLFlipperControlEnv::GetEnvNavConfig()
{
    return &EnvNAVXYTHETAFLIPCfg;
}

bool HectorSBPLFlipperControlEnv::UpdateCost(int x, int y, unsigned char newcost)
{
#if DEBUG
    //SBPL_FPRINTF(fDeb, "Cost updated for cell %d %d from old cost=%d to new cost=%d\n", x,y,EnvNAVXYTHETALATCfg.Grid2D[x][y], newcost);
#endif

    EnvNAVXYTHETAFLIPCfg.Grid2D[x][y] = newcost;

    bNeedtoRecomputeStartHeuristics = true;
    bNeedtoRecomputeGoalHeuristics = true;

    return true;
}

bool HectorSBPLFlipperControlEnv::SetMap(const unsigned char* mapdata)
{
    int xind = -1, yind = -1;

    for (xind = 0; xind < EnvNAVXYTHETAFLIPCfg.EnvWidth_c; xind++) {
        for (yind = 0; yind < EnvNAVXYTHETAFLIPCfg.EnvHeight_c; yind++) {
            EnvNAVXYTHETAFLIPCfg.Grid2D[xind][yind] = mapdata[xind + yind * EnvNAVXYTHETAFLIPCfg.EnvWidth_c];
        }
    }

    bNeedtoRecomputeStartHeuristics = true;
    bNeedtoRecomputeGoalHeuristics = true;

    return true;
}

void HectorSBPLFlipperControlEnv::PrintTimeStat(FILE* fOut)
{
#if TIME_DEBUG
    SBPL_FPRINTF(fOut, "time3_addallout = %f secs, time_gethash = %f secs, time_createhash = %f secs, "
                 "time_s = %f\n",
                 time3_addallout/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC,
                 time_createhash/(double)CLOCKS_PER_SEC, time_s/(double)CLOCKS_PER_SEC);
#endif
}

bool HectorSBPLFlipperControlEnv::IsObstacle(int x, int y)
{
#if DEBUG
    SBPL_FPRINTF(fDeb, "Status of cell %d %d is queried. Its cost=%d\n", x,y,EnvNAVXYTHETALATCfg.Grid2D[x][y]);
#endif

    return (EnvNAVXYTHETAFLIPCfg.Grid2D[x][y] >= EnvNAVXYTHETAFLIPCfg.obsthresh);
}

void HectorSBPLFlipperControlEnv::GetEnvParms(int *size_x, int *size_y, int* num_thetas, double* startx, double* starty, double* starttheta, bool* startFlipper,
                                              double* goalx, double* goaly, double* goaltheta, bool* endFlipper, double* cellsize_m,
                                              double* nominalvel_mpersecs, double* timetoturn45degsinplace_secs,
                                              unsigned char* obsthresh, std::vector<SBPL_xythetaflip_mprimitive>* motionprimitiveV)
{
    *size_x = EnvNAVXYTHETAFLIPCfg.EnvWidth_c;
    *size_y = EnvNAVXYTHETAFLIPCfg.EnvHeight_c;
    *num_thetas = EnvNAVXYTHETAFLIPCfg.NumThetaDirs;

    *startx = DISCXY2CONT(EnvNAVXYTHETAFLIPCfg.StartX_c, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    *starty = DISCXY2CONT(EnvNAVXYTHETAFLIPCfg.StartY_c, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    *starttheta = DiscTheta2Cont(EnvNAVXYTHETAFLIPCfg.StartTheta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);
    *startFlipper= EnvNAVXYTHETAFLIPCfg.Start_flipperFlag;
    *goalx = DISCXY2CONT(EnvNAVXYTHETAFLIPCfg.EndX_c, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    *goaly = DISCXY2CONT(EnvNAVXYTHETAFLIPCfg.EndY_c, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    *goaltheta = DiscTheta2Cont(EnvNAVXYTHETAFLIPCfg.EndTheta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);
    *endFlipper= EnvNAVXYTHETAFLIPCfg.End_flipperFlag;

    *cellsize_m = EnvNAVXYTHETAFLIPCfg.cellsize_m;
    *nominalvel_mpersecs = EnvNAVXYTHETAFLIPCfg.nominalvel_mpersecs;
    *timetoturn45degsinplace_secs = EnvNAVXYTHETAFLIPCfg.timetoturn45degsinplace_secs;

    *obsthresh = EnvNAVXYTHETAFLIPCfg.obsthresh;

    *motionprimitiveV = EnvNAVXYTHETAFLIPCfg.mprimV;
}

bool HectorSBPLFlipperControlEnv::PoseContToDisc(double px, double py, double pth, int &ix, int &iy, int &ith) const
{
    ix = CONTXY2DISC(px, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    iy = CONTXY2DISC(py, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    ith = ContTheta2Disc(pth, EnvNAVXYTHETAFLIPCfg.NumThetaDirs); // ContTheta2Disc() normalizes the angle
    return (pth >= -2 * PI_CONST) && (pth <= 2 * PI_CONST) && (ix >= 0) && (ix < EnvNAVXYTHETAFLIPCfg.EnvWidth_c) &&
            (iy >= 0) && (iy < EnvNAVXYTHETAFLIPCfg.EnvHeight_c);
}

bool HectorSBPLFlipperControlEnv::PoseDiscToCont(int ix, int iy, int ith, double &px, double &py, double &pth) const
{
    px = DISCXY2CONT(ix, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    py = DISCXY2CONT(iy, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    pth = normalizeAngle(DiscTheta2Cont(ith, EnvNAVXYTHETAFLIPCfg.NumThetaDirs));
    return (ith >= 0) && (ith < EnvNAVXYTHETAFLIPCfg.NumThetaDirs) && (ix >= 0) &&
            (ix < EnvNAVXYTHETAFLIPCfg.EnvWidth_c) && (iy >= 0) && (iy < EnvNAVXYTHETAFLIPCfg.EnvHeight_c);
}

unsigned char HectorSBPLFlipperControlEnv::GetMapCost(int x, int y)
{
    return EnvNAVXYTHETAFLIPCfg.Grid2D[x][y];
}

bool HectorSBPLFlipperControlEnv::SetEnvParameter(const char* parameter, int value)
{
    if (EnvNAVXYTHETAFLIP.bInitialized == true) {
        SBPL_ERROR("ERROR: all parameters must be set before initialization of the environment\n");
        return false;
    }

    SBPL_PRINTF("setting parameter %s to %d\n", parameter, value);

    if (strcmp(parameter, "cost_inscribed_thresh") == 0) {
        if (value < 0 || value > 255) {
            SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
            return false;
        }
        EnvNAVXYTHETAFLIPCfg.cost_inscribed_thresh = (unsigned char)value;
    }
    else if (strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0) {
        if (value < 0 || value > 255) {
            SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
            return false;
        }
        EnvNAVXYTHETAFLIPCfg.cost_possibly_circumscribed_thresh = value;
    }
    else if (strcmp(parameter, "cost_obsthresh") == 0) {
        if (value < 0 || value > 255) {
            SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
            return false;
        }
        EnvNAVXYTHETAFLIPCfg.obsthresh = (unsigned char)value;
    }
    else if (strcmp(parameter, "lowerStairsThrsh") == 0) {
        if (value < 0 || value > 255) {
            SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
            return false;
        }
        EnvNAVXYTHETAFLIPCfg.lowerStaisThresh = value;
    }
    else {
        SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
        return false;
    }

    return true;
}

int HectorSBPLFlipperControlEnv::GetEnvParameter(const char* parameter)
{
    if (strcmp(parameter, "cost_inscribed_thresh") == 0) {
        return (int)EnvNAVXYTHETAFLIPCfg.cost_inscribed_thresh;
    }
    else if (strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0) {
        return (int)EnvNAVXYTHETAFLIPCfg.cost_possibly_circumscribed_thresh;
    }
    else if (strcmp(parameter, "cost_obsthresh") == 0) {
        return (int)EnvNAVXYTHETAFLIPCfg.obsthresh;
    }
    else if (strcmp(parameter, "lowerStairsThrsh") == 0) {
        return (int)EnvNAVXYTHETAFLIPCfg.lowerStaisThresh;
    }
    else {
        SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
        throw new SBPL_Exception();
    }
}

void HectorSBPLFlipperControlEnv::GetCoordFromState(int stateID, int& x, int& y, int& theta, int &currentRobotHeight, bool &flipperFlag, bool &flipperInStairsDrivingPos) const
{
    EnvNAVXYTHETAFLIPHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    x = HashEntry->X;
    y = HashEntry->Y;
    theta = HashEntry->Theta;
    currentRobotHeight=HashEntry->currentRobotHeight;
    flipperFlag= HashEntry->flipperActuationFlag;
    flipperInStairsDrivingPos= HashEntry->flipperInStairsDrivingPos;
}

int HectorSBPLFlipperControlEnv::GetStateFromCoord(int x, int y, int theta, bool flipperFlag)
{
    EnvNAVXYTHETAFLIPHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(x, y, theta, flipperFlag)) == NULL) {
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta, flipperFlag);
    }
    return OutHashEntry->stateID;
}

void HectorSBPLFlipperControlEnv::GetActionsFromStateIDPath(vector<int>* stateIDPath,
                                                            vector<EnvNAVXYTHETAFLIPAction_t>* action_list)
{
    vector<EnvNAVXYTHETAFLIPAction_t*> actionV;
    vector<int> CostV;
    vector<int> SuccIDV;
    int targetx_c, targety_c, targettheta_c, target_robotHeight_c;
    int sourcex_c, sourcey_c, sourcetheta_c, source_robotHeight_c;
    bool source_flipperFlag, source_flipperInStairsDrivingPos;
    bool target_flipperFlag, target_flipperInStairsDrivingPos;

    SBPL_PRINTF("checks=%ld\n", checks);

    action_list->clear();

    for (int pind = 0; pind < (int)(stateIDPath->size()) - 1; pind++) {
        int sourceID = stateIDPath->at(pind);
        int targetID = stateIDPath->at(pind + 1);

        //get successors and pick the target via the cheapest action
        SuccIDV.clear();
        CostV.clear();
        actionV.clear();
        GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);

        int bestcost = INFINITECOST;
        int bestsind = -1;

        for (int sind = 0; sind < (int)SuccIDV.size(); sind++) {
            if (SuccIDV[sind] == targetID && CostV[sind] <= bestcost) {
                bestcost = CostV[sind];
                bestsind = sind;
            }
        }
        if (bestsind == -1) {
            SBPL_ERROR("ERROR: successor not found for transition:\n");
            GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, source_robotHeight_c, source_flipperFlag, source_flipperInStairsDrivingPos);
            GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c, target_robotHeight_c, target_flipperFlag, target_flipperInStairsDrivingPos);
            SBPL_PRINTF("%d %d %d %i-> %d %d %d %i\n", sourcex_c, sourcey_c, sourcetheta_c, source_flipperFlag, targetx_c, targety_c,
                        targettheta_c, target_flipperFlag);
            throw new SBPL_Exception();
        }

#if DEBUG
        SBPL_FPRINTF(fDeb, "Start: %.3f %.3f %.3f Target: %.3f %.3f %.3f Prim ID, Start Theta: %d %d\n",
                     sourcex_c, sourcey_c, sourcetheta_c,
                     targetx_c, targety_c, targettheta_c,
                     actionV[bestsind]->aind, actionV[bestsind]->starttheta);
#endif

        action_list->push_back(*(actionV[bestsind]));
    }
}

void HectorSBPLFlipperControlEnv::ConvertStateIDPathintoXYThetaFlipperPath(vector<int>* stateIDPath,
                                                                           vector<sbpl_xy_theta_flip_pt_t>* xythetaPath)
{
    SBPL_PRINTF("convert ID path to worldPath");
    vector<EnvNAVXYTHETAFLIPAction_t*> actionV;
    vector<int> CostV;
    vector<int> SuccIDV;
    int targetx_c, targety_c, targettheta_c, target_robotHeight_c;
    int sourcex_c, sourcey_c, sourcetheta_c, source_robotHeight_c;
    bool source_flipperFlag, source_flipperInStairsDrivingPos;
    bool target_flipperFlag, target_flipperInStairsDrivingPos;


//    //TODO::remove after testing===========================
//    for (int i = 0; i < stateIDPath->size(); i++) {
//        int sourceID = stateIDPath->at(i);

//        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, source_robotHeight_c, source_flipperFlag, source_flipperInStairsDrivingPos);

//        SBPL_PRINTF("%d %d %d %i robotHeight: %d  flipper in StairsDrivingPos: %d\n", sourcex_c, sourcey_c, sourcetheta_c, source_flipperFlag, source_robotHeight_c, source_flipperInStairsDrivingPos);
//    }
//    //=====================================================





    SBPL_PRINTF("checks=%ld\n", checks);

    xythetaPath->clear();

#if DEBUG
    SBPL_FPRINTF(fDeb, "converting stateid path into coordinates:\n");
#endif

    for (int pind = 0; pind < (int)(stateIDPath->size()) - 1; pind++) {
        int sourceID = stateIDPath->at(pind);
        int targetID = stateIDPath->at(pind + 1);

#if DEBUG
        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
#endif

        //get successors and pick the target via the cheapest action
        SuccIDV.clear();
        CostV.clear();
        actionV.clear();
        GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);

        int bestcost = INFINITECOST;
        int bestsind = -1;

#if false
        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, source_robotHeight_c, source_flipperFlag);
        GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c, target_robotHeight_c, target_flipperFlag);
        SBPL_PRINTF("%d %d %d %i-> %d %d %d %i\n", sourcex_c, sourcey_c, sourcetheta_c, source_flipperFlag, targetx_c, targety_c,
                    targettheta_c, target_flipperFlag);
#endif

        for (int sind = 0; sind < (int)SuccIDV.size(); sind++) {

#if false
            int x_c, y_c, theta_c;
            bool flipper_c;
            GetCoordFromState(SuccIDV[sind], x_c, y_c, theta_c, flipper_c);
            SBPL_PRINTF("succ: %d %d %d %i\n", x_c, y_c, theta_c, flipper_c);

            if (SuccIDV[sind] == targetID){
                SBPL_PRINTF("get transition");
            }
#endif

            if (SuccIDV[sind] == targetID && CostV[sind] <= bestcost) {
                bestcost = CostV[sind];
                bestsind = sind;
            }
        }
        if (bestsind == -1) {
            SBPL_ERROR("ERROR: successor not found for transition:\n");
            GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, source_robotHeight_c, source_flipperFlag, source_flipperInStairsDrivingPos);
            GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c, target_robotHeight_c, target_flipperFlag, target_flipperInStairsDrivingPos);
            SBPL_PRINTF("%d %d %d %i-> %d %d %d %i\n", sourcex_c, sourcey_c, sourcetheta_c, source_flipperFlag, targetx_c, targety_c,
                        targettheta_c, target_flipperFlag);
            throw new SBPL_Exception();
        }

        //now push in the actual path
        int sourcex_c, sourcey_c, sourcetheta_c;
        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, source_robotHeight_c, source_flipperFlag, source_flipperInStairsDrivingPos);
        double sourcex, sourcey;
        sourcex = DISCXY2CONT(sourcex_c, EnvNAVXYTHETAFLIPCfg.cellsize_m);
        sourcey = DISCXY2CONT(sourcey_c, EnvNAVXYTHETAFLIPCfg.cellsize_m);
        int robotHeight= EnvNAVXYTHETAFLIPCfg.Grid2D[sourcex_c][sourcey_c];
        //TODO - when there are no motion primitives we should still print source state
        for (int ipind = 0; ipind < ((int)actionV[bestsind]->intermptV.size()) - 1; ipind++) {
            //translate appropriately
            sbpl_xy_theta_flip_pt_t pushback;
            sbpl_xy_theta_pt_t intermpt = actionV[bestsind]->intermptV[ipind];

            intermpt.x += sourcex;
            intermpt.y += sourcey;

            pushback.x= intermpt.x;
            pushback.y= intermpt.y;
            pushback.theta=intermpt.theta;
            pushback.robotHeight=robotHeight;
            if(isFlipperAction(actionV[bestsind])){
                pushback.flipperFlag=true;
            }else{
                pushback.flipperFlag=false;
            }

#if DEBUG
            int nx = CONTXY2DISC(intermpt.x, EnvNAVXYTHETALATCfg.cellsize_m);
            int ny = CONTXY2DISC(intermpt.y, EnvNAVXYTHETALATCfg.cellsize_m);
            SBPL_FPRINTF(fDeb, "%.3f %.3f %.3f (%d %d %d cost=%d) ",
                         intermpt.x, intermpt.y, intermpt.theta,
                         nx, ny,
                         ContTheta2Disc(intermpt.theta, EnvNAVXYTHETALATCfg.NumThetaDirs), EnvNAVXYTHETALATCfg.Grid2D[nx][ny]);
            if(ipind == 0) SBPL_FPRINTF(fDeb, "first (heur=%d)\n", GetStartHeuristic(sourceID));
            else SBPL_FPRINTF(fDeb, "\n");
#endif

            //store
            //            ROS_INFO("x: %f, y: %f, theta: %f, flipper:%i", pushback.x, pushback.y, pushback.theta, pushback.flipperFlag);
            xythetaPath->push_back(pushback);
            if(isFlipperAction(actionV[bestsind])){
                //                ROS_INFO("flipper action in rebuild plan");
                ipind=((int)actionV[bestsind]->intermptV.size());
            }
        }
    }
}

//returns the stateid if success, and -1 otherwise
int HectorSBPLFlipperControlEnv::SetGoal(double x_m, double y_m, double theta_rad, bool flipperFlag)
{
    int x = CONTXY2DISC(x_m, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    int y = CONTXY2DISC(y_m, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    int theta = ContTheta2Disc(theta_rad, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

    bool possibleStair= PossibleStairsAtStartGoal(x, y, theta, EnvNAVXYTHETAFLIPCfg.Grid2D[x][y]);
    bool validStartGoalPos= ValidStartGoalPosition(x, y, theta);

    if (!validStartGoalPos) {
        SBPL_PRINTF("WARNING: goal configuration is invalid\n");
        return -1;
    }

    if (100 == EnvNAVXYTHETAFLIPCfg.Grid2D[x][y]) {
        SBPL_PRINTF("WARNING: goal configuration is invalid\n");
        return -1;
    }

    if(possibleStair){
        SBPL_PRINTF("WARNING: goal configuration is on stairs => flipper == 1\n");
        flipperFlag=true;
    }

    if (!IsWithinMapCell(x, y)) {
        SBPL_ERROR("ERROR: trying to set a goal cell %d %d that is outside of map\n", x, y);
        return -1;
    }

    SBPL_PRINTF("env: setting goal to %.3f %.3f %.3f (%d %d %d %i)\n", x_m, y_m, theta_rad, x, y, theta, flipperFlag);

    EnvNAVXYTHETAFLIPHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(x, y, theta, flipperFlag)) == NULL) {
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta, flipperFlag);
    }
    OutHashEntry->currentRobotHeight=EnvNAVXYTHETAFLIPCfg.Grid2D[x][y];
    ROS_INFO("goal height: %i", OutHashEntry->currentRobotHeight);
    OutHashEntry->flipperInStairsDrivingPos=flipperFlag;

    //need to recompute start heuristics?
    if (EnvNAVXYTHETAFLIP.goalstateid != OutHashEntry->stateID) {
        bNeedtoRecomputeStartHeuristics = true; //because termination condition may not plan all the way to the new goal
        bNeedtoRecomputeGoalHeuristics = true; //because goal heuristics change
    }

    EnvNAVXYTHETAFLIP.goalstateid = OutHashEntry->stateID;

    EnvNAVXYTHETAFLIPCfg.EndX_c = x;
    EnvNAVXYTHETAFLIPCfg.EndY_c = y;
    EnvNAVXYTHETAFLIPCfg.EndTheta = theta;
    EnvNAVXYTHETAFLIPCfg.End_flipperFlag = flipperFlag;

    return EnvNAVXYTHETAFLIP.goalstateid;
}

//returns the stateid if success, and -1 otherwise
int HectorSBPLFlipperControlEnv::SetStart(double x_m, double y_m, double theta_rad, bool flipperFlag, bool &startOnStairs)
{
    int x = CONTXY2DISC(x_m, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    int y = CONTXY2DISC(y_m, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    int theta = ContTheta2Disc(theta_rad, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);


    bool possibleStair= PossibleStairsAtStartGoal(x, y, theta, EnvNAVXYTHETAFLIPCfg.Grid2D[x][y]);
    bool validStartGoalPos= ValidStartGoalPosition(x, y, theta);

    if (!validStartGoalPos) {
        SBPL_PRINTF("WARNING: start configuration is invalid\n");
        return -1;
    }

    if (100 == EnvNAVXYTHETAFLIPCfg.Grid2D[x][y]) {
        SBPL_PRINTF("WARNING: start configuration is invalid\n");
        return -1;
    }

    if(possibleStair){
        SBPL_PRINTF("WARNING: start configuration is on stairs => flipper == 1\n");
        flipperFlag=true;
        startOnStairs=flipperFlag;
    }

    if (!IsWithinMapCell(x, y)) {
        SBPL_ERROR("ERROR: trying to set a start cell %d %d that is outside of map\n", x, y);
        return -1;
    }

    SBPL_PRINTF("env: setting start to %.3f %.3f %.3f (%d %d %d %i)\n", x_m, y_m, theta_rad, x, y, theta, flipperFlag);

    EnvNAVXYTHETAFLIPHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(x, y, theta, flipperFlag)) == NULL) {
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta, flipperFlag);
    }
    OutHashEntry->currentRobotHeight=EnvNAVXYTHETAFLIPCfg.Grid2D[x][y];
    OutHashEntry->flipperInStairsDrivingPos=flipperFlag;

    //need to recompute start heuristics?
    if (EnvNAVXYTHETAFLIP.startstateid != OutHashEntry->stateID) {
        bNeedtoRecomputeStartHeuristics = true;
        //because termination condition can be not all states TODO - make it dependent on term. condition
        bNeedtoRecomputeGoalHeuristics = true;
    }

    //set start
    EnvNAVXYTHETAFLIP.startstateid = OutHashEntry->stateID;
    EnvNAVXYTHETAFLIPCfg.StartX_c = x;
    EnvNAVXYTHETAFLIPCfg.StartY_c = y;
    EnvNAVXYTHETAFLIPCfg.StartTheta = theta;
    EnvNAVXYTHETAFLIPCfg.Start_flipperFlag = flipperFlag;

    return EnvNAVXYTHETAFLIP.startstateid;
}

void HectorSBPLFlipperControlEnv::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
    if(stateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal (2)\n");
        throw new SBPL_Exception();
    }
#endif

    if (fOut == NULL) fOut = stdout;

    EnvNAVXYTHETAFLIPHashEntry_t* HashEntry = StateID2CoordTable[stateID];

    if (stateID == EnvNAVXYTHETAFLIP.goalstateid && bVerbose) {
        SBPL_FPRINTF(fOut, "the state is a goal state\n");
    }

    if (bVerbose)
        SBPL_FPRINTF(fOut, "X=%d Y=%d Theta=%d FlipperFlag%i \n", HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->flipperActuationFlag);
    else
        SBPL_FPRINTF(fOut, "%.3f %.3f %.3f\n", DISCXY2CONT(HashEntry->X, EnvNAVXYTHETAFLIPCfg.cellsize_m),
                     DISCXY2CONT(HashEntry->Y, EnvNAVXYTHETAFLIPCfg.cellsize_m),
                     DiscTheta2Cont(HashEntry->Theta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs));
}

EnvNAVXYTHETAFLIPHashEntry_t* HectorSBPLFlipperControlEnv::GetHashEntry_lookup(int X, int Y, int Theta, bool flipperFlag)
{
    if (X < 0 || X >= EnvNAVXYTHETAFLIPCfg.EnvWidth_c || Y < 0 || Y >= EnvNAVXYTHETAFLIPCfg.EnvHeight_c || Theta < 0 ||
            Theta >= EnvNAVXYTHETAFLIPCfg.NumThetaDirs) return NULL;


    int index = XYTHETAFLIP2INDEX(X,Y,Theta, flipperFlag);

    return Coord2StateIDHashTable_lookup[index];
}

EnvNAVXYTHETAFLIPHashEntry_t* HectorSBPLFlipperControlEnv::GetHashEntry_hash(int X, int Y, int Theta, bool flipperFlag)
{
#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    int binid = GETHASHBIN(X, Y, Theta, flipperFlag);

#if DEBUG
    if ((int)Coord2StateIDHashTable[binid].size() > 5)
    {
        SBPL_FPRINTF(fDeb, "WARNING: Hash table has a bin %d (X=%d Y=%d) of size %d\n",
                     binid, X, Y, (int)Coord2StateIDHashTable[binid].size());

        PrintHashTableHist(fDeb);
    }
#endif

    //iterate over the states in the bin and select the perfect match
    vector<EnvNAVXYTHETAFLIPHashEntry_t*>* binV = &Coord2StateIDHashTable[binid];
    for (int ind = 0; ind < (int)binV->size(); ind++) {
        EnvNAVXYTHETAFLIPHashEntry_t* hashentry = binV->at(ind);
        if (hashentry->X == X && hashentry->Y == Y && hashentry->Theta == Theta) {
#if TIME_DEBUG
            time_gethash += clock()-currenttime;
#endif
            return hashentry;
        }
    }

#if TIME_DEBUG
    time_gethash += clock()-currenttime;
#endif

    return NULL;
}

EnvNAVXYTHETAFLIPHashEntry_t* HectorSBPLFlipperControlEnv::CreateNewHashEntry_lookup(int X, int Y, int Theta, bool flipperFlag)
{
    int i;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    EnvNAVXYTHETAFLIPHashEntry_t* HashEntry = new EnvNAVXYTHETAFLIPHashEntry_t;

    HashEntry->X = X;
    HashEntry->Y = Y;
    HashEntry->Theta = Theta;
    HashEntry->iteration = 0;
    HashEntry->currentRobotHeight=0;
    HashEntry->flipperActuationFlag=flipperFlag;
    HashEntry->flipperInStairsDrivingPos=false;

    HashEntry->stateID = StateID2CoordTable.size();

    //insert into the tables
    StateID2CoordTable.push_back(HashEntry);


    int index = XYTHETAFLIP2INDEX(X,Y,Theta, flipperFlag);


#if DEBUG
    if(Coord2StateIDHashTable_lookup[index] != NULL)
    {
        SBPL_ERROR("ERROR: creating hash entry for non-NULL hashentry\n");
        throw new SBPL_Exception();
    }
#endif

    Coord2StateIDHashTable_lookup[index] = HashEntry;

    //insert into and initialize the mappings
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }

    if (HashEntry->stateID != (int)StateID2IndexMapping.size() - 1) {
        SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
        throw new SBPL_Exception();
    }

#if TIME_DEBUG
    time_createhash += clock()-currenttime;
#endif

    return HashEntry;
}

EnvNAVXYTHETAFLIPHashEntry_t* HectorSBPLFlipperControlEnv::CreateNewHashEntry_hash(int X, int Y, int Theta, bool flipperFlag)
{
    int i;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    EnvNAVXYTHETAFLIPHashEntry_t* HashEntry = new EnvNAVXYTHETAFLIPHashEntry_t;

    HashEntry->X = X;
    HashEntry->Y = Y;
    HashEntry->Theta = Theta;
    HashEntry->iteration = 0;
    HashEntry->currentRobotHeight=0;
    HashEntry->flipperActuationFlag=flipperFlag;
    HashEntry->flipperInStairsDrivingPos=false;

    HashEntry->stateID = StateID2CoordTable.size();

    //insert into the tables
    StateID2CoordTable.push_back(HashEntry);

    //get the hash table bin
    i = GETHASHBIN(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->flipperActuationFlag);

    //insert the entry into the bin
    Coord2StateIDHashTable[i].push_back(HashEntry);

    //insert into and initialize the mappings
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }

    if (HashEntry->stateID != (int)StateID2IndexMapping.size() - 1) {
        SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
        throw new SBPL_Exception();
    }

#if TIME_DEBUG
    time_createhash += clock()-currenttime;
#endif

    return HashEntry;
}

void HectorSBPLFlipperControlEnv::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<
                                           EnvNAVXYTHETAFLIPAction_t*>* actionV /*=NULL*/){
    int aind;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();
    SuccIDV->reserve(EnvNAVXYTHETAFLIPCfg.actionwidth);
    CostV->reserve(EnvNAVXYTHETAFLIPCfg.actionwidth);
    if (actionV != NULL) {
        actionV->clear();
        actionV->reserve(EnvNAVXYTHETAFLIPCfg.actionwidth);
    }

    //goal state should be absorbing
    if (SourceStateID == EnvNAVXYTHETAFLIP.goalstateid) return;

    //get X, Y for the state
    EnvNAVXYTHETAFLIPHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];

    pcl::PointXYZ vis_point;
    vis_point.x=DISCXY2CONT(HashEntry->X, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    vis_point.y=DISCXY2CONT(HashEntry->Y, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    vis_point.z=0;
    vis_pc->points.push_back(vis_point);

    //iterate through actions
    bool flipperFlagInCurrentState=HashEntry->flipperActuationFlag;
    bool flipperInStarisDrivingPos= HashEntry->flipperInStairsDrivingPos;
    for (aind = 0; aind < EnvNAVXYTHETAFLIPCfg.actionwidth; aind++) {
        flipperFlagInCurrentState=HashEntry->flipperActuationFlag;
        flipperInStarisDrivingPos= HashEntry->flipperInStairsDrivingPos;
        EnvNAVXYTHETAFLIPAction_t* nav3daction = &EnvNAVXYTHETAFLIPCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];

        //do not use flipper here
        if(isFlipperAction(nav3daction)){
            continue;
        }

        int newX = HashEntry->X + nav3daction->dX;
        int newY = HashEntry->Y + nav3daction->dY;
        int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

        if (!IsValidCellFlipper(newX, newY, HashEntry->currentRobotHeight, HashEntry->flipperActuationFlag)){
            continue;
        }

        if (!IsValidRotation(newX, newY, HashEntry->Theta,  newTheta, HashEntry->currentRobotHeight)){
            continue;
        }

        bool possibleStairFront= IsPossibleStairsTransitionFront(HashEntry->X, HashEntry->Y, nav3daction->dX, nav3daction->dY,HashEntry->currentRobotHeight, 10, HashEntry->Theta, true);
        bool possibleStairBack= IsPossibleStairsTransitionFront(HashEntry->X, HashEntry->Y, nav3daction->dX, nav3daction->dY,HashEntry->currentRobotHeight, 10, HashEntry->Theta, false);

        bool robotOnStairs= RobotOnStairs(newX, newY, HashEntry->currentRobotHeight);
        bool possibleStairs= robotOnStairs || possibleStairBack ||possibleStairFront;
        bool rotationOnStairs= RotationOnStairs(HashEntry->X, HashEntry->Y, HashEntry->Theta, newTheta, HashEntry->currentRobotHeight);

        if(possibleStairs){
            if(HashEntry->Theta != newTheta){
                continue;
            }
        }

        if(HashEntry->flipperActuationFlag){
            if(HashEntry->Theta != newTheta){
                continue;
            }
            if(!possibleStairs){
                continue;
            }

            if(EnvNAVXYTHETAFLIPCfg.Grid2D[newX][newY] > HashEntry->currentRobotHeight || HashEntry->flipperInStairsDrivingPos){
                // robot have to go upwards
                // robot at bottom of stairs
                if(robotOnStairs && HashEntry->flipperInStairsDrivingPos==false){
                    flipperInStarisDrivingPos=true;
                    flipperFlagInCurrentState=false;
                }
                //robot at top of stairs
                if(HashEntry->flipperInStairsDrivingPos==true && (HashEntry->currentRobotHeight == EnvNAVXYTHETAFLIPCfg.Grid2D[EnvNAVXYTHETAFLIPCfg.EndX_c][EnvNAVXYTHETAFLIPCfg.EndY_c])){
                    flipperInStarisDrivingPos=false;
                    flipperFlagInCurrentState=false;
                }
            }
        }else{
            if(HashEntry->flipperInStairsDrivingPos){
                if(HashEntry->Theta != newTheta){
                    continue;
                }
            }
            if(rotationOnStairs){
                continue;
            }
            if(possibleStairs){
                continue;
            }
        }

        //get cost
        int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction, EnvNAVXYTHETAFLIPCfg.Grid2D[newX][newY]);
        if (cost >= INFINITECOST) {
            continue;
        }

        EnvNAVXYTHETAFLIPHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, flipperFlagInCurrentState)) == NULL) {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, flipperFlagInCurrentState);
        }

        OutHashEntry->currentRobotHeight=EnvNAVXYTHETAFLIPCfg.Grid2D[newX][newY];
        OutHashEntry->flipperInStairsDrivingPos= flipperInStarisDrivingPos;

        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
        if (actionV != NULL) actionV->push_back(nav3daction);

    }

    //possible staris,  try flipperAction
        for (aind = 0; aind < EnvNAVXYTHETAFLIPCfg.actionwidth; aind++) {
            EnvNAVXYTHETAFLIPAction_t* nav3daction = &EnvNAVXYTHETAFLIPCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];

            //continue if action is no flipper action
            if(!isFlipperAction(nav3daction)){
                continue;
            }

            int newX = HashEntry->X + nav3daction->dX;
            int newY = HashEntry->Y + nav3daction->dY;
            int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvNAVXYTHETAFLIPCfg.NumThetaDirs);

            bool flipperFlagInCurrentState=nav3daction->flipperActuationFlag;

            //same flippper action two times in a row are forbidden
            if(flipperFlagInCurrentState==HashEntry->flipperActuationFlag){
                continue;
            }

            if(HashEntry->flipperActuationFlag){
                //no turn while flipper actuation, could never be true, due to the fact only flipper actions reach this point
                if(HashEntry->Theta != newTheta){
                    continue;
                }
            }

            //get cost
            int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction, EnvNAVXYTHETAFLIPCfg.Grid2D[newX][newY]);
            if (cost >= INFINITECOST) {
                continue;
            }

            EnvNAVXYTHETAFLIPHashEntry_t* OutHashEntry;
            if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, flipperFlagInCurrentState)) == NULL) {
                //have to create a new entry
                OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, flipperFlagInCurrentState);
            }

            OutHashEntry->currentRobotHeight=EnvNAVXYTHETAFLIPCfg.Grid2D[newX][newY];
            OutHashEntry->flipperInStairsDrivingPos= HashEntry->flipperInStairsDrivingPos;;
            //ROS_INFO("flipper action inserted as possible succsessor");
            SuccIDV->push_back(OutHashEntry->stateID);
            CostV->push_back(cost);
            if (actionV != NULL) actionV->push_back(nav3daction);

        }
//    }
#if TIME_DEBUG
    time_getsuccs += clock()-currenttime;
#endif
}

bool HectorSBPLFlipperControlEnv::isFlipperAction(EnvNAVXYTHETAFLIPAction_t *action){
    //    if((action->dX==action->dY) && (action->starttheta == action->endtheta)){
    if((action->dX==0) &&  (action->dY==0) && (action->starttheta == action->endtheta)){
        return true;
    }else{
        return false;
    }
}

void HectorSBPLFlipperControlEnv::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
    //TODO- to support tolerance, need:
    // a) generate preds for goal state based on all possible goal state variable settings,
    // b) change goal check condition in gethashentry c) change
    //    getpredsofchangedcells and getsuccsofchangedcells functions

    int aind;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    //get X, Y for the state
    EnvNAVXYTHETAFLIPHashEntry_t* HashEntry = StateID2CoordTable[TargetStateID];

    pcl::PointXYZ vis_point;
    vis_point.x=DISCXY2CONT(HashEntry->X, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    vis_point.y=DISCXY2CONT(HashEntry->Y, EnvNAVXYTHETAFLIPCfg.cellsize_m);
    vis_point.z=0;
    vis_pc->points.push_back(vis_point);

    //clear the successor array
    PredIDV->clear();
    CostV->clear();
    PredIDV->reserve(EnvNAVXYTHETAFLIPCfg.PredActionsV[(unsigned int)HashEntry->Theta].size());
    CostV->reserve(EnvNAVXYTHETAFLIPCfg.PredActionsV[(unsigned int)HashEntry->Theta].size());

    //iterate through actions
    bool flipperActionCouldbeUsefull=false;
    bool flipperFlagInCurrentState=HashEntry->flipperActuationFlag;
    vector<EnvNAVXYTHETAFLIPAction_t*>* actionsV = &EnvNAVXYTHETAFLIPCfg.PredActionsV[(unsigned int)HashEntry->Theta];
    for (aind = 0; aind < (int)EnvNAVXYTHETAFLIPCfg.PredActionsV[(unsigned int)HashEntry->Theta].size(); aind++) {
        bool flipperInStarisDrivingPos= HashEntry->flipperInStairsDrivingPos;

        EnvNAVXYTHETAFLIPAction_t* nav3daction = actionsV->at(aind);

        //do not use flipper here
        if(isFlipperAction(nav3daction)){
            continue;
        }

        int predX = HashEntry->X - nav3daction->dX;
        int predY = HashEntry->Y - nav3daction->dY;
        int predTheta = nav3daction->starttheta;


        if (!IsValidCell(predX, predY, HashEntry->currentRobotHeight)){
            continue;
        }

        bool possibleStair= PossibleStairsCell(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->currentRobotHeight);
        bool robotOnStairs= RobotCompleteOnStairs(predX, predY, predTheta, EnvNAVXYTHETAFLIPCfg.Grid2D[predX][predY]);
        float scale=0.3;
        int footprint_max_height= getFootprintMaxHeightDistScaled(predX, predY, predTheta, scale);

        if(HashEntry->flipperActuationFlag){
            if(HashEntry->Theta != predTheta){
                continue;
            }
            if(!(possibleStair || robotOnStairs)){
                flipperActionCouldbeUsefull=true;
                continue;
            }
            // robot at bottom of stairs
            if(!(HashEntry->currentRobotHeight >= footprint_max_height) && robotOnStairs && HashEntry->flipperInStairsDrivingPos==false){
                flipperInStarisDrivingPos=true;
                flipperFlagInCurrentState=false;
            }
            //robot at top of stairs
            if((HashEntry->currentRobotHeight  >= footprint_max_height) && HashEntry->flipperInStairsDrivingPos){
                ROS_INFO("footprint_max_height: %i", footprint_max_height);
                ROS_INFO("corrospondin center height: %i", HashEntry->currentRobotHeight);
                flipperInStarisDrivingPos=false;
                flipperFlagInCurrentState=false;
            }
        }else{
            if(possibleStair || robotOnStairs){
                flipperActionCouldbeUsefull=true;
                continue;
            }
        }

        //get cost
        int cost = GetActionCost( HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction, HashEntry->currentRobotHeight);
        if (cost >= INFINITECOST) {
            continue;
        }

        EnvNAVXYTHETAFLIPHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(predX, predY, predTheta, flipperFlagInCurrentState)) == NULL) {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(predX, predY, predTheta, flipperFlagInCurrentState);
        }

        OutHashEntry->currentRobotHeight=EnvNAVXYTHETAFLIPCfg.Grid2D[predX][predY];
        OutHashEntry->flipperInStairsDrivingPos= flipperInStarisDrivingPos;

        PredIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
    }


    //possible staris,  try flipperAction
    if(flipperActionCouldbeUsefull){
        for (aind = 0; aind < EnvNAVXYTHETAFLIPCfg.actionwidth; aind++) {
            EnvNAVXYTHETAFLIPAction_t* nav3daction = actionsV->at(aind);

            //continue if action is no flipper action
            if(!isFlipperAction(nav3daction)){
                continue;
            }

            int predX = HashEntry->X - nav3daction->dX;
            int predY = HashEntry->Y - nav3daction->dY;
            int predTheta = nav3daction->starttheta;


            //skip the invalid cells
            if (!IsValidCell(predX, predY, HashEntry->currentRobotHeight)){
                continue;
            }

            if(!IsValidConfiguration(predX, predY, predTheta, HashEntry->currentRobotHeight)){
                continue;
            }

            bool flipperFlagInCurrentState=nav3daction->flipperActuationFlag;

            //same flippper action two times in a row are forbidden
            if(flipperFlagInCurrentState==HashEntry->flipperActuationFlag){
                continue;
            }

            if(HashEntry->flipperActuationFlag){
                //no turn while flipper actuation, could never be true, due to the fact only flipper actions reach this point
                if(HashEntry->Theta != predTheta){
                    continue;
                }
            }

            //get cost
            int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction, HashEntry->currentRobotHeight);
            if (cost >= INFINITECOST) {
                continue;
            }

            EnvNAVXYTHETAFLIPHashEntry_t* OutHashEntry;
            if ((OutHashEntry = (this->*GetHashEntry)(predX, predY, predTheta, flipperFlagInCurrentState)) == NULL) {
                //have to create a new entry
                OutHashEntry = (this->*CreateNewHashEntry)(predX, predY, predTheta, flipperFlagInCurrentState);
            }

            OutHashEntry->currentRobotHeight=EnvNAVXYTHETAFLIPCfg.Grid2D[predX][predY];
            OutHashEntry->flipperInStairsDrivingPos= HashEntry->flipperInStairsDrivingPos;

            PredIDV->push_back(OutHashEntry->stateID);
            CostV->push_back(cost);
        }
    }


#if TIME_DEBUG
    time_getsuccs += clock()-currenttime;
#endif
}

void HectorSBPLFlipperControlEnv::GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV,
                                                         vector<int> *preds_of_changededgesIDV)
{
    nav2dcell_t cell;
    sbpl_xy_theta_cell_t affectedcell;
    EnvNAVXYTHETAFLIPHashEntry_t* affectedHashEntry;

    //increment iteration for processing savings
    iteration++;

    for (int i = 0; i < (int)changedcellsV->size(); i++) {
        cell = changedcellsV->at(i);

        //now iterate over all states that could potentially be affected
        for (int sind = 0; sind < (int)affectedpredstatesV.size(); sind++) {
            affectedcell = affectedpredstatesV.at(sind);

            //translate to correct for the offset
            affectedcell.x = affectedcell.x + cell.x;
            affectedcell.y = affectedcell.y + cell.y;

            //insert only if it was actually generated independent from Flipper
            affectedHashEntry = (this->*GetHashEntry)(affectedcell.x, affectedcell.y, affectedcell.theta, false);
            if (affectedHashEntry != NULL && affectedHashEntry->iteration < iteration) {
                preds_of_changededgesIDV->push_back(affectedHashEntry->stateID);
                affectedHashEntry->iteration = iteration; //mark as already inserted
            }else{
                affectedHashEntry = (this->*GetHashEntry)(affectedcell.x, affectedcell.y, affectedcell.theta, true);
                if (affectedHashEntry != NULL && affectedHashEntry->iteration < iteration) {
                    preds_of_changededgesIDV->push_back(affectedHashEntry->stateID);
                    affectedHashEntry->iteration = iteration; //mark as already inserted
                }
            }
        }
    }
}

void HectorSBPLFlipperControlEnv::GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV,
                                                         vector<int> *succs_of_changededgesIDV)
{
    nav2dcell_t cell;
    sbpl_xy_theta_cell_t affectedcell;
    EnvNAVXYTHETAFLIPHashEntry_t* affectedHashEntry;

    SBPL_ERROR("ERROR: getsuccs is not supported currently\n");
    throw new SBPL_Exception();

    //increment iteration for processing savings
    iteration++;

    //TODO - check
    for (int i = 0; i < (int)changedcellsV->size(); i++) {
        cell = changedcellsV->at(i);

        //now iterate over all states that could potentially be affected
        for (int sind = 0; sind < (int)affectedsuccstatesV.size(); sind++) {
            affectedcell = affectedsuccstatesV.at(sind);

            //translate to correct for the offset
            affectedcell.x = affectedcell.x + cell.x;
            affectedcell.y = affectedcell.y + cell.y;

            //insert only if it was actually generated, independent from Flipper
            affectedHashEntry = (this->*GetHashEntry)(affectedcell.x, affectedcell.y, affectedcell.theta, false);
            if (affectedHashEntry != NULL && affectedHashEntry->iteration < iteration) {
                succs_of_changededgesIDV->push_back(affectedHashEntry->stateID);
                affectedHashEntry->iteration = iteration; //mark as already inserted
            }else{
                affectedHashEntry = (this->*GetHashEntry)(affectedcell.x, affectedcell.y, affectedcell.theta, true);
                if (affectedHashEntry != NULL && affectedHashEntry->iteration < iteration) {
                    succs_of_changededgesIDV->push_back(affectedHashEntry->stateID);
                    affectedHashEntry->iteration = iteration; //mark as already inserted
                }
            }
        }
    }
}

void HectorSBPLFlipperControlEnv::InitializeEnvironment()
{
    EnvNAVXYTHETAFLIPHashEntry_t* HashEntry;

    int maxsize = EnvNAVXYTHETAFLIPCfg.EnvWidth_c * EnvNAVXYTHETAFLIPCfg.EnvHeight_c * EnvNAVXYTHETAFLIPCfg.NumThetaDirs*2; // *2 <=> flipper true or false

    if (maxsize <= SBPL_XYTHETALAT_MAXSTATESFORLOOKUP) {
        SBPL_PRINTF("environment stores states in lookup table\n");

        Coord2StateIDHashTable_lookup = new EnvNAVXYTHETAFLIPHashEntry_t*[maxsize];
        for (int i = 0; i < maxsize; i++)
            Coord2StateIDHashTable_lookup[i] = NULL;
        GetHashEntry = &HectorSBPLFlipperControlEnv::GetHashEntry_lookup;
        CreateNewHashEntry = &HectorSBPLFlipperControlEnv::CreateNewHashEntry_lookup;

        //not using hash table
        HashTableSize = 0;
        Coord2StateIDHashTable = NULL;
    }
    else {
        SBPL_PRINTF("environment stores states in hashtable\n");

        //initialize the map from Coord to StateID
        HashTableSize = 4 * 1024 * 1024; //should be power of two
        Coord2StateIDHashTable = new vector<EnvNAVXYTHETAFLIPHashEntry_t*> [HashTableSize];
        GetHashEntry = &HectorSBPLFlipperControlEnv::GetHashEntry_hash;
        CreateNewHashEntry = &HectorSBPLFlipperControlEnv::CreateNewHashEntry_hash;

        //not using hash
        Coord2StateIDHashTable_lookup = NULL;
    }

    //initialize the map from StateID to Coord
    StateID2CoordTable.clear();

    //create start state
    if ((HashEntry = (this->*GetHashEntry)(EnvNAVXYTHETAFLIPCfg.StartX_c, EnvNAVXYTHETAFLIPCfg.StartY_c,
                                           EnvNAVXYTHETAFLIPCfg.StartTheta, EnvNAVXYTHETAFLIPCfg.Start_flipperFlag)) == NULL) {
        //have to create a new entry
        HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETAFLIPCfg.StartX_c, EnvNAVXYTHETAFLIPCfg.StartY_c,
                                                EnvNAVXYTHETAFLIPCfg.StartTheta, EnvNAVXYTHETAFLIPCfg.Start_flipperFlag);
        HashEntry->currentRobotHeight=EnvNAVXYTHETAFLIPCfg.Grid2D[EnvNAVXYTHETAFLIPCfg.StartX_c][EnvNAVXYTHETAFLIPCfg.StartY_c];
        HashEntry->flipperInStairsDrivingPos=EnvNAVXYTHETAFLIPCfg.Start_flipperFlag;
    }
    EnvNAVXYTHETAFLIP.startstateid = HashEntry->stateID;

    //create goal state
    if ((HashEntry = (this->*GetHashEntry)(EnvNAVXYTHETAFLIPCfg.EndX_c, EnvNAVXYTHETAFLIPCfg.EndY_c,
                                           EnvNAVXYTHETAFLIPCfg.EndTheta, EnvNAVXYTHETAFLIPCfg.End_flipperFlag)) == NULL) {
        //have to create a new entry
        HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETAFLIPCfg.EndX_c, EnvNAVXYTHETAFLIPCfg.EndY_c,
                                                EnvNAVXYTHETAFLIPCfg.EndTheta, EnvNAVXYTHETAFLIPCfg.End_flipperFlag);
    }
    EnvNAVXYTHETAFLIP.goalstateid = HashEntry->stateID;

    //initialized
    EnvNAVXYTHETAFLIP.bInitialized = true;
}

//examples of hash functions: map state coordinates onto a hash value
//#define GETHASHBIN(X, Y) (Y*WIDTH_Y+X)
//here we have state coord: <X1, X2, X3, X4>
unsigned int HectorSBPLFlipperControlEnv::GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int Theta, bool flipperFlag)
{
    return inthash(inthash(X1) + (inthash(X2) << 1) + (inthash(Theta) << 2) + (inthash(flipperFlag)<<3)) & (HashTableSize - 1);
}

void HectorSBPLFlipperControlEnv::PrintHashTableHist(FILE* fOut)
{
    int s0 = 0, s1 = 0, s50 = 0, s100 = 0, s200 = 0, s300 = 0, slarge = 0;

    for (int j = 0; j < HashTableSize; j++) {
        if ((int)Coord2StateIDHashTable[j].size() == 0)
            s0++;
        else if ((int)Coord2StateIDHashTable[j].size() < 5)
            s1++;
        else if ((int)Coord2StateIDHashTable[j].size() < 25)
            s50++;
        else if ((int)Coord2StateIDHashTable[j].size() < 50)
            s100++;
        else if ((int)Coord2StateIDHashTable[j].size() < 100)
            s200++;
        else if ((int)Coord2StateIDHashTable[j].size() < 400)
            s300++;
        else
            slarge++;
    }
    SBPL_FPRINTF(fOut, "hash table histogram: 0:%d, <5:%d, <25:%d, <50:%d, <100:%d, <400:%d, >400:%d\n", s0, s1, s50,
                 s100, s200, s300, slarge);
}

int HectorSBPLFlipperControlEnv::GetFromToHeuristic(int FromStateID, int ToStateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if(FromStateID >= (int)StateID2CoordTable.size()
            || ToStateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //get X, Y for the state
    EnvNAVXYTHETAFLIPHashEntry_t* FromHashEntry = StateID2CoordTable[FromStateID];
    EnvNAVXYTHETAFLIPHashEntry_t* ToHashEntry = StateID2CoordTable[ToStateID];

    //TODO - check if one of the gridsearches already computed and then use it.

    return (int)(HECTOR_COSTMULT * EuclideanDistance_m(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X,
                                                       ToHashEntry->Y) /
                 EnvNAVXYTHETAFLIPCfg.nominalvel_mpersecs);

}

int HectorSBPLFlipperControlEnv::GetGoalHeuristic(int stateID)
{
    return 0;
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    EnvNAVXYTHETAFLIPHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    //computes distances from start state that is grid2D, so it is EndX_c EndY_c
    int h2D = grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
    int hEuclid = (int)(HECTOR_COSTMULT * EuclideanDistance_m(HashEntry->X, HashEntry->Y,
                                                              EnvNAVXYTHETAFLIPCfg.EndX_c,
                                                              EnvNAVXYTHETAFLIPCfg.EndY_c));

    return (int)(((double)__max(h2D, hEuclid)) / EnvNAVXYTHETAFLIPCfg.nominalvel_mpersecs);

}

int HectorSBPLFlipperControlEnv::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    EnvNAVXYTHETAFLIPHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    int h2D = grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
    int hEuclid = (int)(HECTOR_COSTMULT * EuclideanDistance_m(EnvNAVXYTHETAFLIPCfg.StartX_c,
                                                              EnvNAVXYTHETAFLIPCfg.StartY_c, HashEntry->X,
                                                              HashEntry->Y));

    //define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D, hEuclid)) / EnvNAVXYTHETAFLIPCfg.nominalvel_mpersecs);
}

int HectorSBPLFlipperControlEnv::SizeofCreatedEnv()
{
    return (int)StateID2CoordTable.size();
}

const EnvNAVXYTHETAFLIPHashEntry_t*
HectorSBPLFlipperControlEnv::GetStateEntry(int state_id) const
{
    if (state_id >= 0 && state_id < (int)StateID2CoordTable.size()) {
        return StateID2CoordTable[state_id];
    }
    else {
        return NULL;
    }
}


bool HectorSBPLFlipperControlEnv::isGoal(int id){
    return EnvNAVXYTHETAFLIP.goalstateid == id;
}

bool HectorSBPLFlipperControlEnv::InitializeEnv(const char* sEnvFile){
    //implement this if the planner needs to print out EnvNAVXYTHETALAT. configuration

    SBPL_ERROR("ERROR in HectorSBPLFlipperControlEnv... function: initializeEnv(const char* sEnvFile) is undefined\n");
    throw new SBPL_Exception();
}


void HectorSBPLFlipperControlEnv::PrintEnv_Config(FILE* fOut)
{
    //implement this if the planner needs to print out EnvNAVXYTHETALAT. configuration

    SBPL_ERROR("ERROR in HectorSBPLFlipperControlEnv... function: PrintEnv_Config is undefined\n");
    throw new SBPL_Exception();
}

void HectorSBPLFlipperControlEnv::SetAllPreds(CMDPSTATE* state)
{
    //implement this if the planner needs access to predecessors

    SBPL_ERROR("ERROR in HectorSBPLFlipperControlEnv... function: SetAllPreds is undefined\n");
    throw new SBPL_Exception();
}

