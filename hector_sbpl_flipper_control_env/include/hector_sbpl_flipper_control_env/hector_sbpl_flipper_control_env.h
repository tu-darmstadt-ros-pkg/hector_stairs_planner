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

#ifndef __HECTOR_FLIPPER_CONTROL_ENV_H_
#define __HECTOR_FLIPPER_CONTROL_ENV_H_

#include <cstdio>
#include <vector>
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/utils/utils.h>
#include <sbpl/config.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl_ros/point_cloud.h>

// Define to test against in client code. Signals that Set2DBlockSize and
// Set2DBucketSize are available in EnvironmentNAVXYTHETALATTICE
#define SBPL_CUSTOM_2D_OPTIONS 1

//eight-connected grid
#define HECTOR_FLIPPER_CONTROL_DXYWIDTH 8
#define HECTOR_FLIPPER_CONTROL_DEFAULTOBSTHRESH 254	//see explanation of the value below
//maximum number of states for storing them into lookup (as opposed to hash)
#define SBPL_XYTHETALATFLIP_MAXSTATESFORLOOKUP 100000000
//definition of theta orientations
//0 - is aligned with X-axis in the positive direction (1,0 in polar coordinates)
//theta increases as we go counterclockwise
//number of theta values - should be power of 2
#define HECTOR_FLIPPER_CONTROL_THETADIRS 16
//number of actions per x,y,theta state
//decrease, increase, same angle while moving plus decrease, increase angle while standing.
#define HECTOR_FLIPPER_CONTROL_DEFAULT_ACTIONWIDTH 5
#define HECTOR_FLIPPER_CONTROL_COSTMULT_MTOMM 1000

//cost for flipper actuatuon
// FILPPER COST
#define HECTOR_FLIPPER_ACTUATION_COST 1
#define HECTOR_COSTMULT 1000

class CMDPSTATE;
class MDPConfig;
class SBPL2DGridSearch;

typedef struct
{
    unsigned char aind; //index of the action (unique for given starttheta)
    char starttheta;
    char dX;
    char dY;
    char endtheta;
    unsigned int cost;
    bool flipperActuationFlag;
    std::vector<sbpl_2Dcell_t> intersectingcellsV;
    //start at 0,0,starttheta and end at endcell in continuous domain with half-bin less to account for 0,0 start
    std::vector<sbpl_xy_theta_pt_t> intermptV;
    //start at 0,0,starttheta and end at endcell in discrete domain
    std::vector<sbpl_xy_theta_cell_t> interm3DcellsV;
} EnvNAVXYTHETAFLIPAction_t;

typedef struct
{
    int stateID;
    int X;
    int Y;
    char Theta;
    bool flipperActuationFlag;
    bool flipperInStairsDrivingPos;
    int iteration;
    int currentRobotHeight;
} EnvNAVXYTHETAFLIPHashEntry_t;

typedef struct
{
    int motprimID;
    unsigned char starttheta_c;
    int additionalactioncostmult;
    bool flipperActuationFlag;
    sbpl_xy_theta_cell_t endcell;
    //intermptV start at 0,0,starttheta and end at endcell in continuous
    //domain with half-bin less to account for 0,0 start
    std::vector<sbpl_xy_theta_pt_t> intermptV;
} SBPL_xythetaflip_mprimitive;

//variables that dynamically change (e.g., array of states, ...)
typedef struct
{
    int startstateid;
    int goalstateid;

    bool bInitialized;

    //any additional variables
} EnvNAVXYTHETAFLIP_t;

//configuration parameters
typedef struct ENV_NAVXYTHETAFLIP_CONFIG
{
    int EnvWidth_c;
    int EnvHeight_c;
    int NumThetaDirs;
    int StartX_c;
    int StartY_c;
    bool Start_flipperFlag;
    int StartTheta;
    int EndX_c;
    int EndY_c;
    int EndTheta;
    bool End_flipperFlag;
    unsigned char** Grid2D;

    // the value at which and above which cells are obstacles in the maps sent from outside
    // the default is defined above
    unsigned char obsthresh;  //(<=> upperStairsTresh)

    // lower StairsThresh
    unsigned char lowerStaisThresh;

    // the value at which and above which until obsthresh (not including it)
    // cells have the nearest obstacle at distance smaller than or equal to
    // the inner circle of the robot. In other words, the robot is definitely
    // colliding with the obstacle, independently of its orientation
    // if no such cost is known, then it should be set to obsthresh (if center
    // of the robot collides with obstacle, then the whole robot collides with
    // it independently of its rotation)
    unsigned char cost_inscribed_thresh;

    // the value at which and above which until cost_inscribed_thresh (not including it) cells
    // **may** have a nearest osbtacle within the distance that is in between
    // the robot inner circle and the robot outer circle
    // any cost below this value means that the robot will NOT collide with any
    // obstacle, independently of its orientation
    // if no such cost is known, then it should be set to 0 or -1 (then no cell
    // cost will be lower than it, and therefore the robot's footprint will
    // always be checked)
    int cost_possibly_circumscribed_thresh; // it has to be integer, because -1 means that it is not provided.

    double nominalvel_mpersecs;
    double timetoturn45degsinplace_secs;
    double cellsize_m;

    double flipperOffsetX;
    double flipperOffsetY;

    int dXY[HECTOR_FLIPPER_CONTROL_DXYWIDTH][2];

    //array of actions, ActionsV[i][j] - jth action for sourcetheta = i
    EnvNAVXYTHETAFLIPAction_t** ActionsV;
    //PredActionsV[i] - vector of pointers to the actions that result in a state with theta = i
    std::vector<EnvNAVXYTHETAFLIPAction_t*>* PredActionsV;

    int actionwidth; //number of motion primitives
    std::vector<SBPL_xythetaflip_mprimitive> mprimV;

    std::vector<sbpl_2Dpt_t> FootprintPolygon;
} EnvNAVXYTHETAFLIPConfig_t;

//class sbpl_xy_theta_flip_pt_t : public sbpl_xy_theta_pt_t{
class sbpl_xy_theta_flip_pt_t {
public:
    sbpl_xy_theta_flip_pt_t(){
        x = 0;
        y = 0;
        theta = 0;
        robotHeight=0;
        flipperFlag=false;
    }

    sbpl_xy_theta_flip_pt_t(double x_, double y_, double theta_, bool flipperFlag_)
    {
        x = x_;
        y = y_;
        theta = theta_;
        flipperFlag=flipperFlag_;
    }

    bool operator==(const sbpl_xy_theta_flip_pt_t p) const
    {
        return x == p.x && y == p.y && theta == p.theta && flipperFlag == p.flipperFlag;
    }

    double x;
    double y;
    int robotHeight;
    double theta;
    bool flipperFlag;

};

namespace hector_sbpl_flipper_control_env{

class EnvNAVXYTHETAFLIP_InitParms
{
public:
    unsigned int numThetas;
    const unsigned char* mapdata;
    double startx;
    double starty;
    double starttheta;
    double goalx;
    double goaly;
    double goaltheta;
    double goaltol_x;
    double goaltol_y;
    double goaltol_theta;
};

/** \brief 3D (x,y,theta) planning using lattice-based graph problem. For
 *         general structure see comments on parent class DiscreteSpaceInformation
 *         For info on lattice-based planning used here, you can check out the paper:
 *         Maxim Likhachev and Dave Ferguson, " Planning Long Dynamically-Feasible
 *         Maneuvers for Autonomous Vehicles", IJRR'09
 */
class HectorSBPLFlipperControlEnv : public DiscreteSpaceInformation
{
public:
    HectorSBPLFlipperControlEnv();

    virtual ~HectorSBPLFlipperControlEnv();

    /**
     * \brief way to set up various parameters. For a list of parameters, see
     *        the body of the function - it is pretty straightforward
     */
    bool SetEnvParameter(const char* parameter, int value);

    /**
     * \brief returns the value of specific parameter - see function body for the list of parameters
     */
    int GetEnvParameter(const char* parameter);

    /**
     * \brief see comments on the same function in the parent class
     */
    bool InitializeMDPCfg(MDPConfig *MDPCfg);

    /**
     * \brief see comments on the same function in the parent class
     */
    int GetFromToHeuristic(int FromStateID, int ToStateID);

    /**
     * \brief see comments on the same function in the parent class
     */
    int GetGoalHeuristic(int stateID);

    /**
     * \brief see comments on the same function in the parent class
     */
    int GetStartHeuristic(int stateID);

    /** \brief depending on the search used, it may call GetSuccs function
      *         (for forward search) or GetPreds function (for backward search)
      *         or both (for incremental search). At least one of this functions should
      *         be implemented (otherwise, there will be no search to run) Some searches
      *         may also use SetAllActionsandAllOutcomes or SetAllPreds functions if they
      *         keep the pointers to successors (predecessors) but most searches do not
      *         require this, so it is not necessary to support this
      */
    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV){
        GetSuccs(SourceStateID, SuccIDV, CostV, NULL);
    }

    /**
      * \brief see comments for GetSuccs functon
      */
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);

    void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV,
                  std::vector<EnvNAVXYTHETAFLIPAction_t*>* actionV /*=NULL*/);
    /**
     * \brief see comments on the same function in the parent class
     */
    void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);

    /**
     * \brief see comments on the same function in the parent class
     */
    void EnsureHeuristicsUpdated(bool bGoalHeuristics);

    /**
     * \brief initialize environment. Gridworld is defined as matrix A of size width by height.
     *        So, internally, it is accessed as A[x][y] with x ranging from 0 to width-1 and and y from 0 to height-1
     *        Each element in A[x][y] is unsigned char. A[x][y] = 0 corresponds to
     *        fully traversable and cost is just Euclidean distance
     *        The cost of transition between two neighboring cells is
     *        EuclideanDistance*(max(A[sourcex][sourcey],A[targetx][targety])+1)
     *        f A[x][y] >= obsthresh, then in the above equation it is assumed to be infinite.
     *        The cost also incorporates the length of a motion primitive and its cost_multiplier (see getcost function)
     *        mapdata is a pointer to the values of A. If it is null, then A is
     *        initialized to all zeros. Mapping is: A[x][y] = mapdata[x+y*width]
     *        start/goal are given by startx, starty, starttheta, goalx,goaly, goaltheta in meters/radians.
     *        If they are not known yet, just set them to 0. Later setgoal/setstart can be executed
     *        finally obsthresh defined obstacle threshold, as mentioned above
     *        goaltolerances are currently ignored
     *        for explanation of perimeter, see comments for InitializeEnv function that reads all from file
     *        cellsize is discretization in meters
     *        nominalvel_mpersecs is assumed velocity of vehicle while moving forward in m/sec
     *        timetoturn45degsinplace_secs is rotational velocity in secs/45 degrees turn
     */
    bool InitializeEnv(int width, int height,
                       /** if mapdata is NULL the grid is initialized to all freespace */
                       const unsigned char* mapdata,
                       double startx, double starty, double starttheta, bool startFlipper,
                       double goalx, double goaly, double goaltheta, bool endFlipper,
                       double goaltol_x, double goaltol_y, double goaltol_theta,
                       const std::vector<sbpl_2Dpt_t>& perimeterptsV, double cellsize_m,
                       double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
                       unsigned char obsthresh, const char* sMotPrimFile, double flipperOffsetX,
                       double flipperOffsetY, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double inflationRadius);

    /**
     * \brief update the traversability of a cell<x,y>
     */
    bool UpdateCost(int x, int y, unsigned char newcost);

    /**
     * \brief re-setting the whole 2D map
     *        transform from linear array mapdata to the 2D matrix used internally: Grid2D[x][y] = mapdata[x+y*width]
     */
    bool SetMap(const unsigned char* mapdata);

    /**
     * \brief this function fill in Predecessor/Successor states of edges whose costs changed
     *        It takes in an array of cells whose traversability changed, and
     *        returns (in vector preds_of_changededgesIDV) the IDs of all
     *        states that have outgoing edges that go through the changed
     *        cells
     */
    virtual void GetPredsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV,
                                        std::vector<int> *preds_of_changededgesIDV);
    /**
     * \brief same as GetPredsofChangedEdges, but returns successor states.
     *        Both functions need to be present for incremental search
     */
    virtual void GetSuccsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV,
                                        std::vector<int> *succs_of_changededgesIDV);

    /**
     * returns true if cell is untraversable
     */
    bool IsObstacle(int x, int y);

    /**
     * \brief returns false if robot intersects obstacles or lies outside of
     *        the map. Note this is pretty expensive operation since it computes the
     *        footprint of the robot based on its x,y,theta
     */
    bool IsValidConfiguration(int X, int Y, int Theta, int currentRobotHeight);

    bool IsValidConfigurationWithFlipper(int X, int Y, int Theta, int currentRobotHeight);

    bool IsValidRotation(int sourceX, int sourceY, int sourceTheta, int newTheta, int currentRobotHeight);

    bool FlipperActionIsValid(int X, int Y, int Theta, int currentRobotHeight);

    bool RotationOnStairs(int sourceX, int sourceY, int sourceTheta, int newTheta, int currentRobotHeight);

    bool PossibleStairsCell(int X, int Y, int Theta, int currentRobotHeight);

    bool PossibleStairsAtStartGoal(int sourceX, int sourceY, int theta, int currentRobotHeight);
    bool ValidStartGoalPosition(int sourceX, int sourceY, int theta);

    bool PossibleStairsCellAtStartGoal(int X, int Y, int Theta, int currentRobotHeight);

    bool RobotCompleteOnStairs(int X, int Y, int Theta, int currentRobotHeight);

    int getFootprintMaxHeightDistScaled(int X, int Y, int Theta, float scale);

    bool isFlipperAction(EnvNAVXYTHETAFLIPAction_t *action);

    bool IsValidTransitionHeightDiff(int X, int Y, int currentRobotHeight, int flipper);

    /**
     * \brief returns environment parameters. Useful for creating a copy environment
     */
    void GetEnvParms(int *size_x, int *size_y, int* num_thetas, double* startx, double* starty, double* starttheta, bool* startFlipper,
                     double* goalx, double* goaly, double* goaltheta, bool* endFlipper, double* cellsize_m,
                     double* nominalvel_mpersecs, double* timetoturn45degsinplace_secs,
                     unsigned char* obsthresh, std::vector<SBPL_xythetaflip_mprimitive>* motionprimitiveV);

    /**
     * \brief get internal configuration data structure
     */
    const EnvNAVXYTHETAFLIPConfig_t* GetEnvNavConfig();

    /**
     * \brief prints time statistics
     */
    void PrintTimeStat(FILE* fOut);

    /**
     * \brief returns the cost corresponding to the cell <x,y>
     */
    unsigned char GetMapCost(int x, int y);

    /**
     * \brief returns true if cell is within map
     */
    bool IsWithinMapCell(int X, int Y);

    /**
     * \brief Transform a pose into discretized form. The angle 'pth' is
     *        considered to be valid if it lies between -2pi and 2pi (some
     *        people will prefer 0<=pth<2pi, others -pi<pth<=pi, so this
     *        compromise should suit everyone).
     *
     * \note Even if this method returns false, you can still use the
     *       computed indices, for example to figure out how big your map
     *       should have been.
     *
     * \return true if the resulting indices lie within the grid bounds
     *         and the angle was valid.
     */
    bool PoseContToDisc(double px, double py, double pth, int &ix, int &iy, int &ith) const;

    /** \brief Transform grid indices into a continuous pose. The computed
     *         angle lies within 0<=pth<2pi.
     *
     * \note Even if this method returns false, you can still use the
     *      computed indices, for example to figure out poses that lie
     *      outside of your current map.
     *
     * \return true if all the indices are within grid bounds.
     */
    bool PoseDiscToCont(int ix, int iy, int ith, double &px, double &py, double &pth) const;

    /**
     * \brief sets start in meters/radians
     */
    int SetStart(double x, double y, double theta, bool flipflipperFlag, bool &startOnStairs);

    /**
     * \brief sets goal in meters/radians
     */
    int SetGoal(double x, double y, double theta, bool flipperFlag);


    /**
     * \brief returns state coordinates of state with ID=stateID
     */
    void GetCoordFromState(int stateID, int& x, int& y, int& theta, int &currentRobotHeight, bool &flipperFlag, bool &flipperInStairsDrivingPos) const;

    /**
     * \brief returns stateID for a state with coords x,y,theta
     */
    int GetStateFromCoord(int x, int y, int theta, bool flipperFlag);

    /**
     * \brief returns the actions / motion primitives of the passed path.
     */
    void GetActionsFromStateIDPath(std::vector<int>* stateIDPath,
                                   std::vector<EnvNAVXYTHETAFLIPAction_t>* action_list);

    /** \brief converts a path given by stateIDs into a sequence of
     *         coordinates. Note that since motion primitives are short actions
     *         represented as a sequence of points,
     *         the path returned by this function contains much more points than the
     *         number of points in the input path. The returned coordinates are in
     *         meters,meters,radians
     */
    void ConvertStateIDPathintoXYThetaFlipperPath(std::vector<int>* stateIDPath,
                                                  std::vector<sbpl_xy_theta_flip_pt_t> *xythetaPath);

    /**
     * \brief prints state info (coordinates) into file
     */
    void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);

    /**
     * \brief see comments on the same function in the parent class
     */
    int SizeofCreatedEnv();

    const EnvNAVXYTHETAFLIPHashEntry_t* GetStateEntry(int state_id) const;

protected:
    virtual int GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETAFLIPAction_t* action, int currentRobotHeight);

    //member data
    EnvNAVXYTHETAFLIPConfig_t EnvNAVXYTHETAFLIPCfg;
    EnvNAVXYTHETAFLIP_t EnvNAVXYTHETAFLIP;
    std::vector<sbpl_xy_theta_cell_t> affectedsuccstatesV; //arrays of states whose outgoing actions cross cell 0,0
    std::vector<sbpl_xy_theta_cell_t> affectedpredstatesV; //arrays of states whose incoming actions cross cell 0,0
    int iteration;

    //2D search for heuristic computations
    bool bNeedtoRecomputeStartHeuristics; //set whenever grid2Dsearchfromstart needs to be re-executed
    bool bNeedtoRecomputeGoalHeuristics; //set whenever grid2Dsearchfromgoal needs to be re-executed
    SBPL2DGridSearch* grid2Dsearchfromstart; //computes h-values that estimate distances from start x,y to all cells
    SBPL2DGridSearch* grid2Dsearchfromgoal; //computes h-values that estimate distances to goal x,y from all cells

    //    virtual void ReadConfiguration(FILE* fCfg);

    void InitializeEnvConfig(std::vector<SBPL_xythetaflip_mprimitive>* motionprimitiveV);

    void SetConfiguration(int width, int height, const unsigned char* mapdata, int startx,
                          int starty, int starttheta, bool startFlipper, int goalx, int goaly, int goaltheta, bool endFlipper,
                          double cellsize_m, double nominalvel_mpersecs,
                          double timetoturn45degsinplace_secs,
                          const std::vector<sbpl_2Dpt_t> & robot_perimeterV);

    bool InitGeneral(std::vector<SBPL_xythetaflip_mprimitive>* motionprimitiveV);
    void PrecomputeActionswithCompleteMotionPrimitive(std::vector<SBPL_xythetaflip_mprimitive>* motionprimitiveV);

    void ComputeHeuristicValues();

    bool IsValidCell(int X, int Y, int currentRobotHeight);
    bool IsValidCellFlipper(int X, int Y, int currentRobotHeight, bool flipperflag);
    bool IsPossibleStairsTransitionFront(int X, int Y, int dX, int dY, int currentRobotHeight, int times, int theta, bool inFront);
    bool RobotOnStairs(int X, int Y, int currentRobotHeight);

    void CalculateFootprintForPose(sbpl_xy_theta_pt_t pose, std::vector<sbpl_2Dcell_t>* footprint);
    void CalculateFootprintForPose(sbpl_xy_theta_pt_t pose, std::vector<sbpl_2Dcell_t>* footprint,
                                   const std::vector<sbpl_2Dpt_t>& FootprintPolygon);
    void RemoveSourceFootprint(sbpl_xy_theta_pt_t sourcepose, std::vector<sbpl_2Dcell_t>* footprint);
    void RemoveSourceFootprint(sbpl_xy_theta_pt_t sourcepose, std::vector<sbpl_2Dcell_t>* footprint,
                               const std::vector<sbpl_2Dpt_t>& FootprintPolygon);
    bool isGoal(int id);

    double EuclideanDistance_m(int X1, int Y1, int X2, int Y2);

    void ComputeReplanningData();
    void ComputeReplanningDataforAction(EnvNAVXYTHETAFLIPAction_t* action);

    bool ReadMotionPrimitives(FILE* fMotPrims);
    bool ReadinMotionPrimitive(SBPL_xythetaflip_mprimitive* pMotPrim, FILE* fIn);
    bool ReadinCell(sbpl_xy_theta_cell_t* cell, FILE* fIn);
    bool ReadinPose(sbpl_xy_theta_pt_t* pose, FILE* fIn);

    void PrintHeuristicValues();

    //TODO::Debug/visualization
    pcl::PointCloud<pcl::PointXYZ>::Ptr vis_pc;

    double inflationRadius_;

    //hash table of size x_size*y_size. Maps from coords to stateId
    int HashTableSize;
    std::vector<EnvNAVXYTHETAFLIPHashEntry_t*>* Coord2StateIDHashTable;
    //vector that maps from stateID to coords
    std::vector<EnvNAVXYTHETAFLIPHashEntry_t*> StateID2CoordTable;

    EnvNAVXYTHETAFLIPHashEntry_t** Coord2StateIDHashTable_lookup;

    unsigned int GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Theta, bool flipperFlag);

    EnvNAVXYTHETAFLIPHashEntry_t* GetHashEntry_hash(int X, int Y, int Theta, bool flipperFlag);
    EnvNAVXYTHETAFLIPHashEntry_t* CreateNewHashEntry_hash(int X, int Y, int Theta, bool flipperFlag);
    EnvNAVXYTHETAFLIPHashEntry_t* GetHashEntry_lookup(int X, int Y, int Theta, bool flipperFlag);
    EnvNAVXYTHETAFLIPHashEntry_t* CreateNewHashEntry_lookup(int X, int Y, int Theta, bool flipperFlag);

    //pointers to functions
    EnvNAVXYTHETAFLIPHashEntry_t* (HectorSBPLFlipperControlEnv::*GetHashEntry)(int X, int Y, int Theta, bool flipperFlag);
    EnvNAVXYTHETAFLIPHashEntry_t* (HectorSBPLFlipperControlEnv::*CreateNewHashEntry)(int X, int Y, int Theta, bool flipperFlag);

    void InitializeEnvironment();

    void PrintHashTableHist(FILE* fOut);

    //not implemented in this class

    /**
      * \brief initialization environment from file (see .cfg files for examples)
      */
    virtual bool InitializeEnv(const char* sEnvFile);

    /**
      * \brief see comments for GetSuccs functon
      */
    virtual void SetAllPreds(CMDPSTATE* state);


    /**
      * \brief prints environment config file
      */
    virtual void PrintEnv_Config(FILE* fOut);
};
}

#endif
