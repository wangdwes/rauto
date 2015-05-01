 
/* Copyright (c) 2014-2015, Team Evolutus
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include <sbpl/headers.h>
#include <map>
#include <limits>
#include <vector>
#include <algorithm>
#include <iterator>
#include <iostream>

namespace evl
{
  class Navigator
  {
    /** \brief A simple class that represents a circular obstacle. */
    class Obstacle  
    {
    public:
      Obstacle(): x(0), y(0), radius(0) {}
      Obstacle(double _x, double _y, double _radius):
        x(_x), y(_y), radius(_radius) {}

      double x, y, radius;              // the parameters, 
      std::vector<nav2dcell_t> cells;   // and its occupied cells. 
    }; 

  public: 

    /** \brief A deconstructor that releases memory of member variables. */
    ~Navigator();  
    /** \brief A constructor that initializes a discretized arena. */
    Navigator(double x_lower_limit, double x_upper_limit,
              double y_lower_limit, double y_upper_limit, double granularity); 

    /** \brief Create a circular obstacle with specified parameters. */
    virtual int createObstacle(double x, double y, double radius);
    /** \brief Update the position of an obstacle. */ 
    virtual bool updateObstacle(int identifier, double x, double y); 
    /** \brief Remove an obstacle. */
    virtual bool removeObstacle(int identifier); 

    /** \brief Set the starting state in the world frame. */
    virtual void setStart(double x, double y); 
    /** \brief Set the goal state in the world frame. */ 
    virtual void setGoal(double x, double y);
    /** \brief Run the planner for some allotted time and retrieve the solution. */
    virtual std::vector<std::pair<double, double> > plan(double allotted_time); 
    /** \brief Retrieve the arena with optional most recently planned path overlay. */
    virtual std::vector<std::vector<int> > retrieveArenaSnapshot(bool path_overlay = true);

    /** \brief Convert x-coordinate from world frame to discretized arena frame. */ 
    inline int toArenaX(double x) const { return CONTXY2DISC(x - x_lower_limit_, granularity_); } 
    /** \brief Convert y-coordinate from world frame to discretized arena frame. */
    inline int toArenaY(double y) const { return CONTXY2DISC(y - y_lower_limit_, granularity_); }
    /** \brief Convert x-coordinate from the discretized arena frame to the world frame. */ 
    inline double toWorldX(int x) const { return DISCXY2CONT(x, granularity_) + x_lower_limit_; }
    /** \brief Convert y-coordinate from the discretized arena frame to the world frame. */
    inline double toWorldY(int y) const { return DISCXY2CONT(y, granularity_) + y_lower_limit_; } 

  protected: 

    /** \brief Prepare a list of cells that constitute a circular obstacle. */ 
    virtual std::vector<nav2dcell_t> prepareCells(double x, double y, double radius);
    /** \brief Update selected cells within the environment. */ 
    virtual std::vector<nav2dcell_t> updateEnvironment(const std::vector<nav2dcell_t> &cells, char delta); 
    /** \brief Notify the planner that the environment has changed. */ 
    virtual void notifyPlanner(const std::vector<nav2dcell_t> &changed_cells); 

    EnvironmentNAV2D *environment_;   //!< the planar environment. 
    SBPLPlanner *planner_;            //!< some generic planner.

    std::map<int, std::vector<nav2dcell_t> > obstacle_templates_;   //!< storage for templates. 
    std::map<int, Obstacle> obstacles_;                             //!< storage for obstacles. 
    std::vector<nav2dcell_t> most_recent_path_;                     //!< storage for most recent path. 

    int width_;               //!< the number of cells on the x-axis.  
    int height_;              //!< the number of cells on the y-axis. 
    int start_state_id_;      //!< the state id of the starting position.
    int goal_state_id_;       //!< the state id of the goal position.

    double x_lower_limit_;    //!< the lower bound of the x-coordinates.  
    double y_lower_limit_;    //!< the lower bound of the y-coordinates. 
    double x_upper_limit_;    //!< the upper bound of the x-coordinates. 
    double y_upper_limit_;    //!< the upper bound of the y-coordinates.
    double granularity_;      //!< the side length of a square cell in the cost map. 
    size_t obstacle_count_;   //!< an accumulative count of all obstacles in the history.  

  }; 
}

#endif
