 
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

#include <navigator.h>

namespace evl
{

  Navigator::Navigator(double x_lower_limit, double x_upper_limit, 
    double y_lower_limit, double y_upper_limit, double granularity)
    : x_lower_limit_(x_lower_limit),
      x_upper_limit_(x_upper_limit),
      y_lower_limit_(y_lower_limit),  
      y_upper_limit_(y_upper_limit),
      granularity_(granularity)
  { 
    // size of the discretized arena. 
    width_ = toArenaX(x_upper_limit) + 1; 
    height_ = toArenaY(y_upper_limit) + 1; 
   
    // instantiate a simple planar environment with all spaces free. 
    // the four zeros are start and goal coordinates. 1 is the obstacle threshold.  
    environment_ = new EnvironmentNAV2D();
    environment_->InitializeEnv(width_, height_, NULL, 0, 0, 0, 0, 1);  

    // this has to be done before invoking plan() or planner throws exceptions.  
    setStart(0, 0); setGoal(0, 0); 

    // instantiate a planner with some weird parameters.  
    planner_ = new ADPlanner(environment_, false); 
    planner_->set_initialsolution_eps(3.0); 
    planner_->set_search_mode(false); 
  }

  Navigator::~Navigator()
  {
    delete planner_; 
    delete environment_; 
  }

  // Public interfaces for creating, updating and removing obstacles. To create
  // an obstacle, generate the corresponding list of cells, update the environment, 
  // notify the planner, and have it registered. To update an obstacle, first
  // retrieve that entry from the registry, find out about the old cells, figure
  // out what new cells are to be occupied, then find the difference incurred from
  // update and notification for optimal performance. Removal is just creation reversed. 

  int Navigator::createObstacle(double x, double y, double radius)
  { 
    // instantiate an obstacle and prepare the cells.  
    Obstacle instance(x, y, radius); 
    std::vector<nav2dcell_t> cells = prepareCells(x, y, radius);
    instance.cells.swap(cells); // a temporary variable must be created first. 

    // then update the environment and notify the planner. Isn't this elegant? 
    notifyPlanner(updateEnvironment(instance.cells, 1));   
    obstacles_[++obstacle_count_] = instance;
    return obstacle_count_; 
  }

  bool Navigator::updateObstacle(int identifier, double x, double y)
  {
    // unable to locate the obstacle? reject this request.  
    if (obstacles_.find(identifier) == obstacles_.end()) return false; 
    Obstacle &instance = obstacles_[identifier]; 

    // obtain the brand new cells.
    std::vector<nav2dcell_t> new_cells = prepareCells(x, y, instance.radius);

    // find the left and right differences and leave the union intact. 
    // this ensures that a minimum amount of cells are changed. 
    std::vector<nav2dcell_t> left_difference, right_difference; 
    std::set_difference(new_cells.begin(), new_cells.end(),
      instance.cells.begin(), instance.cells.end(), std::back_inserter(left_difference));
    std::set_difference(instance.cells.begin(), instance.cells.end(), 
      new_cells.begin(), new_cells.end(), std::back_inserter(right_difference)); 

    // update the environment, and obtain the cells that actually have changed.  
    std::vector<nav2dcell_t> left_changed = updateEnvironment(left_difference, +1);  
    std::vector<nav2dcell_t> right_changed = updateEnvironment(right_difference, -1);
    left_changed.insert(left_changed.end(), right_changed.begin(), right_changed.end());
    notifyPlanner(left_changed); // notify the planner. 
    
    instance.cells.swap(new_cells); instance.x = x; instance.y = y; 
    return true; 
  }

  bool Navigator::removeObstacle(int identifier)
  {
    // unable to locate the obstacle? reject this request.  
    if (obstacles_.find(identifier) == obstacles_.end()) return false; 
    Obstacle &instance = obstacles_[identifier]; 
  
    // update and environment and erase the entry from the registry. 
    notifyPlanner(updateEnvironment(instance.cells, -1));
    obstacles_.erase(identifier);
    return true;  
  }

  // After wasting an excessive amount of time pondering about discretization, I've
  // decided to simplify this problem by decoupling the position and the radius. This
  // would definitely introduce greater discretization error, but implementation would
  // be much easier, in addition to alleviated computational burden. Returning a
  // container is made possible with return-value optimization - no deep copy required. 

  std::vector<nav2dcell_t> Navigator::prepareCells(double x, double y, double radius)
  {
    int discretized_radius = CONTXY2DISC(radius, granularity_);    
    int discretized_radius_squared = discretized_radius * discretized_radius; 
  
    // If no obstacle of this radius has ever been created, we would have to create a new template.
    if (obstacle_templates_.find(discretized_radius) == obstacle_templates_.end()) {

      std::vector<nav2dcell_t> template_cells; // storage for the cell coordinates.
      for (int y = -discretized_radius; y <= discretized_radius; ++y)
        for (int x = -discretized_radius; x <= discretized_radius; ++x)
          if (x * x + y * y <= discretized_radius_squared) // cell within or on the circle?
            template_cells.push_back(nav2dcell_t(x, y)); // in which case save this cell. 

      // insert the generated template of cells into storage. 
      obstacle_templates_[discretized_radius] = template_cells; 
    } 

    // discretize the centroids. note how this is decoupled from the radius. 
    int x_centroid = toArenaX(x);  
    int y_centroid = toArenaY(y); 

    // make a deep copy of these template cells and apply an offset.  
    std::vector<nav2dcell_t> cells(obstacle_templates_[discretized_radius]); 
    for (int index = 0; index < cells.size(); index++) {
      cells[index].x += x_centroid; 
      cells[index].y += y_centroid; }

    return cells; // and finally return the prepared cells. 
  }

  // Update the environment, and populate a list of cells that become obstacles
  // or are no longer obstacles after this update. Since one individual cell
  // can be occupied by multiple obstacles, and we're using 'cost' to keep track
  // of how many obstacles are actually overlapping on this cell, a change in
  // cost does not necessarily imply the aforesaid transition. 

  std::vector<nav2dcell_t> Navigator::updateEnvironment(const std::vector<nav2dcell_t> &cells, char delta)
  {
    // changed in terms of the status of being an obstacle, not the cost.  
    std::vector<nav2dcell_t> changed_cells; 

    // iterate through all cells to be updated. 
    for (std::vector<nav2dcell_t>::const_iterator it = cells.begin(); it != cells.end(); ++it) {
      nav2dcell_t cell = *it; // i honestly don't like the asterisk. 

      // be sure that the cells are within boundary.  
      if (cell.x < 0 || cell.x >= width_ || cell.y < 0 || cell.y >= height_) continue; 
      unsigned char current_cost = environment_->GetMapCost(cell.x, cell.y); 
      unsigned char updated_cost = current_cost + static_cast<unsigned char>(delta); // yeah... 

      // the actual cost always have to be updated, unless delta is zero;
      // the cells are considered changed however, if and only if the criterion above is met.  
      environment_->UpdateCost(cell.x, cell.y, updated_cost); 
      if (current_cost == 0 || updated_cost == 0)
        changed_cells.push_back(nav2dcell_t(cell.x, cell.y)); 
    }
    
    return changed_cells; 
  }  

  // This should be invoked right after updating the environment. Although, consider
  // how expensive this step may be, it is not advised to call it very frequently. 
  // Only those cells that truly become obstacle, or are no longer obstacle, should
  // be passed to this member function to minimize unnecessary recomputation.  

  void Navigator::notifyPlanner(const std::vector<nav2dcell_t> &changed_cells) 
  {
    if (changed_cells.size() > 0) { // unlikely but still.
      ARAPlanner *araplanner = dynamic_cast<ARAPlanner*>(planner_);
      ADPlanner *adplanner = dynamic_cast<ADPlanner*>(planner_); 

      // proceed with different routines for incremental and non-incremental planners. 
      if (araplanner) araplanner->costs_changed(); 
      else if (adplanner) { std::vector<int> preds; // no idea what this does. 
        environment_->GetPredsofChangedEdges(&changed_cells, &preds);  
        adplanner->update_preds_of_changededges(&preds); 
      }
    }
  } 

  std::vector<std::pair<double, double> > Navigator::plan(double allotted_time) 
  {
    // plan using the current planner and retrieve the solution state ids. 
    std::vector<int> state_ids;  
    planner_->set_start(start_state_id_);
    planner_->set_goal(goal_state_id_); 
    planner_->replan(allotted_time, &state_ids); 

    // convert the state ids to human-readable coordinates.  
    int x, y; most_recent_path_.clear(); 
    std::vector<std::pair<double, double> > coordinates; 

    for (std::vector<int>::const_iterator it = state_ids.begin(); it != state_ids.end(); ++it) {
      environment_->GetCoordFromState(*it, x, y);
      most_recent_path_.push_back(nav2dcell_t(x, y)); // discrete coordinates for internel use. 
      coordinates.push_back(std::make_pair(toWorldX(x), toWorldY(y))); // continuous for output. 
    } return coordinates;  
  }

  // retrieve the current cost map and the most recently planned path. 
  std::vector<std::vector<int> > Navigator::getArena(bool path_overlay)
  {
    // initialize a data structure for our matrix. 
    std::vector<std::vector<int> > arena;
 
    // retrieve the map cost first from the environment.   
    for (int x = 0; x < width_; ++x) {
      std::vector<int> column; 
      for (int y = 0; y < height_; ++y)
        column.push_back(environment_->GetMapCost(x, y)); 
      arena.push_back(column); }

    if (path_overlay) // overlay the path if needed. 
      for (int index = 0; index < most_recent_path_.size(); index++)
        arena[most_recent_path_[index].x][most_recent_path_[index].y] = 0 - 1 - index; 

    return arena; 
  }

  // set the start and retrieve its corresponding state id. 
  inline void Navigator::setStart(double x, double y) {
    start_state_id_ = environment_->SetStart(toArenaX(x), toArenaY(y)); }

  // set the goal and retrieve its corresponding state id.  
  inline void Navigator::setGoal(double x, double y) {
    goal_state_id_ = environment_->SetGoal(toArenaX(x), toArenaY(y)); }


}
