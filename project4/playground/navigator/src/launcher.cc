 
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

int main(int argc, char *argv[])
{
  
  evl::Navigator navigator(-0.4, 0.4, -0.4, 0.4, 0.025); 
  int a = navigator.createObstacle(0.1, 0.1, 0.08);
  int b = navigator.createObstacle(0.1, 0.05, 0.1);
  navigator.updateObstacle(a, 0.05, -0.0); 
  navigator.setStart(-0.2, -0.2);
  navigator.setGoal(0.2, 0.2); 
  navigator.plan(2.0);

  std::vector<std::vector<int> > arena = navigator.getArena(); 
  for (int y = 0; y < arena.size(); y++) {
    for (int x = 0; x < arena.front().size(); x++) 
      std::cout << arena[x][y] << ','; 
    std::cout << std::endl; }

  int c = navigator.createObstacle(0, 0.05, 0.2);
  int d = navigator.createObstacle(0.4, 0.05, 0.2);
  navigator.plan(2.0); 

  arena = navigator.getArena(); 
  for (int y = 0; y < arena.size(); y++) {
    for (int x = 0; x < arena.front().size(); x++) 
      std::cout << arena[x][y] << ','; 
    std::cout << std::endl; }

  return 0;  
}  
