 
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

#include "navigator.h"
#include "mex.h"
#include "class_handle.h"

// matlab function entry.  
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])  
{
  static int maximum_name_length = 32; 
  char function_name[maximum_name_length]; 

  // obtain the name of the member function being invoked.  
  mxGetString(prhs[0], function_name, maximum_name_length); 
  
  if (!strcmp(function_name, "new")) {

    double arguments[5]; 
    // make sure the input and output arguments are as expected.  
    if (nlhs != 1) { mexErrMsgTxt("Navigator.new: one output expected."); return; } 
    if (nrhs != 6) { mexErrMsgTxt("Navigator.new: five input arguments expected."); return; }  
    for (int index = 1; index < nrhs; index++) {
      if (!mxIsDouble(prhs[index]) || mxIsComplex(prhs[index]) || mxGetNumberOfElements(prhs[index]) != 1) {
        mexErrMsgTxt("Navigator.new: all input arguments expected to be double."); return; }
        arguments[index - 1] = *(mxGetPr(prhs[index]));
    }

    // cast the data to double and instantiate the navigator.  
    if (arguments[1] < arguments[0] || arguments[3] < arguments[2]) { 
      mexErrMsgTxt("Navigator.new: the arena is expected to have non-zero size."); return; } 

    plhs[0] = convertPtr2Mat<evl::Navigator>( 
      new evl::Navigator(arguments[0], arguments[1], arguments[2], arguments[3], arguments[4])); 
    return; // the navigator is instantiated here.  
  } 

  // delete (literally) the object.   
  if (!strcmp(function_name, "delete")) {
    if (nlhs != 0 || nrhs != 2) {
      mexWarnMsgTxt("Navigator.delete: unexpected arguments ignored."); return; } 
    destroyObject<evl::Navigator>(prhs[1]); // actually delete the object. 
    return; 
  }

  // all subsequent functions require an instantiated navigator. 
  evl::Navigator *instance = convertMat2Ptr<evl::Navigator>(prhs[1]); 

  if (!strcmp(function_name, "createObstacle")) {
    if (nlhs >= 2) { mexWarnMsgTxt("Navigator.createObstacle: at most one argument output expected."); return; } 
    if (nrhs != 5) { mexErrMsgTxt("Navigator.createObstacle: three input arguments expected."); return; }  
    for (int index = 2; index < nrhs; index++) 
      if (!mxIsDouble(prhs[index]) || mxIsComplex(prhs[index]) || mxGetNumberOfElements(prhs[index]) != 1) {
        mexErrMsgTxt("Navigator.createObstacle: all input arguments expected to be double."); return; }
 
    // actually invoke the function.    
    double x = *(mxGetPr(prhs[2]));
    double y = *(mxGetPr(prhs[3])); 
    double radius = *(mxGetPr(prhs[4]));
    int64_t identifier = instance->createObstacle(x, y, radius); 
  
    // allocate some storage using their designated allocator. 
    int64_t *dynamicData = static_cast<int64_t*>(mxCalloc(1, sizeof(int64_t)));
    *dynamicData = identifier; 

    // stuff the retrieved identifier to matlab compatible format.  
    plhs[0] = mxCreateNumericMatrix(0, 0, mxINT64_CLASS, mxREAL);  
    mxSetData(plhs[0], dynamicData);
    mxSetM(plhs[0], 1); 
    mxSetN(plhs[0], 1); 

    return;
  }

  if (!strcmp(function_name, "updateObstacle")) {
    if (nlhs >= 2) { mexWarnMsgTxt("Navigator.updateObstacle: at most one output argument expected."); return; } 
    if (nrhs != 5) { mexErrMsgTxt("Navigator.updateObstacle: three input arguments expected."); return; }  
    if (!mxIsInt64(prhs[2]) || mxIsComplex(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 1) {
      mexErrMsgTxt("Navigator.updateObstacle: the identifier must be an 64-bit integer."); return; } 
    for (int index = 3; index < nrhs; index++) 
      if (!mxIsDouble(prhs[index]) || mxIsComplex(prhs[index]) || mxGetNumberOfElements(prhs[index]) != 1) { 
        mexErrMsgTxt("Navigator.updateObstacle: the new x and y are expected to be double."); return; }

    int64_t identifier = *static_cast<int64_t*>(mxGetData(prhs[2]));
    double x = *(mxGetPr(prhs[3])); 
    double y = *(mxGetPr(prhs[4]));
    bool succeeded = instance->updateObstacle(identifier, x, y); 

    plhs[0] = mxCreateLogicalScalar(static_cast<mxLogical>(succeeded));  
    return; 
  }

  if (!strcmp(function_name, "removeObstacle")) {
    if (nlhs >= 2) { mexWarnMsgTxt("Navigator.removeObstacle: at most one output argument expected."); return; } 
    if (nrhs != 3) { mexErrMsgTxt("Navigator.removeObstacle: one input argument expected."); return; }  
    if (!mxIsInt64(prhs[2]) || mxIsComplex(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 1) {
      mexErrMsgTxt("Navigator.removeObstacle: the identifier must be an 64-bit integer."); return; } 

    int64_t identifier = *static_cast<int64_t*>(mxGetData(prhs[2]));
    bool succeeded = instance->removeObstacle(identifier); 

    plhs[0] = mxCreateLogicalScalar(static_cast<mxLogical>(succeeded));
    return;  
  }
  
  // yeah these two functions look really alike so let's handle them together. 
  if (!strcmp(function_name, "setStart") || !strcmp(function_name, "setGoal")) {
    if (nlhs != 0) { mexWarnMsgTxt("Navigator.setStart: zero output arguments expected."); return; } 
    if (nrhs != 4) { mexErrMsgTxt("Navigator.setStart: two input arguments expected."); return; }  
    for (int index = 2; index < nrhs; index++) 
      if (!mxIsDouble(prhs[index]) || mxIsComplex(prhs[index]) || mxGetNumberOfElements(prhs[index]) != 1) {
        mexErrMsgTxt("Navigator.setStart: all input arguments expected to be double."); return; }
  
    double x = *(mxGetPr(prhs[2]));
    double y = *(mxGetPr(prhs[3]));
    if (!strcmp(function_name, "setStart")) instance->setStart(x, y);
    if (!strcmp(function_name, "setGoal"))  instance->setGoal(x, y); 

    return;
  }

  if (!strcmp(function_name, "plan")) {
    if (nlhs >= 2) { mexWarnMsgTxt("Navigator.plan: at most one output argument expected."); return; }
    if (nrhs != 3) { mexErrMsgTxt("Navigator.plan: one input argument expected."); return; } 
    if (!mxIsDouble(prhs[2]) || mxIsComplex(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 1) {
      mexErrMsgTxt("Navigator.plan: input expected to be a double scalar."); return; } 
    
    double allotted_time = *(mxGetPr(prhs[2]));
    std::vector<std::pair<double, double> > path = instance->plan(allotted_time);  

    // there might be a variable number of path waypoints... 
    double *dynamicData = static_cast<double*>(mxCalloc(path.size() * 2, sizeof(double))); 
    for (int index = 0; index < path.size(); index++) {
      dynamicData[index] = path[index].first;  
      dynamicData[index + path.size()] = path[index].second; } 

    // i don't exactly understand why this is being done this way... just following examples.
    plhs[0] = mxCreateNumericMatrix(0, 0, mxDOUBLE_CLASS, mxREAL);  
    mxSetData(plhs[0], dynamicData);
    mxSetM(plhs[0], path.size());
    mxSetN(plhs[0], 2); 

    return; 
  }

  if (!strcmp(function_name, "getArena")) {
    if (nlhs >= 2) { mexWarnMsgTxt("Navigator.getArena: at most one output argument expected."); return; }
    if (nrhs >= 4) { mexErrMsgTxt("Navigator.getArena: at most one input argument expected"); return; }

    bool path_overlay = true; // haven't figured out a better way do/cast this. 
    if (nrhs == 3) {
      if (!mxIsLogical(prhs[2]) || mxIsComplex(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 1) {  
        mexErrMsgTxt("Navigator.getArena: input expected to be logical."); return; } 
      path_overlay = *static_cast<bool*>(mxGetLogicals(prhs[2])); 
    }

    // be sure that the arena has at least one row so we can access its begin(). 
    std::vector<std::vector<int> > arena = instance->getArena(path_overlay);
    if (arena.size() == 0) { mexWarnMsgTxt("Navigator.getArena: the arena is empty."); return; }
    int column_count = arena.size();
    int row_count = arena[0].size(); // i don't want to use an asterisk to dereference the iterator. 

    // allocate some storage and stuff our cost map and path in. 
    INT64_T *dynamicData = static_cast<INT64_T*>(mxCalloc(column_count * row_count, sizeof(INT64_T))); 
    for (int row = 0; row < row_count; row++)
      for (int column = 0; column < column_count; column++)
        dynamicData[column * row_count + row] = arena[column][row];  

    plhs[0] = mxCreateNumericMatrix(0, 0, mxINT64_CLASS, mxREAL);
    mxSetData(plhs[0], dynamicData);
    mxSetM(plhs[0], row_count);
    mxSetN(plhs[0], column_count); 
    
    return;
  }

  if (!strcmp(function_name, "getIdentifiers")) {
    if (nlhs >= 2) { mexWarnMsgTxt("Navigator.getIdentifiers: at most one output argument expected."); return; }
    if (nrhs >= 3) { mexWarnMsgTxt("Navigator.getIdentifiers: input arguments ignored."); return; }
    
    std::vector<int> identifiers = instance->getIdentifiers();
    INT64_T *dynamicData = static_cast<INT64_T*>(mxCalloc(identifiers.size(), sizeof(INT64_T))); 
    for (int index = 0; index < identifiers.size(); index++) 
      dynamicData[index] = identifiers[index]; 
   
    plhs[0] = mxCreateNumericMatrix(0, 0, mxINT64_CLASS, mxREAL);
    mxSetData(plhs[0], dynamicData);
    mxSetM(plhs[0], identifiers.size());
    mxSetN(plhs[0], 1);
  
    return;  
  }
}
